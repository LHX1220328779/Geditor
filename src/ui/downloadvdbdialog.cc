#include "downloadvdbdialog.h"
#include "ui_downloadvdbdialog.h"

#include <QFileDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QHeaderView>

DownloadVdbDialog::DownloadVdbDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::DownloadVdbDialog), networkManager_(new QNetworkAccessManager(this)) {
  ui->setupUi(this);
  currentUrl_ = ui->urlEdit->text();
  
  // 设置表格属性
  ui->fileListWidget->setSelectionMode(QAbstractItemView::SingleSelection);
  ui->fileListWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->fileListWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui->fileListWidget->horizontalHeader()->setStretchLastSection(true);
  
  connect(ui->fileListWidget, &QTableWidget::itemSelectionChanged, this, [this]() {
    ui->downloadButton->setEnabled(!ui->fileListWidget->selectedItems().isEmpty());
  });

  refreshFileList();
}

DownloadVdbDialog::~DownloadVdbDialog() {
  delete ui;
}

void DownloadVdbDialog::on_refreshButton_clicked() {
  refreshFileList();
}

void DownloadVdbDialog::on_urlEdit_textChanged(const QString &text) {
  currentUrl_ = text;
}

void DownloadVdbDialog::on_downloadButton_clicked() {
  downloadSelectedFile();
}

void DownloadVdbDialog::refreshFileList() {
  // 清空现有数据
  ui->fileListWidget->clear();
  fileRecords_.clear();
  ui->downloadButton->setEnabled(false);
  
  // 设置表头
  ui->fileListWidget->setColumnCount(7);  // 增加到7列，包括序号、名称、备注、时间、UUID、状态、图层类型
  QStringList headers;
  headers << tr("序号") << tr("名称") << tr("备注") << tr("时间") << tr("UUID") << tr("状态") << tr("图层类型");
  ui->fileListWidget->setHorizontalHeaderLabels(headers);
  
  // 构建请求URL
  QUrl url(currentUrl_);
  QString path = url.path();
  if (!path.endsWith('/')) {
    path += '/';
  }
  path += "pipeline/anno/vdb/query";  // 修改为VDB查询路径
  url.setPath(path);
  
  // 发送POST请求
  QNetworkRequest request(url);
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  QJsonObject jsonData;  // 如果需要发送数据，可以在这里添加
  QJsonDocument doc(jsonData);
  currentReply_ = networkManager_->post(request, doc.toJson());
  
  connect(currentReply_, &QNetworkReply::finished, this, &DownloadVdbDialog::onFileListReceived);
  connect(currentReply_, &QNetworkReply::errorOccurred, this, &DownloadVdbDialog::onNetworkError);
  
  // 禁用按钮，防止重复请求
  ui->refreshButton->setEnabled(false);
  ui->urlEdit->setEnabled(false);
}

void DownloadVdbDialog::onFileListReceived() {
  // 重新启用按钮
  ui->refreshButton->setEnabled(true);
  ui->urlEdit->setEnabled(true);
  
  if (currentReply_->error() != QNetworkReply::NoError) {
    // 错误处理已在onNetworkError中完成
    currentReply_->deleteLater();
    currentReply_ = nullptr;
    return;
  }
  
  // 解析JSON响应
  QByteArray responseData = currentReply_->readAll();
  currentReply_->deleteLater();
  currentReply_ = nullptr;
  
  QJsonDocument jsonDoc = QJsonDocument::fromJson(responseData);
  if (jsonDoc.isNull() || !jsonDoc.isObject()) {
    QMessageBox::warning(this, tr("错误"), tr("无效的服务器响应"));
    return;
  }
  
  QJsonObject jsonObj = jsonDoc.object();
  if (!jsonObj.contains("data") || !jsonObj["data"].isArray()) {
    QMessageBox::warning(this, tr("错误"), tr("服务器响应中没有数据"));
    return;
  }
  
  QJsonArray dataArray = jsonObj["data"].toArray();
  if (dataArray.isEmpty()) {
    QMessageBox::information(this, tr("信息"), tr("没有可用的文件"));
    return;
  }
  
  // 准备表格
  ui->fileListWidget->setRowCount(dataArray.size());
  
  // 填充表格
  for (int i = 0; i < dataArray.size(); ++i) {
    QJsonObject fileObj = dataArray[i].toObject();
    
    FileRecord record;
    record.name = fileObj["name"].toString();
    record.remark = fileObj["remark"].toString();
    record.time = fileObj["time"].toString();
    record.uuid = fileObj["uuid"].toString();
    record.state = fileObj["state"].toString();  // 新增：状态字段
    record.layerType = fileObj["layerType"].toInt();  // 新增：图层类型
    
    fileRecords_[i] = record;
    
    // 添加序号列
    QTableWidgetItem *indexItem = new QTableWidgetItem(QString::number(i + 1));
    ui->fileListWidget->setItem(i, 0, indexItem);
    
    // 添加名称列
    QTableWidgetItem *nameItem = new QTableWidgetItem(record.name);
    ui->fileListWidget->setItem(i, 1, nameItem);
    
    // 添加备注列
    QTableWidgetItem *remarkItem = new QTableWidgetItem(record.remark);
    ui->fileListWidget->setItem(i, 2, remarkItem);
    
    // 添加时间列
    QTableWidgetItem *timeItem = new QTableWidgetItem(record.time);
    ui->fileListWidget->setItem(i, 3, timeItem);
    
    // 添加UUID列
    QTableWidgetItem *uuidItem = new QTableWidgetItem(record.uuid);
    ui->fileListWidget->setItem(i, 4, uuidItem);
    
    // 添加状态列
    QTableWidgetItem *stateItem = new QTableWidgetItem(record.state);
    ui->fileListWidget->setItem(i, 5, stateItem);
    
    // 添加图层类型列
    QTableWidgetItem *layerTypeItem = new QTableWidgetItem(getLayerTypeText(record.layerType));
    ui->fileListWidget->setItem(i, 6, layerTypeItem);
  }
  
  // 自动调整列宽以适应内容
  ui->fileListWidget->resizeColumnsToContents();
}

QString DownloadVdbDialog::getLayerTypeText(int layerType) const {
  switch (layerType) {
    case 0:
      return tr("全量地图");
    case 1:
      return tr("基础地图");
    case 2:
      return tr("业务地图");
    default:
      return tr("未知类型");
  }
}

void DownloadVdbDialog::downloadSelectedFile() {
  QList<QTableWidgetItem*> selectedItems = ui->fileListWidget->selectedItems();
  if (selectedItems.isEmpty()) return;

  int currentRow = selectedItems.first()->row();
  if (!fileRecords_.contains(currentRow)) return;

  const FileRecord& record = fileRecords_[currentRow];
  QString fileName = record.name + ".vdb";  // 修改为.vdb扩展名
  currentSavePath_ = QFileDialog::getSaveFileName(this, tr("保存文件"),
                                                fileName,
                                                tr("VDB文件 (*.vdb);;所有文件 (*.*)"));  // 修改为VDB文件类型
  
  if (currentSavePath_.isEmpty()) return;

  QUrl url(currentUrl_);
  QString path = url.path();
  if (!path.endsWith('/')) {
    path += '/';
  }
  path += "download/vdb/" + record.uuid;  // 修改为vdb下载路径
  url.setPath(path);
  QNetworkRequest request(url);
  currentReply_ = networkManager_->get(request);
  
  connect(currentReply_, &QNetworkReply::finished, this, &DownloadVdbDialog::onFileDownloaded);
  connect(currentReply_, &QNetworkReply::errorOccurred, this, &DownloadVdbDialog::onNetworkError);

  // 禁用按钮，防止重复下载
  ui->downloadButton->setEnabled(false);
  ui->refreshButton->setEnabled(false);
  ui->urlEdit->setEnabled(false);
}

void DownloadVdbDialog::onFileDownloaded() {
  // 重新启用按钮
  ui->downloadButton->setEnabled(true);
  ui->refreshButton->setEnabled(true);
  ui->urlEdit->setEnabled(true);
  
  if (currentReply_->error() != QNetworkReply::NoError) {
    // 错误处理已在onNetworkError中完成
    currentReply_->deleteLater();
    currentReply_ = nullptr;
    return;
  }
  
  // 保存文件
  QByteArray fileData = currentReply_->readAll();
  currentReply_->deleteLater();
  currentReply_ = nullptr;
  
  QFile file(currentSavePath_);
  if (!file.open(QIODevice::WriteOnly)) {
    QMessageBox::warning(this, tr("错误"), tr("无法写入文件: %1").arg(file.errorString()));
    return;
  }
  
  file.write(fileData);
  file.close();
  
  QMessageBox::information(this, tr("成功"), tr("文件已成功下载到: %1").arg(currentSavePath_));
}

void DownloadVdbDialog::onNetworkError(QNetworkReply::NetworkError error) {
  // 重新启用按钮
  ui->downloadButton->setEnabled(true);
  ui->refreshButton->setEnabled(true);
  ui->urlEdit->setEnabled(true);
  
  QString errorMessage;
  switch (error) {
    case QNetworkReply::ConnectionRefusedError:
      errorMessage = tr("连接被拒绝");
      break;
    case QNetworkReply::HostNotFoundError:
      errorMessage = tr("找不到主机");
      break;
    case QNetworkReply::TimeoutError:
      errorMessage = tr("连接超时");
      break;
    default:
      errorMessage = tr("网络错误: %1").arg(currentReply_->errorString());
      break;
  }
  
  QMessageBox::warning(this, tr("错误"), errorMessage);
}