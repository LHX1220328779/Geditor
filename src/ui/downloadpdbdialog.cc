#include "downloadpdbdialog.h"

#include <QFileDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QMessageBox>
#include <QNetworkRequest>
#include <QFileInfo>
#include "ui_downloadpdbdialog.h"

DownloadPdbDialog::DownloadPdbDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::DownloadPdbDialog), networkManager_(new QNetworkAccessManager(this)) {
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

DownloadPdbDialog::~DownloadPdbDialog() {
  delete ui;
}

void DownloadPdbDialog::on_refreshButton_clicked() {
  refreshFileList();
}

void DownloadPdbDialog::on_urlEdit_textChanged(const QString &text) {
  currentUrl_ = text;
}

void DownloadPdbDialog::on_downloadButton_clicked() {
  downloadSelectedFile();
}

void DownloadPdbDialog::refreshFileList() {
  ui->fileListWidget->clear();
  ui->downloadButton->setEnabled(false);

  QUrl url(currentUrl_);
  QString path = url.path();
  if (!path.endsWith('/')) {
    path += '/';
  }
  path += "pipeline/anno/pdb/query";
  url.setPath(path);
  
  QNetworkRequest request(url);
  request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
  
  // 创建空的JSON请求体
  QJsonObject jsonRequest;
  QJsonDocument doc(jsonRequest);
  QByteArray data = doc.toJson();
  
  currentReply_ = networkManager_->post(request, data);
  
  connect(currentReply_, &QNetworkReply::finished, this, &DownloadPdbDialog::onFileListReceived);
  connect(currentReply_, &QNetworkReply::errorOccurred, this, &DownloadPdbDialog::onNetworkError);
}

void DownloadPdbDialog::onFileListReceived() {
  if (currentReply_->error() == QNetworkReply::NoError) {
    QByteArray data = currentReply_->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    
    if (doc.isObject()) {
      QJsonObject root = doc.object();
      int code = root["code"].toInt();
      QString message = root["message"].toString();
      
      if (code == 0) {  // 请求成功
        QJsonArray files = root["data"].toArray();
        ui->fileListWidget->setRowCount(0);  // 清空表格
        fileRecords_.clear();
        
        // 设置表格列数
        ui->fileListWidget->setColumnCount(5);
        
        // 设置表头
        QStringList headers;
        headers << "序号" << "名称" << "备注" << "时间" << "UUID";
        ui->fileListWidget->setHorizontalHeaderLabels(headers);
        
        int index = 0;
        for (const QJsonValue &value : files) {
          if (value.isObject()) {
            QJsonObject obj = value.toObject();
            FileRecord record;
            record.name = obj["name"].toString();
            record.remark = obj["remark"].toString();
            record.time = obj["time"].toString();
            record.uuid = obj["uuid"].toString();
            
            fileRecords_[index] = record;
            
            // 添加一行
            ui->fileListWidget->insertRow(index);
            
            // 设置序号
            QTableWidgetItem *indexItem = new QTableWidgetItem(QString::number(index + 1));
            indexItem->setTextAlignment(Qt::AlignCenter);
            ui->fileListWidget->setItem(index, 0, indexItem);
            
            // 设置名称
            QTableWidgetItem *nameItem = new QTableWidgetItem(record.name);
            ui->fileListWidget->setItem(index, 1, nameItem);
            
            // 设置备注
            QTableWidgetItem *remarkItem = new QTableWidgetItem(record.remark);
            ui->fileListWidget->setItem(index, 2, remarkItem);
            
            // 设置时间
            QTableWidgetItem *timeItem = new QTableWidgetItem(record.time);
            ui->fileListWidget->setItem(index, 3, timeItem);
            
            // 设置UUID
            QTableWidgetItem *uuidItem = new QTableWidgetItem(record.uuid);
            ui->fileListWidget->setItem(index, 4, uuidItem);
            
            index++;
          }
        }
        
        // 调整列宽以适应内容
        ui->fileListWidget->resizeColumnsToContents();
        
      } else {
        QMessageBox::warning(this, tr("请求失败"), message);
      }
    }
  }
  
  currentReply_->deleteLater();
  currentReply_ = nullptr;
}

void DownloadPdbDialog::downloadSelectedFile() {
  QList<QTableWidgetItem*> selectedItems = ui->fileListWidget->selectedItems();
  if (selectedItems.isEmpty()) return;

  int currentRow = selectedItems.first()->row();
  if (!fileRecords_.contains(currentRow)) return;

  const FileRecord& record = fileRecords_[currentRow];
  QString fileName = record.name + ".pdb";
  currentSavePath_ = QFileDialog::getSaveFileName(this, tr("保存文件"),
                                                fileName,
                                                tr("PDB文件 (*.pdb);;所有文件 (*.*)"));
  
  if (currentSavePath_.isEmpty()) return;

  QUrl url(currentUrl_);
  QString path = url.path();
  if (!path.endsWith('/')) {
    path += '/';
  }
  path += "download/pdb/" + record.uuid;
  url.setPath(path);
  QNetworkRequest request(url);
  currentReply_ = networkManager_->get(request);
  
  connect(currentReply_, &QNetworkReply::finished, this, &DownloadPdbDialog::onFileDownloaded);
  connect(currentReply_, &QNetworkReply::errorOccurred, this, &DownloadPdbDialog::onNetworkError);

  // 禁用按钮，防止重复下载
  ui->downloadButton->setEnabled(false);
  ui->refreshButton->setEnabled(false);
  ui->urlEdit->setEnabled(false);
}

void DownloadPdbDialog::onFileDownloaded() {
  if (currentReply_->error() == QNetworkReply::NoError) {
    QByteArray data = currentReply_->readAll();
    
    QFile file(currentSavePath_);
    if (file.open(QIODevice::WriteOnly)) {
      file.write(data);
      file.close();
      QMessageBox::information(this, tr("下载完成"), tr("文件已成功下载到：%1").arg(currentSavePath_));
      accept();
    } else {
      QMessageBox::critical(this, tr("错误"), tr("无法保存文件：%1").arg(file.errorString()));
    }
  }

  // 重新启用按钮
  ui->downloadButton->setEnabled(true);
  ui->refreshButton->setEnabled(true);
  ui->urlEdit->setEnabled(true);

  currentReply_->deleteLater();
  currentReply_ = nullptr;
}

void DownloadPdbDialog::onNetworkError(QNetworkReply::NetworkError error) {
  QString errorMessage = tr("网络错误：%1").arg(currentReply_->errorString());
  QMessageBox::critical(this, tr("错误"), errorMessage);

  // 重新启用按钮
  ui->downloadButton->setEnabled(true);
  ui->refreshButton->setEnabled(true);
  ui->urlEdit->setEnabled(true);

  if (currentReply_) {
    currentReply_->deleteLater();
    currentReply_ = nullptr;
  }
}