#include "uploaddialog.h"

#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QHttpMultiPart>
#include <QHttpPart>
#include <QMessageBox>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QUrl>
#include <QtConcurrent>
#include <iostream>

#include "algorithm/common.h"
#include "map/pdb_manage.h"
#include "map/tile_pdb.h"
#include "pcd/db_read_write.h"
#include "pcd/pcd_split.h"
#include "pcd/sqlite_rwer.h"
#include "pcd/voxel_grid.h"
#include "ui_uploaddialog.h"

using namespace geditor;

UploadDialog::UploadDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::UploadDialog) {
  ui->setupUi(this);
}

UploadDialog::~UploadDialog() { delete ui; }

void UploadDialog::on_btn_file_clicked() {
  QFileDialog fd(this, "Select file", "", "vdb文件(*.vdb);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];
    QDir dir = fd.directory();
    ui->edt_file->setText(sel);
  }
}

void UploadDialog::on_btn_upload_clicked() {
  QString url = ui->edt_url->text();
  auto file_f = ui->edt_file->text();
  auto name_d = ui->edt_data_name->text();
  auto remark = ui->edt_data_remark->text();
  auto layer_type = getLayerType();

  if (url.isEmpty() || file_f.isEmpty() || name_d.isEmpty()) {
    ShowResult("上传参数信息不完整，请填写空白信息！", false);
    return;
  }

  QNetworkAccessManager *manager = new QNetworkAccessManager();
  QHttpMultiPart *multiPart = new QHttpMultiPart(QHttpMultiPart::FormDataType);

  QHttpPart filePart;
  // filePart.setHeader(QNetworkRequest::ContentTypeHeader,
  //                    QVariant("application/octet-stream"));
  filePart.setHeader(QNetworkRequest::ContentDispositionHeader,
                     QVariant("form-data; name=\"file\"; filename=\"" +
                              QFileInfo(file_f).fileName() + "\""));

  QFile *file = new QFile(file_f);
  if (!file->open(QIODevice::ReadOnly)) {
    ShowResult("上传文件打开失败！", false);
    return;
  }
  filePart.setBodyDevice(file);
  file->setParent(multiPart);
  multiPart->append(filePart);

  QHttpPart dataname;
  dataname.setHeader(QNetworkRequest::ContentDispositionHeader,
                     QVariant("form-data; name=\"name\""));
  dataname.setBody(name_d.toUtf8());
  multiPart->append(dataname);

  QHttpPart layerPart;
  layerPart.setHeader(QNetworkRequest::ContentDispositionHeader,
                      QVariant("form-data; name=\"layerType\""));
  layerPart.setBody(QString::number(layer_type).toUtf8());
  multiPart->append(layerPart);

  QHttpPart dataremark;
  dataremark.setHeader(QNetworkRequest::ContentDispositionHeader,
                       QVariant("form-data; name=\"remark\""));
  dataremark.setBody(remark.toUtf8());
  multiPart->append(dataremark);

  QUrl qurl(url);
  QNetworkRequest request(qurl);

  QNetworkReply *reply = manager->post(request, multiPart);
  multiPart->setParent(reply);

  QEventLoop loop;
  QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
  loop.exec();

  if (reply->error() == QNetworkReply::NoError) {
    ShowResult("上传成功！", true);
  } else {
    ShowResult(reply->errorString(), false);
  }

  reply->deleteLater();
  manager->deleteLater();
}

void UploadDialog::ShowResult(const QString &msg, bool ok) {
  if (ok)
    QMessageBox::information(this, "提示", msg);
  else
    QMessageBox::warning(this, "警告", msg);
}

int UploadDialog::getLayerType() const {
  return ui->cmb_layer_type->currentIndex();
}