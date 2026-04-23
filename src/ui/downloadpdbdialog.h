#pragma once

#include <QDialog>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUrl>
#include <QListWidgetItem>
#include <QMap>

namespace Ui {
class DownloadPdbDialog;
}

// 文件记录结构体
struct FileRecord {
    QString name;
    QString remark;
    QString time;
    QString uuid;
};

class DownloadPdbDialog : public QDialog {
  Q_OBJECT

 public:
  explicit DownloadPdbDialog(QWidget *parent = nullptr);
  ~DownloadPdbDialog();

 private slots:
  void on_refreshButton_clicked();
  void on_downloadButton_clicked();
  void on_urlEdit_textChanged(const QString &text);
  void onFileListReceived();
  void onFileDownloaded();
  void onNetworkError(QNetworkReply::NetworkError error);

 private:
  void refreshFileList();
  void downloadSelectedFile();
  
 private:
  Ui::DownloadPdbDialog *ui;
  QNetworkAccessManager *networkManager_;
  QString currentUrl_;
  QNetworkReply *currentReply_;
  QString currentSavePath_;  // 保存当前下载文件的保存路径
  QMap<int, FileRecord> fileRecords_;  // 存储文件记录，键为列表索引
};