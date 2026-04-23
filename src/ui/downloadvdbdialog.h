#ifndef GEDITOR_UI_DOWNLOADVDBDIALOG_H_
#define GEDITOR_UI_DOWNLOADVDBDIALOG_H_

#include <QDialog>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QString>
#include <QMap>

namespace Ui {
class DownloadVdbDialog;
}

class DownloadVdbDialog : public QDialog {
  Q_OBJECT

 public:
  explicit DownloadVdbDialog(QWidget *parent = nullptr);
  ~DownloadVdbDialog();

 private slots:
  void on_refreshButton_clicked();
  void on_urlEdit_textChanged(const QString &text);
  void on_downloadButton_clicked();

 private:
  struct FileRecord {
    QString name;
    QString remark;
    QString time;
    QString uuid;
    QString state;      // 新增：状态字段
    int layerType;      // 新增：图层类型 [0:全量地图; 1:基础地图; 2:业务地图]
  };

  void refreshFileList();
  void downloadSelectedFile();
  void onFileListReceived();
  void onFileDownloaded();
  void onNetworkError(QNetworkReply::NetworkError error);
  QString getLayerTypeText(int layerType) const;  // 新增：获取图层类型的文本描述

  Ui::DownloadVdbDialog *ui;
  QNetworkAccessManager *networkManager_;
  QNetworkReply *currentReply_ = nullptr;
  QString currentUrl_;
  QString currentSavePath_;
  QMap<int, FileRecord> fileRecords_;
};

#endif  // GEDITOR_UI_DOWNLOADVDBDIALOG_H_