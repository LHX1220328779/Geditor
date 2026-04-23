#pragma once

#include <pcd/db_read_write.h>

#include <QDialog>

#include "algorithm/common.h"
#include "pcd/read_pcl.h"

namespace Ui {
class UploadDialog;
}

class UploadDialog : public QDialog {
  Q_OBJECT

 public:
  explicit UploadDialog(QWidget *parent = 0);

  ~UploadDialog();

 private slots:

  void on_btn_file_clicked();
  void on_btn_upload_clicked();

 private:
  void ShowResult(const QString &msg, bool ok);
  // 获取当前选择的地图类型
  int getLayerType() const;
  Ui::UploadDialog *ui;
};