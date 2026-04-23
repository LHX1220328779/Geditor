#pragma once

#include <QDialog>

namespace Ui {
class PointCloudFilterDialog;
}

class PointCloudFilterDialog : public QDialog {
  Q_OBJECT

 public:
  explicit PointCloudFilterDialog(QWidget *parent = 0);
  ~PointCloudFilterDialog();
  void Show();

 private slots:
  void on_pushButton_clicked();

 private:
  Ui::PointCloudFilterDialog *ui;
  QWidget *win_;
};
