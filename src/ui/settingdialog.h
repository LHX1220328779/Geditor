#ifndef SETTINGDIALOG_H
#define SETTINGDIALOG_H

#include <QDialog>

namespace Ui {
class SettingDialog;
}

class SettingDialog : public QDialog {
  Q_OBJECT

 public:
  explicit SettingDialog(QWidget *parent = 0);
  ~SettingDialog();

 private slots:
  void on_pushButton_OK_clicked();
  void on_pushButton_CANCEL_clicked();

 public:
  int m_gridSize;
  int m_lineWidth;
  int m_pointSize;

 private:
  Ui::SettingDialog *ui;
};

#endif  // SETTINGDIALOG_H
