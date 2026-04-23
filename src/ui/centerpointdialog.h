#pragma once

#include <QDialog>

namespace Ui {
class CenterPointDialog;
}

class CenterPointDialog : public QDialog {
  Q_OBJECT

 public:
  explicit CenterPointDialog(QWidget *parent = 0);
  ~CenterPointDialog();

 public:
  double m_lon;
  double m_lat;

 private slots:

  void on_pushButton_REST_CONTEXT_clicked();

  void on_pushButton_CONV_GPS_clicked();

  void on_pushButton_CONV_UTM_clicked();

  void on_pushButton_OK_clicked();

 private:
  Ui::CenterPointDialog *ui;
};
