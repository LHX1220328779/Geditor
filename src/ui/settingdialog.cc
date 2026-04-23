#include "settingdialog.h"
#include "ui_settingdialog.h"

// using namespace geditor;

SettingDialog::SettingDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::SettingDialog) {
  ui->setupUi(this);

  m_gridSize = 100;
  m_lineWidth = 120;
  m_pointSize = 20;
}

SettingDialog::~SettingDialog() { delete ui; }

void SettingDialog::on_pushButton_OK_clicked() {
  QString str;

  str = ui->spinBox_GRID->text();
  m_gridSize = str.toInt();

  str = ui->spinBox_LANE_WIDTH->text();
  m_lineWidth = str.toInt();

  str = ui->spinBox_CONTROL_POINT->text();
  m_pointSize = str.toInt();
  accept();
}

void SettingDialog::on_pushButton_CANCEL_clicked() { reject(); }
