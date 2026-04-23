#include "centerpointdialog.h"

#include <QFileDialog>
#include <QMessageBox>

#include "core/point3d.h"
#include "map/projection_utm.h"
#include "ui_centerpointdialog.h"

using namespace geditor;

CenterPointDialog::CenterPointDialog(QWidget* parent)
    : QDialog(parent), ui(new Ui::CenterPointDialog) {
  ui->setupUi(this);
}

CenterPointDialog::~CenterPointDialog() { delete ui; }

void CenterPointDialog::on_pushButton_REST_CONTEXT_clicked() {
  ui->edit_WGS84X->setText("");
  ui->edit_WGS84Y->setText("");

  ui->edit_UtmX->setText("");
  ui->edit_UtmY->setText("");
  ui->edit_UtmZone->setText("");
}

void CenterPointDialog::on_pushButton_CONV_GPS_clicked() {
  QString strX = ui->edit_UtmX->text();
  QString strY = ui->edit_UtmY->text();
  QString strZone = ui->edit_UtmZone->text();

  if (strX.length() > 0 && strY.length() > 0 && strZone.length() > 0) {
    double paramX = strX.toDouble();
    double paramY = strY.toDouble();
    int zone = strZone.toInt();

    LatLon point;
    ProjectionUTM project;
    project.CartesianToLatLon(paramX, paramY, zone, false, point);
    m_lon = point.lon;
    m_lat = point.lat;
    ui->edit_WGS84X->setText(QString::number(point.lon, 'f', 8));
    ui->edit_WGS84Y->setText(QString::number(point.lat, 'f', 8));

  } else {
    QMessageBox::warning(this, "提示", "请输入合法的UTM坐标!");
  }
}

void CenterPointDialog::on_pushButton_CONV_UTM_clicked() {
  QString strX = ui->edit_WGS84X->text();
  QString strY = ui->edit_WGS84Y->text();

  if (strX.length() > 0 && strY.length() > 0) {
    double paramX = strX.toDouble();
    double paramY = strY.toDouble();

    UTMPoint point;

    ProjectionUTM prject;
    prject.LatLonToCartesian(paramY, paramX, point);
    ui->edit_UtmX->setText(QString::number(point.x, 'f', 3));
    ui->edit_UtmY->setText(QString::number(point.y, 'f', 3));
    ui->edit_UtmZone->setText(QString::number(point.zone));
    ui->edit_gamma->setText(QString::number(point.gamma, 'f', 3));
  } else {
    QMessageBox::warning(this, "提示", "请输入合法的WGS84坐标!");
  }
}

void CenterPointDialog::on_pushButton_OK_clicked() {
  QString strGpsX = ui->edit_WGS84X->text();
  QString strGpsY = ui->edit_WGS84Y->text();

  QString strUtmX = ui->edit_UtmX->text();
  QString strUtmY = ui->edit_UtmY->text();
  QString strZone = ui->edit_UtmZone->text();

  ProjectionUTM project;

  if (strGpsX.length() > 0 && strGpsY.length() > 0) {
    m_lon = strGpsX.toDouble();
    m_lat = strGpsY.toDouble();
    accept();
  } else if (strUtmX.length() > 0 && strUtmY.length() > 0 &&
             strZone.length() > 0) {
    double paramX = strUtmX.toDouble();
    double paramY = strUtmY.toDouble();
    int zone = strZone.toInt();

    LatLon point;
    project.CartesianToLatLon(paramX, paramY, zone, false, point);

    m_lon = point.lon;
    m_lat = point.lat;
    accept();
  } else {
    QMessageBox::warning(this, "提示", "请输入合法的坐标!");
  }
}