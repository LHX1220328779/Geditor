#include "pointcloudfilterdialog.h"

#include <QMessageBox>

#include "ui/display_widget.h"
#include "ui_pointcloudfilterdialog.h"

// using namespace geditor;

PointCloudFilterDialog::PointCloudFilterDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::PointCloudFilterDialog), win_(parent) {
  ui->setupUi(this);
}
PointCloudFilterDialog::~PointCloudFilterDialog() { delete ui; }

void PointCloudFilterDialog::Show() {
  geditor::Framework *fram = ((DisplayWidget *)win_)->framework();
  float mini, maxi;
  float minh, maxh;
  int type, r;
  fram->GetPointCloudFilter(mini, maxi);
  fram->GetPointHighFilter(minh, maxh);
  fram->GetColorType(type, r);
  ui->spinBox_min_intensity->setValue(mini);
  ui->spinBox_max_intensity->setValue(maxi);
  ui->min_height->setValue(minh);
  ui->max_height->setValue(maxh);
  ui->color_r->setValue(r);
  ui->color_type->setCurrentIndex(type);
  float wl, wm;
  fram->GetLaneConf(wl, wm);
  ui->w_lane->setValue(wl);
  ui->w_middle->setValue(wm);
  show();
}

void PointCloudFilterDialog::on_pushButton_clicked() {
  float min_intensity = ui->spinBox_min_intensity->value();
  float max_intensity = ui->spinBox_max_intensity->value();
  float min_height = ui->min_height->value();
  float max_height = ui->max_height->value();
  int color_r = ui->color_r->value();
  int color_type = ui->color_type->currentIndex();

  if (min_intensity >= max_intensity) {
    QMessageBox::warning(this, "警告", "请输入合理的强度范围！");
    return;
  }
  if (min_height >= max_height) {
    QMessageBox::warning(this, "警告", "请输入合理的高度范围！");
    return;
  }
  geditor::Framework *fram = ((DisplayWidget *)win_)->framework();
  fram->SetPointCloudFilter(min_intensity, max_intensity);
  fram->SetPointHighFilter(min_height, max_height);
  fram->SetColorType(color_type, color_r);

  fram->SetLaneConf(ui->w_lane->value(), ui->w_middle->value());

  // Show();
}
