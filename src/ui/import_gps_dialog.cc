#include "import_gps_dialog.h"

#include <QDebug>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <fstream>
#include <sstream>

#include "core/point3d.h"
#include "map/projection_utm.h"
#include "ui_importgpsdialog.h"

using namespace geditor;

ImportGPSDialog::ImportGPSDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ImportGPSDialog) {
  ui->setupUi(this);

  model_ = new QStandardItemModel();
  ui->tableView->setModel(model_);
  ui->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);

  model_->setColumnCount(2);
  model_->setHeaderData(0, Qt::Horizontal, "序号");
  model_->setHeaderData(1, Qt::Horizontal, "文件路径");
}

ImportGPSDialog::~ImportGPSDialog() {
  delete ui;
  delete model_;
}

// void ImportGPSDialog::on_add_btn_clicked() {
//   QFileDialog fd(this, "Select file", "", "GPS轨迹(*.txt);;");
//   if (fd.exec() == QDialog::Accepted) {
//     QString sel = fd.selectedFiles()[0];
//     int indexPos = model_->rowCount();
//     model_->setItem(indexPos, 0,
//                     new QStandardItem(std::to_string(indexPos + 1).c_str()));
//     model_->setItem(indexPos, 1, new QStandardItem(sel));
//   }
// }

void ImportGPSDialog::on_add_btn_clicked() {
  QFileDialog fd(this, "Select files", "", "GPS轨迹(*.txt *.csv);;");
  fd.setFileMode(QFileDialog::ExistingFiles); // 设置文件选择模式为多选
  if (fd.exec() == QDialog::Accepted) {
    QStringList sel = fd.selectedFiles();  // 获取所有选中的文件
    for (const QString &file : sel) {
      int indexPos = model_->rowCount();
      model_->setItem(indexPos, 0,
                      new QStandardItem(std::to_string(indexPos + 1).c_str()));
      model_->setItem(indexPos, 1, new QStandardItem(file));  
    }
  }
}

void ImportGPSDialog::on_delete_btn_clicked() {
  int row = ui->tableView->currentIndex().row();
  if (row < 0 || row >= model_->rowCount()) {
    QMessageBox::warning(this, "提示", "请选择要删除的行");
    return;
  }

  model_->removeRow(row);
}

void ImportGPSDialog::on_clear_btn_clicked() {
  model_->removeRows(0, model_->rowCount());
}

void ImportGPSDialog::on_import_btn_clicked() {
  m_segmentArray.clear();
  for (int i = 0; i < model_->rowCount(); i++) {
    //解析seg的ID

    QString data = model_->item(i, 1)->text();
    std::vector<TrackPoint> pointSet;
    if (ParsePointSet(data, pointSet)) {
      CurbsTrack *pGeometry = new CurbsTrack();
      pGeometry->trackSet = pointSet;
      QFileInfo file_info(data);
      pGeometry->sourcePath = data.toStdString();
      pGeometry->sourceCode = file_info.completeBaseName().toStdString();
      for (const auto &track_point : pointSet) {
        if (track_point.idx > 0) {
          pGeometry->sourceIndex = track_point.idx;
          break;
        }
      }
      m_segmentArray.push_back(pGeometry);
      qInfo() << "import tra: " << data;
    }
  }
  if (m_segmentArray.size() > 0) accept();
}

bool ImportGPSDialog::ParsePointSet(
    const QString &filename, std::vector<geditor::TrackPoint> &pointSet) {
  std::string file = filename.toStdString();
  std::fstream fs(file);
  if (!fs.is_open()) {
    qWarning() << "Failed to open " << filename;
    return false;
  }
  std::string line;
  int skip = 0, skiprow = 1;
  ProjectionUTM projectionUTM;
  while (std::getline(fs, line)) {
    if (skip++ < skiprow) continue;
    std::stringstream ss(line);
    double t, x, y, z;
    ss >> t >> x >> y >> z;
    if (frame_type_ == 0) {
      if ((x > 1 || x < -1) && (y > 1 || y < -1)) {
        UTMPoint utmxy;
        projectionUTM.LatLonToCartesian(y, x, utmxy);
        TrackPoint trPnt;
        trPnt.pnt = Point3d(utmxy.x, utmxy.y, 0.0);
        pointSet.push_back(trPnt);
      }
    } else {
      TrackPoint trPnt;
      trPnt.pnt = Point3d(x, y, 0.0);
      trPnt.idx = t > 0 ? t : -1;
      pointSet.push_back(trPnt);
    }
  }
  return true;
}

void ImportGPSDialog::on_radioButton_clicked() { import_type_ = 0; }

void ImportGPSDialog::on_radioButton_2_clicked() { import_type_ = 1; }

void ImportGPSDialog::on_radioButton_3_clicked() { import_type_ = 2; }
