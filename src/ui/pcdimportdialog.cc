#include "pcdimportdialog.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QtConcurrent>
#include <iostream>

#include "algorithm/common.h"
#include "map/pdb_manage.h"
#include "map/tile_pdb.h"
#include "pcd/db_read_write.h"
#include "pcd/pcd_split.h"
#include "pcd/sqlite_rwer.h"
#include "pcd/voxel_grid.h"
#include "ui_pcdimportdialog.h"

using namespace geditor;

PCDImportDialog::PCDImportDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::PCDImportDialog) {
  ui->setupUi(this);
}

PCDImportDialog::~PCDImportDialog() { delete ui; }

void PCDImportDialog::on_set_pcd_path_btn_clicked() {
  QFileDialog fd(this, "Select file", "", "txt文件(*.txt);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];
    QDir dir = fd.directory();
    ui->pcd_path_edit->setText(sel);
    std::string file = sel.toStdString();
    std::fstream fs(file);
    if (!fs.is_open()) {
      qWarning() << "Failed to open " << sel;
    }
    std::string line;
    int skip = 0, skiprow = 1;
    while (std::getline(fs, line)) {
      if (skip++ < skiprow) continue;
      std::stringstream ss(line);
      std::string pcd;
      double x, y, z, zone;
      ss >> pcd >> x >> y >> z >> zone;
      pcd = dir.path().toStdString() + "/" + pcd;
      V3d origin(x, y, z);
      tra_[pcd] = origin;
      if (skip == 2) {
        ui->origin_x_edit->setText(std::to_string(x).c_str());
        ui->origin_y_edit->setText(std::to_string(y).c_str());
        ui->zone_edit->setText(std::to_string(zone).c_str());
        zone_ = zone;
        origin_ = origin;
      }
    }
  }
}

void PCDImportDialog::on_set_pdb_path_btn_clicked() {
  QString sel = QFileDialog::getSaveFileName(this, "设置保存文件名", "",
                                             "pdb地图(*.pdb);;");
  if (!sel.isEmpty()) {
    if (!sel.endsWith(".pdb")) sel += ".pdb";
    ui->pdb_path_edt->setText(sel);
  }
}

void PCDImportDialog::on_import_btn_clicked() {
  pcd_path_ = ui->pcd_path_edit->text().toUtf8().constData();
  pdb_path_ = ui->pdb_path_edt->text().toUtf8().constData();
  if (tra_.empty()) {
    QMessageBox::warning(this, "警告", "请选择正确的PCD路径");
    return;
  }
  if (pdb_path_.empty()) {
    QMessageBox::warning(this, "警告", "请选择正确的PDB路径");
    return;
  }

  if (origin_[0] < 2 || origin_[1] < 2 || zone_ < 2) {
    QMessageBox::warning(this, "警告", "请输入正确的原点坐标");
    // return;
  }

  // auto st = QtConcurrent::run(this, &PCDImportDialog::split_task);
  // st.waitForFinished();

  split_task();

  QMessageBox::information(this, "提示", "切分完毕");
}

void PCDImportDialog::split_task() {
  ui->dbimport_proces_bar->setValue(0);
  ui->import_btn->setEnabled(false);
  ui->set_pcd_path_btn->setEnabled(false);
  ui->set_pdb_path_btn->setEnabled(false);

  PointCloud<PCLPoint> outCloud;
  std::shared_ptr<PCDSplitter> splitter(new PCDSplitter());
  splitter->SetOriginPoint(origin_[0], origin_[1], zone_);

  if (merge_pcd2pcd(pcd_path_, outCloud)) {
    ui->dbimport_proces_bar->setValue(30);
    //切分pcd
    std::vector<TilePDB *> tileArray;
    if (splitter->SplitData(outCloud, tileArray)) {
      outCloud.Clear();
      ui->dbimport_proces_bar->setValue(60);
      //保存到数据库
      PDBManage database;
      if (database.Open(pdb_path_.c_str())) {
        size_t isize = tileArray.size();
        for (size_t pos = 0; pos < isize; pos++) {
          TilePDB *pTile = tileArray[pos];
          database.Save(pTile);
          //释放内存
          delete pTile;
        }
        database.Close();
        ui->import_btn->setEnabled(true);
        ui->dbimport_proces_bar->setValue(100);
        return;
      }
    }
  } else {
    outCloud.Clear();
  }

  ui->import_btn->setEnabled(true);
  ui->set_pcd_path_btn->setEnabled(true);
  ui->set_pdb_path_btn->setEnabled(true);
  ui->dbimport_proces_bar->setValue(100);
  return;
}
bool PCDImportDialog::merge_pcd2pcd(
    const std::string &db_file_name,
    geditor::PointCloud<geditor::PCLPoint> &map) {
  for (auto &node : tra_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ps(
        new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(node.first, *ps) == -1) continue;
    PointCloud<PCLPoint> cloud(ps->points.size());
    auto offset = node.second;
    for (auto &p : ps->points) {
      PCLPoint pp;
      pp.x = p.x + offset.x - origin_.x;
      pp.y = p.y + offset.y - origin_.y;
      pp.z = p.z + offset.z - origin_.z;
      pp.w = p.intensity > 0 ? p.intensity : 1;  // intensity 等于0时，置为1
      cloud.PushBack(pp);
    }
    PointCloud<PCLPoint> out;
    VoxelGrid<PCLPoint> sor;
    sor.setInputCloud(&cloud);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(out);
    map.MergePointCloud(out);
  }
  return true;
}
