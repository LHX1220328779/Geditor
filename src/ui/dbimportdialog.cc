#include <QFileDialog>
#include <QMessageBox>
#include <QtConcurrent>

#include "algorithm/common.h"
#include "dbimportdialog.h"
#include "map/pdb_manage.h"
#include "map/tile_pdb.h"
#include "pcd/db_read_write.h"
#include "pcd/pcd_split.h"
#include "pcd/sqlite_rwer.h"
#include "pcd/voxel_grid.h"
#include "ui_dbimportdialog.h"

using namespace geditor;

std::vector<DBTraPoint> db_tra;

DBImportDialog::DBImportDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::DBImportDialog) {
  ui->setupUi(this);
}

DBImportDialog::~DBImportDialog() { delete ui; }

void DBImportDialog::on_set_db_path_btn_clicked() {
  QFileDialog fd(this, "Select file", "", "db文件(*.db);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];
    ui->db_path_edit->setText(sel);
    // read db and set gps data
    SqliteRWer db(sel.toStdString());
    if (db.IsOpen()) {
      db_tra = db.ReadTra();
      if (!db_tra.empty()) {
        db.CloseDB();
      } else {
        db.CloseDB();
        QMessageBox::warning(this, "警告", "DB格式错误");
        return;
      }
      V3d origin(db_tra[0].x, db_tra[0].y, db_tra[0].z);
      ui->origin_x_edit->setText(std::to_string(db_tra[0].x).c_str());
      ui->origin_y_edit->setText(std::to_string(db_tra[0].y).c_str());
      ui->zone_edit->setText(std::to_string(db_tra[0].zone).c_str());

      zone_ = db_tra[0].zone;
      origin_ = origin;
    } else {
      QMessageBox::warning(this, "警告", "无法读取DB");
      return;
    }
  }
}

void DBImportDialog::on_set_pdb_path_btn_clicked() {
  QString sel = QFileDialog::getSaveFileName(this, "设置保存文件名", "",
                                             "pdb地图(*.pdb);;");
  if (!sel.isEmpty()) {
    if (!sel.endsWith(".pdb")) sel += ".pdb";
    ui->pdb_path_edt->setText(sel);
  }
}

void DBImportDialog::on_import_btn_clicked() {
  db_path_ = ui->db_path_edit->text().toUtf8().constData();
  pdb_path_ = ui->pdb_path_edt->text().toUtf8().constData();
  if (db_path_.empty()) {
    QMessageBox::warning(this, "警告", "请选择正确的DB路径");
    return;
  }
  if (pdb_path_.empty()) {
    QMessageBox::warning(this, "警告", "请选择正确的PDB路径");
    return;
  }

  if (origin_[0] < 2 || origin_[1] < 2 || zone_ < 2) {
    QMessageBox::warning(this, "警告", "请输入正确的原点坐标");
    return;
  }

  // auto st = QtConcurrent::run(this, &DBImportDialog::split_task);
  // st.waitForFinished();

  split_task();

  QMessageBox::information(this, "提示", "切分完毕");
}

void DBImportDialog::split_task() {
  ui->dbimport_proces_bar->setValue(0);
  ui->import_btn->setEnabled(false);
  ui->set_db_path_btn->setEnabled(false);
  ui->set_pdb_path_btn->setEnabled(false);

  PointCloud<PCLPoint> outCloud;
  std::shared_ptr<PCDSplitter> splitter(new PCDSplitter());
  splitter->SetOriginPoint(origin_[0], origin_[1], zone_);

  if (merge_db2pcd(db_path_, outCloud)) {
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
  ui->set_db_path_btn->setEnabled(true);
  ui->set_pdb_path_btn->setEnabled(true);
  ui->dbimport_proces_bar->setValue(100);
  return;
}
bool DBImportDialog::merge_db2pcd(const std::string &db_file_name,
                                  geditor::PointCloud<geditor::PCLPoint> &map) {
  SqliteRWer db(db_file_name);
  if (!db.IsOpen()) return false;
  if (db_tra.empty()) return false;
  double offx = db_tra[0].x;
  double offy = db_tra[0].y;
  double offz = db_tra[0].z;
  for (auto &node : db_tra) {
    DBPoints ps = db.ReadFrameByTime(node.time);
    if (ps.points.empty()) continue;
    PointCloud<PCLPoint> cloud(ps.points.size());
    for (auto &p : ps.points) {
      PCLPoint pp;
      pp.x = ps.offset_x + p.x - offx;
      pp.y = ps.offset_y + p.y - offy;
      pp.z = ps.offset_z + p.z - offz;
      pp.w = p.i;
      cloud.PushBack(pp);
    }
    PointCloud<PCLPoint> out;
    VoxelGrid<PCLPoint> sor;
    sor.setInputCloud(&cloud);
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.filter(out);
    map.MergePointCloud(out);
  }
  db.CloseDB();
  return true;
}
bool DBImportDialog::merge_db2pcd2(
    const std::string &db_file_name,
    geditor::PointCloud<geditor::PCLPoint> &out) {
  DBReadWrite dbrw;
  if (dbrw.OpenDB(db_file_name.c_str())) {
    std::vector<UniqueType> mapIdxPose;
    if (!dbrw.QueryAllRawLaserUnique(mapIdxPose)) {
      dbrw.CloseDB();
      return false;
    }

    //点云合并
    PointCloud<PCLPoint> mergeCloud;
    merge_point_cloud(dbrw, mapIdxPose, mergeCloud);

    //---------------------------------------------
    //点云过滤
    int sample = 0;
    if (sample > 0) {
      VoxelGrid<PCLPoint> sor;
      sor.setInputCloud(&mergeCloud);
      sor.setLeafSize(0.01f, 0.01f, 0.01f);
      sor.filter(out);
    } else {
      out.MergePointCloud(mergeCloud);
    }

    dbrw.CloseDB();
    return true;
  }
  return false;
}

void DBImportDialog::merge_point_cloud(
    geditor::DBReadWrite &dbrw, const std::vector<UniqueType> &array_id,
    geditor::PointCloud<geditor::PCLPoint> &merged_cloud) {
  std::vector<UniqueType>::const_iterator iter;
  for (iter = array_id.begin(); iter != array_id.end(); iter++) {
    void *pData = NULL;
    int length = 0;
    int version = 0;

    DBPose pose;
    if (dbrw.ReadRawLaserData(*iter, pose, pData, length, version)) {
      PointCloud<PCLPoint> cloud;
      if (parse_pointcloud_data((char *)pData, length, version, cloud)) {
        Matrix4x4d mat1 = Matrix4x4d::MakeTrans(pose.pos);
        Matrix4x4d mat2 = Matrix4x4d::MakeRotation(pose.quat);
        cloud.TransformPointCloud(mat2 * mat1);
        merged_cloud.MergePointCloud(cloud);
      }
      delete[](char *) pData;
    }
  }
}

bool DBImportDialog::parse_pointcloud_data(
    const char *pBuffer, int length, int version,
    geditor::PointCloud<geditor::PCLPoint> &cloud) {
  if (version == 1) {
    int pointCount = length / (24 * sizeof(char));

    for (int i = 0; i < pointCount; i++) {
      V3f pos;
      memcpy(&pos, pBuffer, sizeof(float) * 3);
      pBuffer += sizeof(float) * 3;
      float vi = *(unsigned char *)(pBuffer + 9);
      pBuffer += sizeof(char) * 12;
      PCLPoint point;
      point.x = pos[0];
      point.y = pos[1];
      point.z = pos[2];
      point.w = vi;
      cloud.PushBack(point);
    }
  } else {
    int pointCount = length / (32 * sizeof(char));
    for (int i = 0; i < pointCount; i++) {
      V3f pos;
      memcpy(&pos, pBuffer, sizeof(float) * 3);
      pBuffer += sizeof(float) * 3;
      float vi = *(float *)(pBuffer + 4);
      pBuffer += sizeof(char) * 20;
      PCLPoint point;
      point.x = pos[0];
      point.y = pos[1];
      point.z = pos[2];
      point.w = vi;

      cloud.PushBack(point);
    }
  }
  return true;
}
