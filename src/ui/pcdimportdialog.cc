#include "pcdimportdialog.h"

#include <minilzo.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QtConcurrent>
#include <cmath>
#include <fstream>
#include <iostream>

#include "algorithm/common.h"
#include "map/pdb_manage.h"
#include "map/projection_utm.h"
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
  resize(730, 720);
  setMinimumSize(730, 720);

  auto *relative_group =
      new QGroupBox("相对矿山原点PCD转PDB（无需info.txt，使用原始RGB）", this);
  relative_group->setGeometry(20, 385, 690, 310);
  auto *grid = new QGridLayout(relative_group);

  auto *description = new QLabel(
      "适用于PCD的X/Y/Z已经相对矿山原点的情况；原点来自项目根目录 "
      "mine_origins.yaml。",
      relative_group);
  description->setWordWrap(true);
  grid->addWidget(description, 0, 0, 1, 3);

  relative_pcd_path_edit_ = new QLineEdit(relative_group);
  auto *select_pcd = new QPushButton("选择PCD", relative_group);
  grid->addWidget(new QLabel("相对坐标PCD", relative_group), 1, 0);
  grid->addWidget(relative_pcd_path_edit_, 1, 1);
  grid->addWidget(select_pcd, 1, 2);

  relative_origin_combo_ = new QComboBox(relative_group);
  grid->addWidget(new QLabel("矿山原点", relative_group), 2, 0);
  grid->addWidget(relative_origin_combo_, 2, 1, 1, 2);
  relative_origin_details_ = new QLabel(relative_group);
  relative_origin_details_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  grid->addWidget(relative_origin_details_, 3, 1, 1, 2);

  relative_pdb_path_edit_ = new QLineEdit(relative_group);
  auto *select_pdb = new QPushButton("选择输出", relative_group);
  grid->addWidget(new QLabel("PDB输出路径", relative_group), 4, 0);
  grid->addWidget(relative_pdb_path_edit_, 4, 1);
  grid->addWidget(select_pdb, 4, 2);

  relative_progress_ = new QProgressBar(relative_group);
  relative_progress_->setRange(0, 100);
  relative_progress_->setValue(0);
  auto *convert = new QPushButton("相对原点PCD转PDB", relative_group);
  grid->addWidget(relative_progress_, 5, 0, 1, 2);
  grid->addWidget(convert, 5, 2);
  grid->setColumnStretch(1, 1);

  connect(select_pcd, &QPushButton::clicked, this,
          &PCDImportDialog::SelectRelativePCD);
  connect(select_pdb, &QPushButton::clicked, this,
          &PCDImportDialog::SelectRelativePDB);
  connect(convert, &QPushButton::clicked, this, [this]() {
    if (ConvertRelativePCD()) {
      QMessageBox::information(this, "完成", "相对坐标PCD已按原始RGB转换为PDB");
    }
  });
  connect(relative_origin_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PCDImportDialog::UpdateRelativeOriginDetails);

  const QString config_path = QDir(QCoreApplication::applicationDirPath())
                                  .absoluteFilePath("../mine_origins.yaml");
  std::string config_error;
  if (!MineOriginConfig::Load(QDir::cleanPath(config_path).toStdString(),
                              mine_origins_, &config_error)) {
    relative_origin_combo_->setEnabled(false);
    relative_origin_details_->setText(QString::fromStdString(config_error));
  } else {
    for (const auto &origin : mine_origins_) {
      relative_origin_combo_->addItem(
          QString("%1 | LAT=%2, LON=%3, zone=%4")
              .arg(QString::fromStdString(origin.name))
              .arg(origin.latitude, 0, 'f', 9)
              .arg(origin.longitude, 0, 'f', 9)
              .arg(origin.zone));
    }
    UpdateRelativeOriginDetails(0);
  }
}

PCDImportDialog::~PCDImportDialog() { delete ui; }

void PCDImportDialog::SelectRelativePCD() {
  const QString path = QFileDialog::getOpenFileName(
      this, "选择相对矿山原点的PCD", relative_pcd_path_edit_->text(),
      "PCD点云 (*.pcd)");
  if (!path.isEmpty()) relative_pcd_path_edit_->setText(path);
}

void PCDImportDialog::SelectRelativePDB() {
  QString path = QFileDialog::getSaveFileName(
      this, "设置PDB输出路径", relative_pdb_path_edit_->text(),
      "PDB地图 (*.pdb)");
  if (path.isEmpty()) return;
  if (!path.endsWith(".pdb", Qt::CaseInsensitive)) path += ".pdb";
  relative_pdb_path_edit_->setText(path);
}

void PCDImportDialog::UpdateRelativeOriginDetails(int index) {
  if (index < 0 || index >= static_cast<int>(mine_origins_.size())) return;
  const auto &origin = mine_origins_[index];
  relative_origin_details_->setText(
      QString("GLOBAL_ORIGIN_LAT=%1  GLOBAL_ORIGIN_LON=%2  "
              "GLOBAL_ORIGIN_ALT=%3  zone=%4")
          .arg(origin.latitude, 0, 'f', 9)
          .arg(origin.longitude, 0, 'f', 9)
          .arg(origin.z, 0, 'f', 3)
          .arg(origin.zone));
}

bool PCDImportDialog::ConvertRelativePCD() {
  const QString pcd_path = relative_pcd_path_edit_->text().trimmed();
  const QString pdb_path = relative_pdb_path_edit_->text().trimmed();
  const int origin_index = relative_origin_combo_->currentIndex();
  if (!QFileInfo(pcd_path).isFile()) {
    QMessageBox::warning(this, "参数错误", "请选择有效的相对坐标PCD文件");
    return false;
  }
  if (pdb_path.isEmpty()) {
    QMessageBox::warning(this, "参数错误", "请选择PDB输出路径");
    return false;
  }
  if (origin_index < 0 ||
      origin_index >= static_cast<int>(mine_origins_.size())) {
    QMessageBox::warning(this, "参数错误", "请选择有效的矿山原点");
    return false;
  }
  if (lzo_init() != LZO_E_OK) {
    QMessageBox::warning(this, "初始化失败", "无法初始化PDB压缩模块");
    return false;
  }

  // Reject colorless input explicitly: this conversion mode promises source
  // RGB display rather than silently falling back to intensity coloring.
  std::ifstream header(pcd_path.toStdString());
  std::string header_line;
  bool has_rgb = false;
  for (int i = 0; i < 20 && std::getline(header, header_line); ++i) {
    if (header_line.rfind("FIELDS", 0) == 0) {
      std::istringstream fields(header_line);
      std::string value;
      fields >> value;
      while (fields >> value) {
        if (value == "rgb" || value == "rgba") has_rgb = true;
      }
      break;
    }
  }
  if (!has_rgb) {
    QMessageBox::warning(this, "缺少RGB",
                         "所选PCD没有rgb/rgba字段，无法按原始RGB颜色转换");
    return false;
  }

  relative_progress_->setValue(5);
  QApplication::processEvents();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path.toStdString(), *input) <
      0) {
    QMessageBox::warning(this, "读取失败", "无法读取PCD文件");
    relative_progress_->setValue(0);
    return false;
  }

  relative_progress_->setValue(30);
  QApplication::processEvents();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.setDownsampleAllData(true);
  voxel.filter(*filtered);
  input.reset();

  const auto &origin = mine_origins_[origin_index];
  ProjectionUTM projection;
  PointCloud<PCLRGBPoint> cloud(filtered->size());
  for (const auto &point : filtered->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    GPSPoint gps;
    projection.LocalENUToGPS(point.x, point.y, point.z, origin.latitude,
                             origin.longitude, origin.z, gps);
    UTMPoint utm;
    projection.LatLonToCartesian(gps.latlon.lat, gps.latlon.lon, utm);
    if (utm.zone != origin.zone) {
      QMessageBox::warning(this, "转换失败",
                           "点云超出所选矿山原点的UTM带区范围");
      relative_progress_->setValue(0);
      return false;
    }
    PCLRGBPoint converted;
    // SplitRGBData works in offsets from its configured UTM origin. ENU must
    // first be converted through WGS84; directly treating ENU as a UTM delta
    // ignores meridian convergence and caused the observed horizontal shift.
    converted.x = static_cast<float>(utm.x - origin.x);
    converted.y = static_cast<float>(utm.y - origin.y);
    converted.z = gps.altitude;
    converted.w = 128.0f;
    converted.rgb = (static_cast<std::uint32_t>(point.r) << 16) |
                    (static_cast<std::uint32_t>(point.g) << 8) |
                    static_cast<std::uint32_t>(point.b);
    cloud.PushBack(converted);
  }
  filtered.reset();
  if (cloud.GetPointCount() == 0) {
    QMessageBox::warning(this, "转换失败", "PCD中没有有效点");
    relative_progress_->setValue(0);
    return false;
  }

  relative_progress_->setValue(55);
  QApplication::processEvents();
  PCDSplitter splitter;
  splitter.SetOriginPoint(origin.x, origin.y, origin.zone);
  std::vector<TilePDB *> tiles;
  if (!splitter.SplitRGBData(cloud, tiles)) {
    for (TilePDB *tile : tiles) delete tile;
    QMessageBox::warning(this, "转换失败", "无法按矿山原点切分点云");
    relative_progress_->setValue(0);
    return false;
  }
  cloud.Clear();

  relative_progress_->setValue(75);
  QApplication::processEvents();
  if (QFileInfo::exists(pdb_path) && !QFile::remove(pdb_path)) {
    for (TilePDB *tile : tiles) delete tile;
    QMessageBox::warning(this, "写入失败", "无法覆盖已存在的PDB文件");
    relative_progress_->setValue(0);
    return false;
  }
  PDBManage database;
  if (!database.Open(pdb_path.toStdString().c_str()) ||
      !database.SetUTMZone(origin.zone) ||
      !database.SetPointColorModeRGB(true)) {
    for (TilePDB *tile : tiles) delete tile;
    QMessageBox::warning(this, "写入失败", "无法创建PDB或写入PDB元数据");
    relative_progress_->setValue(0);
    return false;
  }
  bool saved = true;
  for (TilePDB *tile : tiles) {
    if (!database.Save(tile)) saved = false;
    delete tile;
  }
  database.Close();
  if (!saved) {
    QMessageBox::warning(this, "写入失败", "部分点云瓦片写入失败");
    relative_progress_->setValue(0);
    return false;
  }
  relative_progress_->setValue(100);
  return true;
}

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
      double x, y, z;
      int zone;
      if (!(ss >> pcd >> x >> y >> z >> zone) || zone < 1 || zone > 60) {
        QMessageBox::warning(this, "警告", "info.txt 中存在无效记录或UTM带区");
        tra_.clear();
        return;
      }
      pcd = dir.path().toStdString() + "/" + pcd;
      V3d origin(x, y, z);
      tra_[pcd] = origin;
      if (skip == 2) {
        ui->origin_x_edit->setText(std::to_string(x).c_str());
        ui->origin_y_edit->setText(std::to_string(y).c_str());
        ui->zone_edit->setText(std::to_string(zone).c_str());
        zone_ = zone;
        origin_ = origin;
      } else if (zone_ != zone) {
        QMessageBox::warning(this, "警告", "info.txt 中所有记录必须使用同一个UTM带区");
        tra_.clear();
        return;
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
        if (!database.SetUTMZone(zone_)) {
          QMessageBox::warning(this, "警告", "无法保存UTM带区元数据");
          return;
        }
        if (!database.SetPointColorModeRGB(false)) {
          QMessageBox::warning(this, "警告", "无法保存点云颜色元数据");
          return;
        }
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
      pp.z = p.z + offset.z;
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
