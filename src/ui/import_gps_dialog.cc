#include "import_gps_dialog.h"

#include <QApplication>
#include <QDebug>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QSet>
#include <QTabWidget>
#include <QTextStream>
#include <QVBoxLayout>
#include <cmath>
#include <fstream>
#include <sstream>

#include "core/point3d.h"
#include "map/projection_utm.h"
#include "trajectory_preprocessor.h"
#include "ui_importgpsdialog.h"

using namespace geditor;

ImportGPSDialog::ImportGPSDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::ImportGPSDialog) {
  ui->setupUi(this);
  setWindowTitle("导入轨迹");
  setMinimumSize(820, 640);
  resize(920, 720);

  model_ = new QStandardItemModel();
  ui->tableView->setModel(model_);
  ui->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
  model_->setColumnCount(2);
  model_->setHeaderData(0, Qt::Horizontal, "序号");
  model_->setHeaderData(1, Qt::Horizontal, "文件路径");
  ui->tableView->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::ResizeToContents);
  ui->tableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  ui->tableView->setAlternatingRowColors(true);

  auto *legacy_button_panel = ui->add_btn->parentWidget();
  for (QPushButton *button :
       {ui->add_btn, ui->delete_btn, ui->clear_btn, ui->import_btn}) {
    ui->verticalLayout->removeWidget(button);
  }

  auto *root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(16, 16, 16, 16);
  root_layout->setSpacing(12);
  auto *title = new QLabel("轨迹数据导入与格式处理", this);
  title->setStyleSheet("font-size:18px;font-weight:600;");
  root_layout->addWidget(title);
  auto *description = new QLabel(
      "按任务选择处理方式。生成或转换后的文件会加入待导入列表，"
      "点击底部“导入轨迹”后才写入当前地图。",
      this);
  description->setWordWrap(true);
  root_layout->addWidget(description);

  workflow_tabs_ = new QTabWidget(this);
  root_layout->addWidget(workflow_tabs_, 1);

  auto *direct_tab = new QWidget(workflow_tabs_);
  auto *direct_layout = new QVBoxLayout(direct_tab);
  auto *direct_help = new QLabel(
      "添加 TXT/CSV 轨迹。自动模式按文件结构区分 GPS 经纬度 CSV、"
      "本地 XY CSV 和带索引 TXT。",
      direct_tab);
  direct_help->setWordWrap(true);
  direct_layout->addWidget(direct_help);
  auto *coordinate_layout = new QHBoxLayout();
  coordinate_layout->addWidget(new QLabel("坐标解释", direct_tab));
  coordinate_mode_combo_ = new QComboBox(direct_tab);
  coordinate_mode_combo_->addItems(
      {"自动识别（推荐）", "WGS84 经度/纬度", "本地 XY 米制坐标"});
  coordinate_layout->addWidget(coordinate_mode_combo_);
  coordinate_layout->addStretch();
  direct_layout->addLayout(coordinate_layout);
  direct_layout->addWidget(ui->tableView, 1);
  auto *file_buttons = new QHBoxLayout();
  ui->add_btn->setText("添加文件…");
  auto *add_directory_btn = new QPushButton("添加目录…", direct_tab);
  ui->delete_btn->setText("移除所选");
  ui->clear_btn->setText("清空列表");
  file_buttons->addWidget(ui->add_btn);
  file_buttons->addWidget(add_directory_btn);
  file_buttons->addStretch();
  file_buttons->addWidget(ui->delete_btn);
  file_buttons->addWidget(ui->clear_btn);
  direct_layout->addLayout(file_buttons);
  workflow_tabs_->addTab(direct_tab, "1  直接导入");
  connect(add_directory_btn, &QPushButton::clicked, this,
          &ImportGPSDialog::AddTrajectoryDirectory);

  auto *preprocess_tab = new QWidget(workflow_tabs_);
  auto *preprocess_layout = new QVBoxLayout(preprocess_tab);
  auto *preprocess_help = new QLabel(
      "为生产路线 CSV 生成 mapping.txt 和带稳定索引的轨迹 TXT；"
      "旧 mapping.txt 可选，用于复用既有索引。",
      preprocess_tab);
  preprocess_help->setWordWrap(true);
  preprocess_layout->addWidget(preprocess_help);
  auto *preprocess_group = new QGroupBox("输入与输出", preprocess_tab);
  preprocess_layout->addWidget(preprocess_group);
  auto *grid = new QGridLayout(preprocess_group);
  production_route_edit_ = new QLineEdit(preprocess_group);
  production_route_edit_->setPlaceholderText("选择包含 CSV 路线文件的目录");
  auto *production_route_btn = new QPushButton("浏览…", preprocess_group);
  grid->addWidget(new QLabel("生产路线 CSV 目录", preprocess_group), 0, 0);
  grid->addWidget(production_route_edit_, 0, 1);
  grid->addWidget(production_route_btn, 0, 2);
  mapping_output_edit_ = new QLineEdit(preprocess_group);
  mapping_output_edit_->setPlaceholderText(
      "输出 mapping.txt 和轨迹 TXT 的目录");
  auto *mapping_output_btn = new QPushButton("浏览…", preprocess_group);
  grid->addWidget(new QLabel("生成目录", preprocess_group), 1, 0);
  grid->addWidget(mapping_output_edit_, 1, 1);
  grid->addWidget(mapping_output_btn, 1, 2);
  old_mapping_edit_ = new QLineEdit(preprocess_group);
  old_mapping_edit_->setPlaceholderText("留空时从 1 开始创建全新映射");
  auto *old_mapping_btn = new QPushButton("浏览…", preprocess_group);
  auto *clear_old_mapping_btn = new QPushButton("清空", preprocess_group);
  auto *old_mapping_buttons = new QHBoxLayout();
  old_mapping_buttons->addWidget(old_mapping_btn);
  old_mapping_buttons->addWidget(clear_old_mapping_btn);
  grid->addWidget(new QLabel("旧 mapping.txt（可选）", preprocess_group), 2, 0);
  grid->addWidget(old_mapping_edit_, 2, 1);
  grid->addLayout(old_mapping_buttons, 2, 2);
  grid->setColumnStretch(1, 1);
  auto *generate_btn =
      new QPushButton("生成并加入待导入列表", preprocess_tab);
  preprocess_layout->addWidget(generate_btn, 0, Qt::AlignLeft);
  preprocess_status_ =
      new QLabel("尚未执行。旧 mapping.txt 留空不会阻止生成。", preprocess_tab);
  preprocess_status_->setWordWrap(true);
  preprocess_status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  preprocess_layout->addWidget(preprocess_status_);
  preprocess_layout->addStretch();
  workflow_tabs_->addTab(preprocess_tab, "2  路线包预处理");

  connect(production_route_btn, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getExistingDirectory(
        this, "选择生产路线包", production_route_edit_->text());
    if (!path.isEmpty()) production_route_edit_->setText(path);
  });
  connect(mapping_output_btn, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getExistingDirectory(
        this, "选择 mapping.txt 输出目录", mapping_output_edit_->text());
    if (!path.isEmpty()) mapping_output_edit_->setText(path);
  });
  connect(old_mapping_btn, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getOpenFileName(
        this, "选择旧 mapping.txt", old_mapping_edit_->text(),
        "mapping文件 (*.txt);;所有文件 (*)");
    if (!path.isEmpty()) old_mapping_edit_->setText(path);
  });
  connect(clear_old_mapping_btn, &QPushButton::clicked, old_mapping_edit_,
          &QLineEdit::clear);
  connect(generate_btn, &QPushButton::clicked, this,
          [this]() { GenerateMapping(true); });

  auto *conversion_tab = new QWidget(workflow_tabs_);
  auto *conversion_layout = new QVBoxLayout(conversion_tab);
  auto *conversion_help = new QLabel(
      "将本地 ENU 格式 x,y,heading,v,gear,kk 批量转换为 WGS84 格式 "
      "x(经度),y(纬度),heading,v；每个输入文件独立输出为 *_gps.csv。",
      conversion_tab);
  conversion_help->setWordWrap(true);
  conversion_layout->addWidget(conversion_help);
  auto *conversion_group = new QGroupBox("转换参数", conversion_tab);
  auto *conversion_grid = new QGridLayout(conversion_group);
  segment_map_edit_ = new QLineEdit(conversion_group);
  segment_map_edit_->setPlaceholderText("选择 segment_map 目录");
  auto *select_segment_map =
      new QPushButton("浏览…", conversion_group);
  conversion_grid->addWidget(new QLabel("segment_map 目录", conversion_group),
                             0, 0);
  conversion_grid->addWidget(segment_map_edit_, 0, 1);
  conversion_grid->addWidget(select_segment_map, 0, 2);
  gps_output_edit_ = new QLineEdit(conversion_group);
  gps_output_edit_->setPlaceholderText("默认在 Rtk_Map 下生成 gps_full");
  auto *select_gps_output = new QPushButton("浏览…", conversion_group);
  conversion_grid->addWidget(new QLabel("gps_full 输出目录", conversion_group),
                             1, 0);
  conversion_grid->addWidget(gps_output_edit_, 1, 1);
  conversion_grid->addWidget(select_gps_output, 1, 2);
  origin_display_ =
      new QLabel("未设置；请先在矿山原点弹窗中确认", conversion_group);
  origin_display_->setWordWrap(true);
  origin_display_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  conversion_grid->addWidget(new QLabel("当前计算原点", conversion_group),
                             2, 0);
  conversion_grid->addWidget(origin_display_, 2, 1, 1, 2);
  conversion_grid->setColumnStretch(1, 1);
  conversion_layout->addWidget(conversion_group);
  auto *conversion_actions = new QHBoxLayout();
  convert_segment_button_ =
      new QPushButton("开始批量转换", conversion_tab);
  convert_segment_button_->setEnabled(false);
  add_converted_check_ =
      new QCheckBox("转换完成后加入待导入列表", conversion_tab);
  add_converted_check_->setChecked(true);
  conversion_actions->addWidget(convert_segment_button_);
  conversion_actions->addWidget(add_converted_check_);
  conversion_actions->addStretch();
  conversion_layout->addLayout(conversion_actions);
  conversion_status_ = new QLabel("尚未执行。", conversion_tab);
  conversion_status_->setWordWrap(true);
  conversion_status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  conversion_layout->addWidget(conversion_status_);
  conversion_layout->addStretch();
  workflow_tabs_->addTab(conversion_tab, "3  segment_map 转 GPS");

  connect(select_segment_map, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getExistingDirectory(
        this, "选择 segment_map 目录", segment_map_edit_->text());
    if (path.isEmpty()) return;
    segment_map_edit_->setText(path);
    const QDir input(path);
    if (input.dirName().compare("segment_map", Qt::CaseInsensitive) == 0) {
      gps_output_edit_->setText(
          QDir(input.absolutePath() + "/..").absoluteFilePath("gps_full"));
    }
  });
  connect(select_gps_output, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getExistingDirectory(
        this, "选择 gps_full 输出目录", gps_output_edit_->text());
    if (!path.isEmpty()) gps_output_edit_->setText(path);
  });
  connect(convert_segment_button_, &QPushButton::clicked, this,
          [this]() { ConvertSegmentMap(true); });

  auto *legacy_import_type_panel = ui->radioButton->parentWidget();
  for (QRadioButton *radio :
       {ui->radioButton, ui->radioButton_2, ui->radioButton_3}) {
    ui->horizontalLayout->removeWidget(radio);
  }
  auto *import_type_layout = new QHBoxLayout(ui->groupBox);
  import_type_layout->addWidget(ui->radioButton);
  import_type_layout->addWidget(ui->radioButton_2);
  import_type_layout->addWidget(ui->radioButton_3);
  import_type_layout->addStretch();
  legacy_import_type_panel->hide();
  root_layout->addWidget(ui->groupBox);
  ui->groupBox->setTitle("导入为");
  auto *bottom_buttons = new QHBoxLayout();
  bottom_buttons->addStretch();
  auto *cancel_button = new QPushButton("取消", this);
  ui->import_btn->setText("导入轨迹");
  ui->import_btn->setDefault(true);
  bottom_buttons->addWidget(cancel_button);
  bottom_buttons->addWidget(ui->import_btn);
  root_layout->addLayout(bottom_buttons);
  connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
  legacy_button_panel->hide();
}

ImportGPSDialog::~ImportGPSDialog() {
  delete ui;
  delete model_;
}

void ImportGPSDialog::SetFrame(int t) {
  frame_type_ = t == 0 ? 0 : 1;
  if (coordinate_mode_combo_) {
    coordinate_mode_combo_->setCurrentIndex(frame_type_ == 0 ? 1 : 2);
  }
}

void ImportGPSDialog::SetMineOrigin(const geditor::MineOrigin &origin) {
  mine_origin_ = origin;
  has_mine_origin_ = origin.zone >= 1 && origin.zone <= 60;
  if (!has_mine_origin_) {
    origin_display_->setText("原点无效；请返回主界面重新选择");
    convert_segment_button_->setEnabled(false);
    return;
  }
  origin_display_->setText(
      QString("%1  |  LAT=%2, LON=%3, ALT=%4, zone=%5")
          .arg(QString::fromStdString(origin.name))
          .arg(origin.latitude, 0, 'f', 9)
          .arg(origin.longitude, 0, 'f', 9)
          .arg(origin.z, 0, 'f', 3)
          .arg(origin.zone));
  convert_segment_button_->setEnabled(true);
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
    AddTrajectoryFiles(fd.selectedFiles());
  }
}

void ImportGPSDialog::AddTrajectoryFiles(const QStringList &files) {
  QSet<QString> existing;
  for (int row = 0; row < model_->rowCount(); ++row) {
    existing.insert(QDir::cleanPath(model_->item(row, 1)->text()));
  }
  for (const QString &file : files) {
    const QString clean_path = QDir::cleanPath(file);
    if (existing.contains(clean_path)) continue;
    const int row = model_->rowCount();
    model_->setItem(row, 0, new QStandardItem(QString::number(row + 1)));
    model_->setItem(row, 1, new QStandardItem(clean_path));
    existing.insert(clean_path);
  }
}

void ImportGPSDialog::AddTrajectoryDirectory() {
  const QString path = QFileDialog::getExistingDirectory(this, "选择轨迹目录");
  if (path.isEmpty()) return;
  QDir dir(path);
  QStringList files;
  const QFileInfoList entries = dir.entryInfoList(
      QStringList() << "*.txt" << "*.csv", QDir::Files | QDir::Readable,
      QDir::Name);
  for (const QFileInfo &entry : entries) {
    if (entry.fileName().compare("mapping.txt", Qt::CaseInsensitive) != 0) {
      files.push_back(entry.absoluteFilePath());
    }
  }
  if (files.isEmpty()) {
    QMessageBox::warning(this, "提示", "所选目录中没有轨迹txt/csv文件");
    return;
  }
  AddTrajectoryFiles(files);
}

bool ImportGPSDialog::GenerateMapping(bool show_success) {
  TrajectoryPreprocessResult result;
  QString error;
  if (!TrajectoryPreprocessor::Process(
          production_route_edit_->text().trimmed(),
          mapping_output_edit_->text().trimmed(),
          old_mapping_edit_->text().trimmed(), result, error)) {
    preprocess_status_->setText(error);
    QMessageBox::warning(this, "生成失败", error);
    return false;
  }

  AddTrajectoryFiles(result.trajectory_files);
  QString status =
      QString("已生成 %1；轨迹文件 %2 个，复用索引 %3 个，新建索引 %4 个。")
          .arg(result.mapping_file)
          .arg(result.trajectory_files.size())
          .arg(result.reused_count)
          .arg(result.created_count);
  if (!result.warning.isEmpty()) status += "\n" + result.warning;
  preprocess_status_->setText(status);
  if (!result.trajectory_files.isEmpty()) workflow_tabs_->setCurrentIndex(0);
  if (!result.warning.isEmpty()) {
    QMessageBox::warning(this, "生成完成（有提示）", status);
  } else if (show_success) {
    QMessageBox::information(this, "生成完成", status);
  }
  return true;
}

bool ImportGPSDialog::ConvertSegmentMap(bool show_success) {
  if (!has_mine_origin_) {
    const QString error = "尚未设置有效的矿山原点，无法进行坐标转换";
    conversion_status_->setText(error);
    QMessageBox::warning(this, "转换失败", error);
    return false;
  }
  SegmentMapConversionResult result;
  QString error;
  convert_segment_button_->setEnabled(false);
  conversion_status_->setText("正在转换，请稍候…");
  QApplication::setOverrideCursor(Qt::WaitCursor);
  const bool converted = TrajectoryPreprocessor::ConvertSegmentMapToGps(
      segment_map_edit_->text().trimmed(), gps_output_edit_->text().trimmed(),
      mine_origin_, result, error);
  QApplication::restoreOverrideCursor();
  convert_segment_button_->setEnabled(true);
  if (!converted) {
    conversion_status_->setText(error);
    QMessageBox::warning(this, "转换失败", error);
    return false;
  }

  if (add_converted_check_->isChecked()) {
    AddTrajectoryFiles(result.gps_files);
  }
  const QString status =
      QString("转换完成：%1 个文件，%2 个轨迹点。\n输出目录：%3\n"
              "使用原点：%4")
          .arg(result.gps_files.size())
          .arg(result.point_count)
          .arg(result.output_directory)
          .arg(QString::fromStdString(mine_origin_.name));
  conversion_status_->setText(status);
  if (add_converted_check_->isChecked()) workflow_tabs_->setCurrentIndex(0);
  if (show_success) QMessageBox::information(this, "转换完成", status);
  return true;
}

void ImportGPSDialog::on_delete_btn_clicked() {
  int row = ui->tableView->currentIndex().row();
  if (row < 0 || row >= model_->rowCount()) {
    QMessageBox::warning(this, "提示", "请选择要删除的行");
    return;
  }

  model_->removeRow(row);
  for (int i = row; i < model_->rowCount(); ++i) {
    model_->item(i, 0)->setText(QString::number(i + 1));
  }
}

void ImportGPSDialog::on_clear_btn_clicked() {
  model_->removeRows(0, model_->rowCount());
}

void ImportGPSDialog::on_import_btn_clicked() {
  for (auto *segment : m_segmentArray) delete segment;
  m_segmentArray.clear();
  QStringList failed_files;
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
    } else {
      failed_files.push_back(QFileInfo(data).fileName());
    }
  }
  if (!failed_files.isEmpty()) {
    QMessageBox::warning(
        this, "部分文件无法导入",
        QString("以下文件格式或坐标数据无效：\n%1")
            .arg(failed_files.join("\n")));
  }
  if (!m_segmentArray.empty()) accept();
}

bool ImportGPSDialog::ParsePointSet(
    const QString &filename, std::vector<geditor::TrackPoint> &pointSet) {
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    qWarning() << "Failed to open" << filename;
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  if (stream.atEnd()) return false;
  const QString header = stream.readLine().trimmed();
  const bool comma_separated = header.contains(',');
  const int header_columns =
      comma_separated ? header.split(',', Qt::KeepEmptyParts).size() : 0;
  const int selected_mode =
      coordinate_mode_combo_ ? coordinate_mode_combo_->currentIndex() : 0;
  // Automatic detection is structural, so local coordinates that happen to
  // fall inside longitude/latitude ranges are never guessed by value alone.
  const bool gps_coordinates =
      selected_mode == 1 || (selected_mode == 0 && comma_separated &&
                             header_columns == 4);

  ProjectionUTM projection;
  int line_number = 1;
  while (!stream.atEnd()) {
    QString line = stream.readLine().trimmed();
    ++line_number;
    if (line.isEmpty()) continue;
    if (comma_separated) line.replace(',', ' ');
    std::istringstream row(line.toStdString());
    double first = 0.0;
    double second = 0.0;
    double third = 0.0;
    if (!(row >> first >> second)) {
      qWarning() << "Invalid trajectory row in" << filename << "line"
                 << line_number;
      return false;
    }

    double x = first;
    double y = second;
    double index = -1.0;
    if (!comma_separated) {
      if (!(row >> third)) {
        qWarning() << "Invalid indexed trajectory row in" << filename
                   << "line" << line_number;
        return false;
      }
      index = first;
      x = second;
      y = third;
    }

    TrackPoint point;
    if (gps_coordinates) {
      const double longitude = x;
      const double latitude = y;
      if (!std::isfinite(longitude) || !std::isfinite(latitude) ||
          longitude < -180.0 || longitude > 180.0 || latitude < -80.0 ||
          latitude > 84.0) {
        qWarning() << "Invalid WGS84 coordinate in" << filename << "line"
                   << line_number;
        return false;
      }
      UTMPoint utm;
      projection.LatLonToCartesian(latitude, longitude, utm);
      point.pnt = Point3d(utm.x, utm.y, 0.0);
    } else {
      if (!std::isfinite(x) || !std::isfinite(y)) return false;
      point.pnt = Point3d(x, y, 0.0);
      point.idx = index > 0 ? index : -1;
    }
    pointSet.push_back(point);
  }
  if (pointSet.empty()) {
    qWarning() << "No valid trajectory points in" << filename;
    return false;
  }
  return true;
}

void ImportGPSDialog::on_radioButton_clicked() { import_type_ = 0; }

void ImportGPSDialog::on_radioButton_2_clicked() { import_type_ = 1; }

void ImportGPSDialog::on_radioButton_3_clicked() { import_type_ = 2; }
