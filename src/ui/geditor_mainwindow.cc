#include "geditor_mainwindow.h"

#include <QAction>
#include <QFileDialog>
#include <QLabel>
#include <QMessageBox>
#include <QStringListModel>
#include <QtWidgets/QMainWindow>

#include <QFileDialog>
#include <QTextEdit>
#include <QVBoxLayout>

#include "autobinddialog.h"
#include "core/log.h"
#include "core/sign_board.h"
#include "dbimportdialog.h"
#include "downloadpdbdialog.h"
#include "downloadvdbdialog.h"
#include "pcdimportdialog.h"
#include "adaptive_boundary_dialog.h"
#include "roadrightdialog.h"
#include "ui_geditor_mainwindow.h"
#include "uploaddialog.h"

GeditorMainWindow::GeditorMainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::GeditorMainWindow) {
  ui->setupUi(this);

  ui->statusBar->setSizeGripEnabled(false);
  status_label_ = new QLabel("就绪", this);
  position_label_ = new QLabel("经度=0, 纬度=0, x=0, y=0", this);
  distance_label_ = new QLabel("测距", this);

  ui->statusBar->addPermanentWidget(status_label_);
  ui->statusBar->addPermanentWidget(position_label_);
  ui->statusBar->addPermanentWidget(distance_label_);

  status_label_->setMinimumWidth(120);
  position_label_->setMinimumWidth(240);
  distance_label_->setMinimumWidth(240);

  ui->tabWidgetFuns->setTabText(0, "矢量地图");
  ui->tabWidgetFuns->setTabText(1, "点云地图");
  ui->tabWidgetFuns->setCurrentIndex(0);
  setStyleSheet("QRadioButton::indicator{width:25px;height:25px;}");
  tab_att_ = new geditor::TabAtt(ui->tab_att);

  timer_ = std::make_shared<QTimer>(this);
  connect(timer_.get(), SIGNAL(timeout()), this, SLOT(update()));
  timer_->start(50);
}

GeditorMainWindow::~GeditorMainWindow() { delete ui; }

void GeditorMainWindow::SetMode(int s) {
  switch (s) {
    case Qt::Key_Escape: {
      if (ui->display_widget->IsPickTrajectoryMode()) {
        ui->display_widget->CancelPickTrajectoryMode();
        return;
      }
      if (ui->display_widget->IsPickDirectionMode()) {
        ui->display_widget->CancelPickDirectionMode();
        if (ui->direction_status->isChecked()) ui->edit_status->click();
        return;
      }
      ui->edit_status->click();
      break;
    }
  }
}
void GeditorMainWindow::on_listView_clicked(const QModelIndex &index) {
  switch (index.row() + 1) {
    case 4:
      ui->display_widget->OnActiveBoundary();
      break;
    case 5:
      ui->display_widget->OnActiveJob();
      break;
    case 6:
      ui->display_widget->OnActiveTopology();
      break;
    case 2:
      ui->display_widget->OnActiveRegion();
      break;
    case 1:
      ui->display_widget->OnActiveSegment();
      break;
    case 3:
      ui->display_widget->OnActiveFunPoint();
      break;
    default:
      break;
  }
  LOG(INFO) << index.data().toString().toStdString() << index.row();
}

void GeditorMainWindow::on_db2pdb_triggered() {
  DBImportDialog dlg(this);
  dlg.exec();
}

void GeditorMainWindow::on_pcd2pdb_triggered() {
  PCDImportDialog dlg(this);
  dlg.exec();
}

void GeditorMainWindow::on_upload_map_triggered() {
  UploadDialog dlg(this);
  dlg.exec();
}

void GeditorMainWindow::on_import_pdb_triggered() {
  QFileDialog fd(this, "Select file", "", "数据文件(*.pdb);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];
    ui->display_widget->OnOpenPDB(sel.toUtf8().constData());
  }
}

void GeditorMainWindow::ShowMeasureDistance(double distance) {
  QString str;
  str.sprintf(R"(总长:%.03fm)", distance);
  distance_label_->setText(str);
}

void GeditorMainWindow::ShowMousePosition(double lon, double lat, double x,
                                          double y) {
  QString str;
  str.sprintf("经度=%.09f, 纬度=%.09f, x=%.02f, y=%.02f", lon, lat, x, y);
  position_label_->setText(str);
}

QVector<QString> MOSTR = {"浏览", "绘制", "编辑", "平移",
                          "打断", "测距", "切割", "点锚"};
void GeditorMainWindow::ShowMapView() {
  QString str;
  if (ui->display_widget->GetViewMode() == 0) {
    str = ("二维浏览");
  } else {
    str = ("三维浏览");
  }
  int type = ui->display_widget->GetMapMode();
  if (type >= 0 && type < MOSTR.size()) {
    str += " ";
    str += MOSTR[type];
  }
  status_label_->setText(str);
}

void GeditorMainWindow::update() {
  tab_att_->OnSelected(ui->display_widget->GetSelected());
  ui->display_widget->update();
  ShowMapView();
}

void GeditorMainWindow::on_action_CLOSE_PDB_triggered() {
  ui->display_widget->OnClosePDB();
}

void GeditorMainWindow::on_display_widget_destroyed() {
  ui->display_widget->OnDestroy();
}

void GeditorMainWindow::on_action_IMPORT_FUNCTION_POINT_triggered() {
  ui->display_widget->OnImportFunctionPoint();
}

void GeditorMainWindow::on_view_pan_triggered() {
  ui->display_widget->OnPanMap();
}

void GeditorMainWindow::on_import_tra_triggered() {
  ui->display_widget->OnImportGps();
}

void GeditorMainWindow::on_action_LOAD_TRACK_triggered() {
  ui->display_widget->OnLoadTrack();
}

void GeditorMainWindow::on_import_vdb_triggered() {
  ui->display_widget->OnOpenVdb();
}

void GeditorMainWindow::on_export_vdb_triggered() {
  ui->display_widget->GenerateTopology();
  ui->display_widget->OnDataSave();
}

void GeditorMainWindow::on_action_DATA_CLOSE_triggered() {
  ui->display_widget->OnDataClose();
}

void GeditorMainWindow::on_action_EXIT_triggered() { close(); }

void GeditorMainWindow::on_llh2utm_triggered() {
  ui->display_widget->OnConvertCoord();
}

void GeditorMainWindow::on_action_VIEW_TOOLBAR_triggered() {
  LOG(INFO) << "View tool bar not implemented yet";
}

void GeditorMainWindow::on_action_VIEW_STATUS_BAR_toggled(bool arg1) {
  if (arg1) {
    this->statusBar()->show();
  } else {
    this->statusBar()->hide();
  }
}

void GeditorMainWindow::on_button_undo_clicked() {
  ui->display_widget->OnUndoOperate();
}

void GeditorMainWindow::on_button_redo_clicked() {
  ui->display_widget->OnRedoOperate();
}

void GeditorMainWindow::on_action_ZOOM_IN_triggered() {
  ui->display_widget->OnMapZoomIn();
}

void GeditorMainWindow::on_action_ZOOM_OUT_triggered() {
  ui->display_widget->OnMapZoomOut();
}

void GeditorMainWindow::on_move_status_clicked() {
  ui->display_widget->OnMoveObject();
}

void GeditorMainWindow::on_delete_status_clicked() {
  ui->display_widget->OnDeleteObject();
}

void GeditorMainWindow::on_measure_status_clicked() {
  ui->display_widget->OnMeasureDistance();
}

// 设定行驶方向：direction_status 作为模式总开关（与其它 radio 互斥）；
// 选中时启用上山/下山两个子按钮与高亮 overlay；选中其它模式时自动收起。
void GeditorMainWindow::on_direction_status_toggled(bool on) {
  ui->btn_direction_up->setEnabled(on);
  ui->btn_direction_down->setEnabled(on);
  if (auto *fw = ui->display_widget->framework()) {
    fw->SetShowDirectionOverlay(on);
  }
  if (!on) {
    // 退出主按钮时清理子按钮与模式
    ui->btn_direction_up->setChecked(false);
    ui->btn_direction_down->setChecked(false);
    if (ui->display_widget->IsPickDirectionMode()) {
      ui->display_widget->CancelPickDirectionMode();
    }
  }
  ui->display_widget->update();
}

void GeditorMainWindow::on_btn_direction_up_toggled(bool on) {
  if (on) {
    ui->btn_direction_down->blockSignals(true);
    ui->btn_direction_down->setChecked(false);
    ui->btn_direction_down->blockSignals(false);
    ui->display_widget->StartPickDirectionMode(1);
  } else if (ui->display_widget->PickDirection() == 1) {
    ui->display_widget->CancelPickDirectionMode();
  }
}

void GeditorMainWindow::on_btn_direction_down_toggled(bool on) {
  if (on) {
    ui->btn_direction_up->blockSignals(true);
    ui->btn_direction_up->setChecked(false);
    ui->btn_direction_up->blockSignals(false);
    ui->display_widget->StartPickDirectionMode(2);
  } else if (ui->display_widget->PickDirection() == 2) {
    ui->display_widget->CancelPickDirectionMode();
  }
}

void GeditorMainWindow::on_action_CLIPER_POLYGON_triggered() {
  ui->display_widget->OnCliperPolygon();
}

void GeditorMainWindow::on_action_ACTIVE_CBOUNDER_toggled(bool arg1) {
  if (arg1) ui->display_widget->OnActiveBoundary();
}

void GeditorMainWindow::on_action_ACTIVE_JOB_toggled(bool arg1) {
  if (arg1) ui->display_widget->OnActiveJob();
}

void GeditorMainWindow::on_action_ACTIVE_TOPOLOGY_toggled(bool arg1) {
  if (arg1) ui->display_widget->OnActiveTopology();
}

void GeditorMainWindow::on_action_ACTIVE_REGION_toggled(bool arg1) {
  if (arg1) ui->display_widget->OnActiveRegion();
}

void GeditorMainWindow::on_action_ACTIVE_SEGMENT_toggled(bool arg1) {
  if (arg1) ui->display_widget->OnActiveSegment();
}

void GeditorMainWindow::on_action_SHOWHIDE_TRACK_toggled(bool arg1) {
  ui->display_widget->OnShowhideTrack(arg1);
}

void GeditorMainWindow::on_show_line_toggled(bool arg1) {
  ui->display_widget->OnShowhideBoundary(arg1);
}

void GeditorMainWindow::on_show_roadarea_toggled(bool arg1) {
  ui->display_widget->OnShowhideJobmap(arg1);
}

void GeditorMainWindow::on_show_lane_toggled(bool arg1) {
  ui->display_widget->OnShowhideRoad(arg1);
}

void GeditorMainWindow::on_show_pdb_toggled(bool arg1) {
  ui->display_widget->OnShowhidePDB(arg1);
}

void GeditorMainWindow::on_show_sign_toggled(bool arg1) {
  ui->display_widget->OnShowhideSign(arg1);
}

void GeditorMainWindow::on_show_tra_toggled(bool arg1) {
  ui->display_widget->OnShowhideTrack(arg1);
}

void GeditorMainWindow::on_action_DRAW_BEZIER_CURVE_triggered() {
  ui->display_widget->OnDrawBezierCurve();
}

void GeditorMainWindow::on_action_DRAW_BSPLINE_CURVE_triggered() {
  ui->display_widget->OnDrawBsplineCurve();
}

void GeditorMainWindow::on_action_DRAW_HERMITE_CURVE_triggered() {
  ui->display_widget->OnDrawHermiteCurve();
}

// void GeditorMainWindow::on_action_JOB_MAP_TOPOLOGY_triggered() {
//   ui->display_widget->OnJobMapTopology();
// }

// void GeditorMainWindow::on_action_AREA_PATH_POINT_triggered() {
//   LOG(WARNING)<<"xxxxxxx";
//   ui->display_widget->OnPathPoint();
// }

void GeditorMainWindow::on_action_EDIT_SEGMENT_triggered() {
  ui->display_widget->OnEditSegment();
}

void GeditorMainWindow::on_action_AREA_TAKEOVER_triggered() {
  // ui->display_widget->OnAreaTakeOver();
}

void GeditorMainWindow::on_action_SETTING_DLG_triggered() {
  ui->display_widget->OnSettingDlg();
}
void GeditorMainWindow::on_actionDRAW_BEIZER_triggered() {
  on_action_DRAW_BEZIER_CURVE_triggered();
}

void GeditorMainWindow::on_actionDRAW_BSPLINE_triggered() {
  on_action_DRAW_BSPLINE_CURVE_triggered();
}

void GeditorMainWindow::on_actionEDIT_SEGMENT_triggered() {
  on_action_EDIT_SEGMENT_triggered();
}

void GeditorMainWindow::on_actionMERGE_SEGMENT_triggered() {}

void GeditorMainWindow::on_button_2D_clicked() {
  ui->edit_status->click();
  ui->display_widget->On2dView();
}

void GeditorMainWindow::on_button_3D_clicked() {
  ui->scan_status->click();
  ui->display_widget->On3dView();
}

void GeditorMainWindow::on_draw_line_clicked() {
  ui->display_widget->OnDrawLine();
}
void GeditorMainWindow::on_draw_lane_clicked() {
  ui->display_widget->OnDrawLane();
}
void GeditorMainWindow::on_draw_roadarea_clicked() {
  ui->display_widget->OnAreaRoad();
}

void GeditorMainWindow::on_draw_sign_clicked() {
  ui->display_widget->OnTrafficSign(999);
}
void GeditorMainWindow::on_draw_jobarea_clicked() {
  ui->display_widget->OnAreaJob();
}
void GeditorMainWindow::on_draw_jobpoint_clicked() {
  ui->display_widget->OnPointJob();
}
void GeditorMainWindow::on_edit_status_clicked() {
  ui->display_widget->OnEditObject();
}

void GeditorMainWindow::on_generate_topo_clicked() {
  ui->display_widget->GenerateTopology();
  // ui->display_widget->OnJobMapTopology();
}

void GeditorMainWindow::on_set_cloud_triggered() {
  ui->display_widget->OnPointFilter();
}

void GeditorMainWindow::on_check_relation_clicked() {
  ui->display_widget->OnCheckRelation();
}

void GeditorMainWindow::on_combox_line_currentIndexChanged(int index) {
  ui->display_widget->ChangeLineType(index);
}

void GeditorMainWindow::on_instruction_triggered() {
  QString txt =
      "1.按住ctrl后左键单击进行多选<br>2.选中对象后右键可进行对象关联<br>3."
      "选中对象后右侧表格显示对象属性，并可进行属性设置<br>4."
      "编辑模式按住shift单击可插入点;按住鼠标滚轮移动地图;滚动滚轮放大缩小";
  QMessageBox::about(this, "Geditor使用说明", txt);
}

void GeditorMainWindow::on_actionDownload_pdb_triggered() {
  DownloadPdbDialog dlg(this);
  dlg.exec();
}

void GeditorMainWindow::on_actionDownload_vdb_triggered() {
  DownloadVdbDialog dlg(this);
  dlg.exec();
}

void GeditorMainWindow::on_action_AUTO_BIND_TRAJECTORY_triggered() {
  AutoBindDialog dlg(ui->display_widget, this);
  dlg.exec();
  // 属性可能已被批量修改，强制属性面板下次重新渲染
  tab_att_->InvalidateSelection();
  ui->display_widget->update();
}

// ============ 快捷功能菜单槽 ============

void GeditorMainWindow::on_action_QUICK_PICK_TRAJ_triggered() {
  ui->display_widget->StartPickTrajectoryMode();
}

void GeditorMainWindow::on_action_QUICK_AUTO_BIND_triggered() {
  AutoBindDialog dlg(ui->display_widget, this);
  dlg.exec();
  tab_att_->InvalidateSelection();
  ui->display_widget->update();
}

void GeditorMainWindow::on_action_QUICK_ROAD_RIGHT_triggered() {
  RoadRightDialog dlg(ui->display_widget, this);
  dlg.exec();
  tab_att_->InvalidateSelection();
  ui->display_widget->update();
}

void GeditorMainWindow::on_action_QUICK_EXPORT_ROAD_RIGHT_triggered() {
  QString path = QFileDialog::getSaveFileName(
      this, "导出 road_right.txt", "road_right.txt", "Text (*.txt)");
  if (path.isEmpty()) return;
  geditor::Framework::RoadRightIOStat stat;
  bool ok = ui->display_widget->OnExportRoadRight(path.toStdString(), &stat);
  if (!ok) {
    QMessageBox::warning(this, "导出失败", "无法写入文件: " + path);
    return;
  }
  QMessageBox::information(this, "导出 road_right.txt",
                           QString("已导出 %1 条路段。\n%2")
                               .arg(stat.written)
                               .arg(QString::fromStdString(stat.detail)));
}

void GeditorMainWindow::on_action_QUICK_IMPORT_ROAD_RIGHT_triggered() {
  QString path = QFileDialog::getOpenFileName(this, "导入 road_right.txt", "",
                                              "Text (*.txt)");
  if (path.isEmpty()) return;
  geditor::Framework::RoadRightIOStat stat;
  bool ok = ui->display_widget->OnImportRoadRight(path.toStdString(), &stat);
  if (!ok) {
    QMessageBox::warning(this, "导入失败", "无法读取文件: " + path);
    return;
  }
  tab_att_->InvalidateSelection();
  QMessageBox::information(this, "导入 road_right.txt",
                           QString("匹配 %1 条  未匹配 %2 条\n%3")
                               .arg(stat.matched)
                               .arg(stat.unmatched)
                               .arg(QString::fromStdString(stat.detail)));
}

void GeditorMainWindow::on_action_QUICK_CHECK_CONNECTIVITY_triggered() {
  geditor::Framework::ConnectivityDiagnostic diag;
  int isolated = ui->display_widget->OnCheckLaneConnectivity(&diag);
  QDialog dlg(this);
  dlg.setWindowTitle("中心线连通性校验");
  dlg.resize(720, 520);
  auto *layout = new QVBoxLayout(&dlg);
  auto *lbl = new QLabel(
      QString("总数=%1  起始=%2  终止=%3  中间=%4  孤立=%5")
          .arg(diag.totalLanes)
          .arg(diag.starts)
          .arg(diag.ends)
          .arg(diag.middles)
          .arg(diag.isolated),
      &dlg);
  if (isolated > 0)
    lbl->setStyleSheet("color:#a00;font-weight:bold;");
  else
    lbl->setStyleSheet("color:#080;font-weight:bold;");
  layout->addWidget(lbl);
  auto *txt = new QTextEdit(&dlg);
  txt->setReadOnly(true);
  txt->setPlainText(QString::fromStdString(diag.detail));
  layout->addWidget(txt, 1);
  dlg.exec();
}

void GeditorMainWindow::on_action_QUICK_VIZ_ROAD_RIGHT_toggled(bool on) {
  if (auto *fw = ui->display_widget->framework()) {
    fw->SetVizRoadRight(on);
  }
  ui->display_widget->update();
}

void GeditorMainWindow::on_action_QUICK_ADAPTIVE_BOUNDARY_triggered() {
  AdaptiveBoundaryDialog dlg(ui->display_widget, this);
  dlg.exec();
  tab_att_->InvalidateSelection();
  ui->display_widget->update();
}
