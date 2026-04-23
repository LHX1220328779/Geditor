
#pragma once

#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <memory>

#include "ui/tab_att.h"

namespace Ui {
class GeditorMainWindow;
}

class GeditorMainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit GeditorMainWindow(QWidget *parent = 0);

  ~GeditorMainWindow();

  /// 在状态栏显示信息
  void ShowMeasureDistance(double distance);

  void ShowMousePosition(double lon, double lat, double x, double y);

  void ShowMapView();

 private slots:
  void on_button_2D_clicked();
  void on_button_3D_clicked();
  void on_move_status_clicked();
  void on_delete_status_clicked();
  void on_measure_status_clicked();
  void on_direction_status_toggled(bool on);
  void on_btn_direction_up_toggled(bool on);
  void on_btn_direction_down_toggled(bool on);
  void on_button_redo_clicked();
  void on_button_undo_clicked();
  void on_import_pdb_triggered();
  void on_show_line_toggled(bool arg1);
  void on_show_roadarea_toggled(bool arg1);
  void on_show_lane_toggled(bool arg1);
  void on_show_pdb_toggled(bool arg1);
  void on_show_sign_toggled(bool arg1);
  void on_show_tra_toggled(bool arg1);
  void on_draw_line_clicked();
  void on_draw_lane_clicked();
  void on_draw_roadarea_clicked();
  void on_draw_sign_clicked();
  void on_draw_jobarea_clicked();
  void on_draw_jobpoint_clicked();
  void on_edit_status_clicked();
  void on_generate_topo_clicked();
  void on_import_vdb_triggered();
  void on_export_vdb_triggered();
  void update();

  void on_set_cloud_triggered();
  void on_instruction_triggered();

  void on_db2pdb_triggered();
  void on_pcd2pdb_triggered();
  void on_llh2utm_triggered();

  void on_check_relation_clicked();

  void on_import_tra_triggered();

  void on_view_pan_triggered();

  void on_combox_line_currentIndexChanged(int index);

  void on_upload_map_triggered();
  void on_actionDownload_pdb_triggered();
  void on_actionDownload_vdb_triggered();

  void on_action_AUTO_BIND_TRAJECTORY_triggered();

  void on_action_QUICK_PICK_TRAJ_triggered();
  void on_action_QUICK_AUTO_BIND_triggered();
  void on_action_QUICK_ROAD_RIGHT_triggered();
  void on_action_QUICK_EXPORT_ROAD_RIGHT_triggered();
  void on_action_QUICK_IMPORT_ROAD_RIGHT_triggered();
  void on_action_QUICK_CHECK_CONNECTIVITY_triggered();
  void on_action_QUICK_VIZ_ROAD_RIGHT_toggled(bool on);
  void on_action_QUICK_ADAPTIVE_BOUNDARY_triggered();

 private:
  void on_action_SHOWHIDE_TRACK_toggled(bool arg1);

  void on_action_CLOSE_PDB_triggered();

  void on_display_widget_destroyed();

  void on_action_IMPORT_FUNCTION_POINT_triggered();

  void on_action_LOAD_TRACK_triggered();

  void on_action_DATA_CLOSE_triggered();

  void on_action_EXIT_triggered();

  void on_action_VIEW_TOOLBAR_triggered();

  void on_action_VIEW_STATUS_BAR_toggled(bool arg1);

  void on_action_ZOOM_IN_triggered();

  void on_action_ZOOM_OUT_triggered();

  void on_action_CLIPER_POLYGON_triggered();

  void on_action_ACTIVE_CBOUNDER_toggled(bool arg1);

  void on_action_ACTIVE_JOB_toggled(bool arg1);

  void on_action_ACTIVE_TOPOLOGY_toggled(bool arg1);

  void on_action_ACTIVE_REGION_toggled(bool arg1);

  void on_action_ACTIVE_SEGMENT_toggled(bool arg1);

  void on_action_DRAW_BEZIER_CURVE_triggered();

  void on_action_DRAW_BSPLINE_CURVE_triggered();

  void on_action_DRAW_HERMITE_CURVE_triggered();

  void on_action_EDIT_SEGMENT_triggered();

  // void on_action_AREA_PATH_POINT_triggered();

  // void on_action_JOB_MAP_TOPOLOGY_triggered();

  void on_action_AREA_TAKEOVER_triggered();

  void on_action_SETTING_DLG_triggered();

  void on_actionDRAW_BEIZER_triggered();

  void on_actionDRAW_BSPLINE_triggered();

  void on_actionEDIT_SEGMENT_triggered();

  void on_actionMERGE_SEGMENT_triggered();

  void on_listView_clicked(const QModelIndex &index);

 public:
  void SetMode(int s);

 private:
  Ui::GeditorMainWindow *ui;

  std::shared_ptr<QTimer> timer_ = nullptr;

  // 状态栏
  QLabel *status_label_ = nullptr;
  QLabel *position_label_ = nullptr;
  QLabel *distance_label_ = nullptr;

  geditor::TabAtt *tab_att_;
};