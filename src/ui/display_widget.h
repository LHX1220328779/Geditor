
#pragma once

#include <QDialog>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QTimer>
#include <memory>

#include "algorithm/common.h"
#include "core/framework.h"
#include "core/point3d.h"
#include "geditor_mainwindow.h"
#include "map/feature_type.h"
#include "map/mine_origin_config.h"
#include "menu_widget.h"

enum {
  MO_MAP_SCAN = 0,     //地图浏览模式
  MO_DRAW_MAP = 1,     //几何绘制模式
  MO_MAP_EDIT = 2,     //控制点编辑，删除，增加
  MO_MOVE_OBJECT = 3,  //几何对象整体平移
  MO_BREAK_SEGMENT = 4,
  MO_DISTANCE = 5,      //测距模式
  MO_CLIPER = 6,        // 切割多边形
  MO_PICK_TRAJ = 7,     // 点轨迹起终点自动生成中心线
  MO_PICK_DIR_TRAJ = 8, // 设定行驶方向：点两轨迹节点，链式生成带方向中心线
};

class DisplayWidget : public QOpenGLWidget {
 public:
  DisplayWidget(QWidget *parent);

  virtual ~DisplayWidget() {}

 protected:
  /// QOpenGL Widget 接口
  virtual void initializeGL() override;

  virtual void resizeGL(int w, int h) override;

  virtual void paintGL() override;

 public:
  // 和主窗口交互
  void OnOpenPDB(const std::string &pdb_path);

  void OnClosePDB();

  void OnDestroy();

  void OnLButtonDown(QPoint point);

  void OnLButtonUp(QPoint point);

  void OnRButtonDown(QPoint point, bool dc);

  void OnMButtonDown(QPoint point);

  void OnMButtonUp(QPoint point);

  void OnImportFunctionPoint();

  void OnImportGps();

  void OnLoadTrack();

  void OnOpenVdb();

  void OnDataSave();

  void OnDataClose();

  void OnPanMap();

  void On3dView();

  void On2dView();

  void OnConvertCoord();

  void ApplyMineOrigin(const geditor::MineOrigin &origin);

  void OnUndoOperate();

  void OnRedoOperate();

  void OnMapZoomIn();

  void OnMapZoomOut();

  void OnEditObject();

  void OnMoveObject();

  void OnDeleteObject();

  void OnMeasureDistance();

  void OnCliperPolygon();

  void OnDrawLine();
  void OnDrawLane();

  void OnPointFilter();

  void OnActiveBoundary();

  void OnActiveJob();

  void OnActiveTopology();

  void OnActiveRegion();

  void OnActiveSegment();

  void OnActiveFunPoint();

  void OnShowhideTrack(bool show);

  void OnShowhideBoundary(bool show);

  void OnShowhideJobmap(bool show);

  void OnShowhideRoad(bool show);

  void OnShowhidePDB(bool show);

  void OnShowhideSign(bool show);

  void OnDrawBezierCurve();

  void OnDrawBsplineCurve();

  void OnDrawHermiteCurve();

  void OnEditSegment();

  void OnReverseSegment();

  void OnJobMapTopology();

  void OnAreaRoad();

  void OnAreaJob();

  void OnPointJob();

  void OnPathPoint();

  void OnTrafficSign(int type);

  void OnSettingDlg();

  void GenerateTopology();
  void OnCheckRelation();
  int OnAutoBindTrajectory(double distTol, double coverageThreshold,
                           bool dryRun,
                           geditor::Framework::AutoBindDiagnostic *diag);

  // 进入"点两下轨迹起/终点生成中心线"模式
  void StartPickTrajectoryMode();
  void CancelPickTrajectoryMode();
  bool IsPickTrajectoryMode() const { return m_MapMode == MO_PICK_TRAJ; }

  // 设定行驶方向模式：direction = 1(上山) / 2(下山)
  void StartPickDirectionMode(int direction);
  void CancelPickDirectionMode();
  bool IsPickDirectionMode() const { return m_MapMode == MO_PICK_DIR_TRAJ; }
  int PickDirection() const { return pick_dir_value_; }

  int OnAutoCheckRoadRight(double threshold,
                           geditor::Framework::RoadRightDiagnostic *diag);
  int OnCheckLaneConnectivity(
      geditor::Framework::ConnectivityDiagnostic *diag);
  bool OnExportRoadRight(const std::string &path,
                         geditor::Framework::RoadRightIOStat *stat);
  bool OnImportRoadRight(const std::string &path,
                         geditor::Framework::RoadRightIOStat *stat);

  int GetMapMode() { return m_MapMode; }
  int GetViewMode() { return view_mode_; }
  geditor::Framework *framework() { return &m_framework; }
  std::vector<geditor::MapFeature *> GetSelected();
  void ChangeLineType(int type);

 protected:
  /// 处理事件
  virtual void mousePressEvent(QMouseEvent *event);

  virtual void mouseReleaseEvent(QMouseEvent *event);

  virtual void mouseMoveEvent(QMouseEvent *event);

  virtual void keyPressEvent(QKeyEvent *event);

  virtual void keyReleaseEvent(QKeyEvent *event);

  virtual void wheelEvent(QWheelEvent *event);

  virtual void mouseDoubleClickEvent(QMouseEvent *event);

 private:
  void ConvertTrack(const std::vector<geditor::CurbsTrack *> &segmentArray,
                    std::vector<geditor::BoundSegment *> &segArray);

 private:
  void HandlePickTrajectoryClick(QPoint point);
  void HandlePickDirectionClick(QPoint point);

 private:
  geditor::Framework m_framework;

  // 点锚状态：第一次点击把起点缓存起来，第二次点击尝试连接并生成中心线
  geditor::BoundSegment *pick_traj_first_seg_ = nullptr;
  int pick_traj_first_idx_ = -1;

  // 方向拾取状态
  int pick_dir_value_ = 0;  // 1=上山 2=下山
  geditor::BoundSegment *pick_dir_first_seg_ = nullptr;
  int pick_dir_first_idx_ = -1;
  MenuWidget *menu_;
  int m_MapMode = 0;
  DrawType m_lineType;
  double m_ObjectX;
  double m_ObjectY;
  QPoint m_downPoint;

  // 修饰键
  bool shift_pressed_ = false;
  bool ctrl_pressed_ = false;

  bool mouse_left_pressed_ = false;
  bool mouse_middle_pressed_ = false;

  bool gl_initialized_ = false;

  bool has_mine_origin_ = false;
  geditor::MineOrigin mine_origin_;

  GeditorMainWindow *main_window_;
  int view_mode_ = 0;
};
