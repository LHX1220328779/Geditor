// clang-format off
#include "renderGL/gl_api.h"
// clang-format on
#include "ui/display_widget.h"

#include <QCoreApplication>
#include <QFileDialog>
#include <QMenu>
#include <QMessageBox>
#include <QStatusBar>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "core/bound_segment.h"
#include "core/lane_segment.h"
#include "core/map_feature.h"
#include "map/vdb_manage.h"
#include "ui/centerpointdialog.h"
#include "ui/geditor_mainwindow.h"
#include "ui/import_func_point_dialog.h"
#include "ui/import_gps_dialog.h"
#include "ui/pointcloudfilterdialog.h"
#include "ui/settingdialog.h"
#include "utils/color_interpolation.h"

using namespace geditor;

namespace {

void CopyCString(const std::string &src, char *dst, size_t dst_size) {
  if (dst == nullptr || dst_size == 0) {
    return;
  }
  std::memset(dst, 0, dst_size);
  if (src.empty()) {
    return;
  }
  std::strncpy(dst, src.c_str(), dst_size - 1);
}

}  // namespace

DisplayWidget::DisplayWidget(QWidget *parent) : QOpenGLWidget(parent) {
  main_window_ = dynamic_cast<GeditorMainWindow *>(parent->parent());
  menu_ = new MenuWidget(this);
}

void DisplayWidget::initializeGL() {
  GLenum err = glewInit();
  if (err != GLEW_OK) {
    LOG(ERROR) << "glew init not good: " << glewGetString(err);
    exit(-1);
  }

  m_framework.Init();
  m_MapMode = MO_MAP_EDIT;
  m_framework.Set2DView();
  if (has_mine_origin_) {
    // Point-cloud vertices are centered by each tile's altitude before
    // rendering. The configured ALT is used for geographic conversion/export,
    // while the display camera keeps its local Z origin at zero.
    m_framework.setMapCenterUTM(mine_origin_.x, mine_origin_.y, 0.0);
  }

  setMouseTracking(true);
  // this->grabKeyboard();
  setFocusPolicy(Qt::StrongFocus);
  gl_initialized_ = true;
}

void DisplayWidget::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Control:
      ctrl_pressed_ = true;
      break;
    case Qt::Key_Shift: {
      shift_pressed_ = true;
      break;
    }
    default:
      main_window_->SetMode(event->key());
      break;
  }
}

void DisplayWidget::keyReleaseEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Control:
      ctrl_pressed_ = false;
      break;
    case Qt::Key_Shift:
      shift_pressed_ = false;
      break;
    default:
      main_window_->SetMode(event->key());
      break;
  }
}

void DisplayWidget::paintGL() {
  if (gl_initialized_) {
    if (!hasFocus()) {
      shift_pressed_ = false;
      ctrl_pressed_ = false;
    }
    m_framework.Draw();
  }
}

void DisplayWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::MouseButton::LeftButton) {
    OnLButtonDown(event->pos());
  } else if (event->button() == Qt::MouseButton::RightButton) {
    OnRButtonDown(event->pos(), true);
  } else if (event->button() == Qt::MouseButton::MiddleButton) {
    OnMButtonDown(event->pos());
    mouse_middle_pressed_ = true;
  }
}
void DisplayWidget::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() == Qt::MouseButton::RightButton) {
    OnRButtonDown(event->pos(), true);
  }
}
void DisplayWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::MouseButton::LeftButton) {
    OnLButtonUp(event->pos());
  } else if (event->button() == Qt::MouseButton::MiddleButton) {
    mouse_middle_pressed_ = false;
    OnMButtonUp(event->pos());
  }
}

void DisplayWidget::wheelEvent(QWheelEvent *event) {
  if (event->delta() < 0) {
    m_framework.Zoom(0.9);
  } else {
    m_framework.Zoom(1.1);
  }
}

void DisplayWidget::OnMButtonUp(QPoint point) { this->releaseMouse(); }

void DisplayWidget::OnLButtonUp(QPoint point) {
  if (m_MapMode == MO_MOVE_OBJECT) {
    //鼠标单击
    if (m_downPoint == point) {
      double fx;
      double fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);

      m_ObjectX = fx;
      m_ObjectY = fy;

      //是否多选择模式
      // bool bmult =  (MK_SHIFT & nFlags);
      // m_framework.SelectSegment(bmult, false, fx, fy);
    } else {
      m_framework.EndMoveObject();
    }
  } else if (m_MapMode == MO_MAP_EDIT) {
    m_framework.EndMovePoint();
  }

  mouse_left_pressed_ = false;
}

void DisplayWidget::mouseMoveEvent(QMouseEvent *event) {
  auto point = event->pos();
  if (m_MapMode == MO_MAP_SCAN) {
    //平移
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    } else if (mouse_left_pressed_) {
      m_framework.MouseRotate(point.x(), point.y());
    }
  } else if (m_MapMode == MO_DRAW_MAP) {
    //绘图模式
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    } else {
      if (m_framework.IsEndGeoemtry()) {
        double fx, fy;
        m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
        m_framework.HighlightSegment(fx, fy, 1);
      } else {
        double fx, fy;
        m_framework.HighlightSegment(fx, fy, 2);
        m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
        m_framework.DrawPointToGeoemtry(fx, fy);
      }
    }

  } else if (m_MapMode == MO_MAP_EDIT) {
    //地图编辑
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    } else if (mouse_left_pressed_) {
      double fx;
      double fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);

      //在此添加判断（如果移动点或者移动涉及的点在多边形范围外，则停止编辑）
      //如果对应的点在区域范围内，则可以编辑移动
      //如果对应的点超出区域范围内，则不可以编辑移动
      // bool flag = m_framework.PointsInPolygon(fx, fy);
      m_framework.MoveSegment(fx, fy);
      m_ObjectX = fx;
      m_ObjectY = fy;
    } else {
      double fx;
      double fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
      m_framework.HighlightSegment(fx, fy, 0);
    }
  } else if (m_MapMode == MO_MOVE_OBJECT) {
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    } else if (mouse_left_pressed_) {
      double fx;
      double fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
      m_framework.MoveMapObject(fx, fy);
    } else {
      double fx;
      double fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
      m_framework.HighlightSegment(fx, fy, 0);
    }
  } else if (m_MapMode == MO_BREAK_SEGMENT) {
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    }
  } else if (m_MapMode == MO_DISTANCE) {
    //测距模式
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    } else {
      double fx, fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
      m_framework.MoveMeasureDistance(fx, fy);
      //显示距离
      double dDist = m_framework.GetMeasureDistance();
      main_window_->ShowMeasureDistance(dDist);
    }
  } else if (m_MapMode == MO_CLIPER) {
    //切分模式
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    } else {
      double fx, fy;
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
      m_framework.MoveCliperPolygon(fx, fy);
    }
  } else if (m_MapMode == MO_PICK_TRAJ || m_MapMode == MO_PICK_DIR_TRAJ) {
    //点轨迹 / 设定行驶方向模式：中键平移
    if (mouse_middle_pressed_) {
      m_framework.MouseMove(point.x(), point.y());
    }
  }

  double endx, endy;
  m_framework.MousePointToCart(point.x(), point.y(), endx, endy);

  // if (m_MapMode != MO_MOVE_OBJECT || (!(mouse_left_pressed_))) {
  //   m_framework.HighlightSegment(endx, endy, 0);
  // }

  LatLon lonlat;
  ProjectionUTM projectionUTM;
  int nzone = ProjectionUTM::zone;
  projectionUTM.CartesianToLatLon(endx, endy, nzone, false, lonlat);
  main_window_->ShowMousePosition(lonlat.lon, lonlat.lat, endx, endy);
}

void DisplayWidget::OnLButtonDown(QPoint point) {
  m_downPoint = point;
  mouse_left_pressed_ = true;
  if (m_MapMode == MO_MAP_SCAN) {
    //地图浏览
    m_framework.LButton(point.x(), point.y());
  } else if (m_MapMode == MO_DRAW_MAP) {
    //绘图模式
    double fx, fy;
    if (m_framework.HighlightPoint(fx, fy) < 0)
      m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
    bool bLink = ctrl_pressed_;
    bool bShift = shift_pressed_;
    if (m_framework.StartNewGeoemtry(m_lineType, fx, fy, bLink)) {
      // 切换为编辑模式
      if (!bShift) {
        m_MapMode = MO_MAP_EDIT;
        main_window_->SetMode(Qt::Key_Escape);
      }
    }
  } else if (m_MapMode == MO_MAP_EDIT) {
    //编辑模式
    double fx;
    double fy;
    m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
    m_ObjectX = fx;
    m_ObjectY = fy;
    if (shift_pressed_ && !ctrl_pressed_) {
      //插入点
      m_framework.InsertPoint(fx, fy);
    } else {
      //是否多选择模式
      bool bmulselect = ctrl_pressed_;
      bool bLink = shift_pressed_;
      // LOG(INFO) << bmulselect;
      m_framework.SelectSegment(bmulselect, bLink, fx, fy);
    }

    int keyPoint = -1;
    MapFeature *pFeature = NULL;
    if (m_framework.SelectTrajectoryImage(fx, fy, pFeature, keyPoint)) {
      // OnViewImage(pFeature, keyPoint);
    }

  } else if (m_MapMode == MO_MOVE_OBJECT) {
    //移动模式
    double fx;
    double fy;
    m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
    m_framework.SelectSegment(true, false, fx, fy);
  } else if (m_MapMode == MO_BREAK_SEGMENT) {
    //打断模式
    double fx;
    double fy;
    m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
    //单选择模式
    m_framework.BreakSegment(fx, fy);
  } else if (m_MapMode == MO_DISTANCE) {
    //测距模式
    double fx;
    double fy;
    m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
    m_framework.BeginMeasureDistance(fx, fy);
  } else if (m_MapMode == MO_CLIPER) {
    //切割模式
    double fx;
    double fy;
    m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
    m_framework.BeginCliperPolygon(fx, fy);
  } else if (m_MapMode == MO_PICK_TRAJ) {
    HandlePickTrajectoryClick(point);
  } else if (m_MapMode == MO_PICK_DIR_TRAJ) {
    HandlePickDirectionClick(point);
  }
}
void DisplayWidget::OnRButtonDown(QPoint point, bool dc) {
  if (m_MapMode == MO_DRAW_MAP) {
    m_framework.EndDrawGeoemtry();
    //切换为编辑模式
    if (dc) {
      m_MapMode = MO_MAP_EDIT;
      main_window_->SetMode(Qt::Key_Escape);
    }
  } else if (m_MapMode == MO_DISTANCE) {
    //测距模式
    m_framework.EndMeasureDistance();
    //切换为编辑模式
    if (dc) {
      m_MapMode = MO_MAP_EDIT;
      main_window_->SetMode(Qt::Key_Escape);
    }
  } else if (m_MapMode == MO_CLIPER) {
    //测距模式
    m_framework.EndCliperPolygon();
    //切换为编辑模式
    if (dc) {
      m_MapMode = MO_MAP_EDIT;
      main_window_->SetMode(Qt::Key_Escape);
    }
  } else if (m_MapMode == MO_MAP_EDIT) {
    if (!m_framework.GetSelected().empty() && m_framework.IsEndGeoemtry()) {
      {
        shift_pressed_ = false;
        ctrl_pressed_ = false;
      }
      menu_->Exec(QCursor::pos());
    }
  } else if (m_MapMode == MO_PICK_TRAJ) {
    CancelPickTrajectoryMode();
  } else if (m_MapMode == MO_PICK_DIR_TRAJ) {
    CancelPickDirectionMode();
  }
}

void DisplayWidget::OnMButtonDown(QPoint point) {
  this->grabMouse();
  m_framework.LButton(point.x(), point.y());
}

void DisplayWidget::resizeGL(int w, int h) {
  if (w != 0 && h != 0) {
    m_framework.Resize(0, 0, w, h);
    if (m_MapMode == MO_MAP_EDIT) {
      m_framework.Set2DView();
    }
  }
}

void DisplayWidget::OnOpenPDB(const std::string &pdb_path) {
  m_framework.SetDataSource(pdb_path.c_str());
  m_framework.ViewPanMap();
}

void DisplayWidget::OnClosePDB() { m_framework.CloseDataSource(); }

void DisplayWidget::OnDestroy() { m_framework.Destroy(); }

void DisplayWidget::OnImportFunctionPoint() {
  ImportFuncPointDialog dlg;
  if (dlg.exec() == QDialog::Accepted) {
    m_framework.AddBoundary(dlg.GetBoundary());
    m_framework.AddJobArea(dlg.GetJobArea());
  };
}

void DisplayWidget::OnImportGps() {
  ImportGPSDialog dlg;
  if (has_mine_origin_) dlg.SetMineOrigin(mine_origin_);
  if (dlg.exec() == QDialog::Accepted) {
    int import_type = dlg.GetImportType();
    auto segment = dlg.GetSegment();
    if (import_type == 0) {
      std::vector<Lane *> segArray;
      size_t iSize = segment.size();
      for (size_t i = 0; i < iSize; i++) {
        Lane *pLane = new Lane();
        GeoPolyline *pGeoLine = new GeoPolyline();
        CurbsTrack *pTrack = segment[i];
        for (size_t x = 0; x < pTrack->trackSet.size(); x++) {
          TrackPoint pnt = pTrack->trackSet[x];
          pGeoLine->AppendVertex(pnt.pnt);
        }

        pLane->polyline = pGeoLine;
        pLane->uniqueId = i + 1;
        pLane->pProperty.length = 0;
        pLane->pProperty.speed = 0;
        pLane->pProperty.turnType = 0;
        pLane->pProperty.lanePitch = 0.0;
        pLane->pProperty.mineSegmentIndex = pTrack->sourceIndex;
        CopyCString(pTrack->sourceCode, pLane->pProperty.mineSegmentCode,
                    sizeof(pLane->pProperty.mineSegmentCode));

        segArray.push_back(pLane);
      }

      m_framework.AddBoundary(segArray);
    } else if (import_type == 1) {
      std::vector<Lane *> segArray;
      size_t iSize = segment.size();
      for (size_t i = 0; i < iSize; i++) {
        Lane *pLane = new Lane();
        GeoPolyline *pGeoLine = new GeoPolyline();
        CurbsTrack *pTrack = segment[i];
        for (size_t x = 0; x < pTrack->trackSet.size(); x++) {
          TrackPoint pnt = pTrack->trackSet[x];
          pGeoLine->AppendVertex(pnt.pnt);
        }

        pLane->polyline = pGeoLine;
        pLane->uniqueId = 0;
        pLane->pProperty.length = 0;
        pLane->pProperty.speed = 0;
        pLane->pProperty.turnType = 0;
        pLane->pProperty.lanePitch = 0.0;
        pLane->pProperty.mineSegmentIndex = pTrack->sourceIndex;
        CopyCString(pTrack->sourceCode, pLane->pProperty.mineSegmentCode,
                    sizeof(pLane->pProperty.mineSegmentCode));

        segArray.push_back(pLane);
      }
      m_framework.AddSegment(segArray);
    } else if (import_type == 2) {
      std::vector<BoundSegment *> segArray;
      ConvertTrack(segment, segArray);
      m_framework.AddTrajectory(segArray);
    }
  }
}

void DisplayWidget::ConvertTrack(
    const std::vector<geditor::CurbsTrack *> &segmentArray,
    std::vector<geditor::BoundSegment *> &segArray) {
  // 切分后curbs，保留原始 sourceCode / sourceIndex
  struct SplitTrack {
    std::vector<TrackPoint> trackSet;
    std::string sourceCode;
    int sourceIndex = 0;
  };
  std::vector<SplitTrack> splitCurbs;

  for (size_t i = 0; i < segmentArray.size(); i++) {
    CurbsTrack *pTrack = segmentArray[i];

    std::vector<TrackPoint> trackSet;
    double ndtStart = pTrack->trackSet[0].ndt;
    for (size_t x = 0; x < pTrack->trackSet.size(); x++) {
      TrackPoint pnt = pTrack->trackSet[x];

      if (abs(ndtStart - pnt.ndt) > 0.02) {
        SplitTrack st;
        st.trackSet = trackSet;
        st.sourceCode = pTrack->sourceCode;
        st.sourceIndex = pTrack->sourceIndex;
        splitCurbs.push_back(std::move(st));

        trackSet.clear();
        ndtStart = pnt.ndt;
      }
      trackSet.push_back(pnt);
    }

    if (trackSet.size() > 1) {
      SplitTrack st;
      st.trackSet = trackSet;
      st.sourceCode = pTrack->sourceCode;
      st.sourceIndex = pTrack->sourceIndex;
      splitCurbs.push_back(std::move(st));
    }

    delete pTrack;
  }

  //=========================================
  int uniqueId = 1;

  for (size_t i = 0; i < splitCurbs.size(); i++) {
    SplitTrack &split = splitCurbs[i];
    std::vector<TrackPoint> &pTrack = split.trackSet;

    if (pTrack.size() > 1) {
      double ndtStart = pTrack[0].ndt;

      BoundSegment *pLaneSegment = new BoundSegment();

      GeoPolyline *pLine = new GeoPolyline();
      int line_num = -1;
      for (size_t m = 0; m < pTrack.size(); m++) {
        pLine->AppendVertex(pTrack[m].pnt);
        pLaneSegment->AddImageIndex(m, pTrack[m].img);
        line_num = pTrack[m].idx;
      }
      pLaneSegment->SetGeometry(pLine);
      pLaneSegment->SetUniqueID(uniqueId++);

      BoundaryProperty pProperty;
      pProperty.length = 0.0;
      pProperty.ndtParam = floor(ndtStart * 10.0 + 0.5) / 10.0;
      pProperty.lanePitch = line_num;
      // 保存矿山路段索引和编号，供自动绑定使用
      pProperty.mineSegmentIndex =
          split.sourceIndex > 0 ? split.sourceIndex : (int)line_num;
      std::strncpy(pProperty.mineSegmentCode, split.sourceCode.c_str(),
                   sizeof(pProperty.mineSegmentCode) - 1);
      pLaneSegment->SetProperty(&pProperty);

      V4f vClr = Hsv2Rgb((ndtStart - 2) * 130, 1, 255);
      PolyLineSytle *pStyle = pLaneSegment->GetStyle();
      pStyle->SetLineColor(vClr);
      segArray.push_back(pLaneSegment);
    }
  }
}

void DisplayWidget::OnLoadTrack() {
  QFileDialog fd(this, "Select file", "", "DB文件(*.db);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];

    if (m_framework.LoadDBMap(sel.toUtf8().constData())) {
      std::vector<CurbsTrack *> segmentArray;
      if (m_framework.ReadCrubsTrack(segmentArray)) {
        std::vector<BoundSegment *> segArray;
        ConvertTrack(segmentArray, segArray);
        m_framework.AddTrajectory(segArray);
      }
    } else {
      QMessageBox::warning(this, "警告", "无法加载轨迹");
    }
  }
}

void DisplayWidget::OnOpenVdb() {
  QFileDialog fd(this, "Select file", "", "矢量地图(*.vdb);;");
  if (fd.exec() == QDialog::Accepted) {
    QString sel = fd.selectedFiles()[0];
    if (!m_framework.Read(sel.toUtf8().constData())) {
      QMessageBox::warning(this, "提示", "读取VDB数据库失败");
    } else {
      m_framework.ViewPanMap();
      m_framework.GenerateTopology();
    }
  }
}

void DisplayWidget::OnDataSave() {
  // if (m_framework.IsOpen()) {
  //   m_framework.Save();
  // } else
  {
    QString sel = QFileDialog::getSaveFileName(this, "save file", "",
                                               "矢量地图(*.vdb);;");
    if (!sel.isEmpty()) {
      if (!sel.endsWith(".vdb")) sel += ".vdb";
      if (m_framework.Create(sel.toUtf8().constData())) {
        m_framework.Save();
        QMessageBox::information(this, "提示", "保存VDB成功！");
        std::string path = QCoreApplication::applicationDirPath().toStdString();
        QFileInfo fi(sel);
        if (fi.exists()) {
          if (!has_mine_origin_) {
            QMessageBox::warning(
                this, "提示",
                "VDB已保存；尚未选择矿山原点，因此跳过地图导出。请在工具->矿山原点配置中选择后重试。");
          } else {
            std::string dir = fi.absolutePath().toStdString();
            std::ostringstream cmd;
            cmd << std::setprecision(15) << path << "/vdb2pb "
                << sel.toStdString() << " " << dir << " 1.0 "
                << mine_origin_.longitude << " " << mine_origin_.latitude
                << " " << mine_origin_.z << " " << mine_origin_.zone;
            const int export_result = system(cmd.str().c_str());
            if (export_result != 0) {
              QMessageBox::warning(this, "警告", "地图导出失败，请检查日志");
            }
          }
        }

      } else {
        QMessageBox::warning(this, "警告", "创建VDB失败！");
      }
    }
  }
}

void DisplayWidget::OnDataClose() {
  if (QMessageBox::question(this, "提示", "数据已修改，是否保存？") ==
      QMessageBox::Yes) {
    OnDataSave();
    m_framework.Close();
  }
}

void DisplayWidget::OnPanMap() { m_framework.ViewPanMap(); }

void DisplayWidget::On3dView() {
  m_framework.Set3DView();
  m_MapMode = MO_MAP_SCAN;
  view_mode_ = 1;
}

void DisplayWidget::On2dView() {
  m_framework.Set2DView();
  m_MapMode = MO_MAP_SCAN;
  view_mode_ = 0;
}

void DisplayWidget::OnConvertCoord() {
  CenterPointDialog dlg;
  if (dlg.exec() == QDialog::Accepted) {
    m_framework.setMapCenter(dlg.m_lat, dlg.m_lon);
    // ProjectionUTM::SetZoneByLon(dlg.m_lon);
  }
}

void DisplayWidget::ApplyMineOrigin(const geditor::MineOrigin &origin) {
  if (origin.zone < 1 || origin.zone > 60) return;
  mine_origin_ = origin;
  has_mine_origin_ = true;
  ProjectionUTM::zone = origin.zone;
  if (gl_initialized_) {
    m_framework.setMapCenterUTM(origin.x, origin.y, 0.0);
  }
  update();
}

void DisplayWidget::OnUndoOperate() { m_framework.UndoOperate(); }

void DisplayWidget::OnRedoOperate() { m_framework.RedoOperate(); }

void DisplayWidget::OnMapZoomIn() { m_framework.Zoom(1.1f); }

void DisplayWidget::OnMapZoomOut() { m_framework.Zoom(0.9f); }

void DisplayWidget::OnEditObject() {
  m_framework.EndDrawGeoemtry();
  m_framework.EndMeasureDistance();
  m_MapMode = MO_MAP_EDIT;
}

void DisplayWidget::OnMoveObject() {
  m_framework.EndDrawGeoemtry();
  m_framework.EndMeasureDistance();
  m_MapMode = MO_MOVE_OBJECT;
}

void DisplayWidget::OnDeleteObject() {
  m_framework.EndDrawGeoemtry();
  m_framework.EndMeasureDistance();
  m_framework.DeleteMapFeature();
}

void DisplayWidget::OnMeasureDistance() {
  m_framework.EndDrawGeoemtry();
  m_framework.EndMeasureDistance();
  m_MapMode = MO_DISTANCE;
}

void DisplayWidget::OnCliperPolygon() { m_MapMode = MO_CLIPER; }

void DisplayWidget::OnDrawLine() {
  m_MapMode = MO_DRAW_MAP;
  m_lineType = DrawType::Boundary();
  m_framework.ActiveLayer(LT_BOUNDARY);
}
void DisplayWidget::OnDrawLane() {
  m_MapMode = MO_DRAW_MAP;
  m_lineType = DrawType::Lane();
  m_framework.ActiveLayer(LT_LANE);
}

void DisplayWidget::OnPointFilter() {
  PointCloudFilterDialog *dlg = new PointCloudFilterDialog(this);
  dlg->Show();
}
///////////////////////////del
void DisplayWidget::OnActiveBoundary() {}

void DisplayWidget::OnActiveJob() {}

void DisplayWidget::OnActiveTopology() {}

void DisplayWidget::OnActiveRegion() {}

void DisplayWidget::OnActiveSegment() {}

void DisplayWidget::OnActiveFunPoint() {}

void DisplayWidget::OnShowhideTrack(bool show) {
  m_framework.ShowHideTrackLayer(show);
}

void DisplayWidget::OnShowhideBoundary(bool show) {
  m_framework.ShowHideBoundaryLayer(show);
}

void DisplayWidget::OnShowhideJobmap(bool show) {
  m_framework.ShowHideJobLayer(show);
  m_framework.ShowHideAreaLayer(show);
}

void DisplayWidget::OnShowhideRoad(bool show) {
  m_framework.ShowHideSegmentlayerLayer(show);
}

void DisplayWidget::OnShowhidePDB(bool show) {
  m_framework.ShowHidePDBLayer(show);
}

void DisplayWidget::OnShowhideSign(bool show) {
  m_framework.ShowHideSignLayer(show);
}

void DisplayWidget::OnDrawBezierCurve() {
  LOG(INFO) << "Bezier Curve Not implemented yet";
  return;
  // MessageBox(_T("功能未实现^ ^!"), _T("信息提示"), MB_OK |
  // MB_ICONINFORMATION);

  // if (m_framework.GetActiveLayer() == 1 || m_framework.GetActiveLayer() == 4)
  // {
  //   m_MapMode = MO_DRAW_MAP;
  //   m_lineType = 5;
  // } else {
  //   // ShowMessage(_T("请选择需要编辑的图层。"));
  // }
}

void DisplayWidget::OnDrawBsplineCurve() {
  LOG(INFO) << " Curve Not implemented yet";
  return;
  // if (m_framework.GetActiveLayer() == 1 || m_framework.GetActiveLayer() == 4)
  // {
  //   m_MapMode = MO_DRAW_MAP;
  //   m_lineType = 6;
  // } else {
  //   // ShowMessage(_T("请选择需要编辑的图层。"));
  // }
}

void DisplayWidget::OnDrawHermiteCurve() {
  LOG(INFO) << "Curve Not implemented yet";
  return;
  // if (m_framework.GetActiveLayer() == 1 || m_framework.GetActiveLayer() == 4)
  // {
  //   m_MapMode = MO_DRAW_MAP;
  //   m_lineType = 7;
  // } else {
  //   // ShowMessage(_T("请选择需要编辑的图层。"));
  // }
}

void DisplayWidget::OnEditSegment() { m_MapMode = MO_BREAK_SEGMENT; }

void DisplayWidget::OnReverseSegment() { m_framework.ReverseSegment(); }

void DisplayWidget::OnAreaRoad() {
  m_MapMode = MO_DRAW_MAP;
  m_lineType = DrawType::RoadArea();
  m_framework.ActiveLayer(LT_ROADAREA);
}

void DisplayWidget::OnAreaJob() {
  m_MapMode = MO_DRAW_MAP;
  m_lineType = DrawType::FunArea();
  m_framework.ActiveLayer(LT_FUNAREA);
}

void DisplayWidget::OnPointJob() {
  m_MapMode = MO_DRAW_MAP;
  m_lineType = DrawType::FunArea(100);
  m_framework.ActiveLayer(LT_FUNAREA);
}

void DisplayWidget::OnPathPoint() {
  // m_MapMode = MO_DRAW_MAP;
  // m_lineType = 2;
  // m_framework.ActiveLayer(6);
}

void DisplayWidget::OnTrafficSign(int type) {
  m_MapMode = MO_DRAW_MAP;
  m_lineType = DrawType::Trafficsign();
  m_framework.ActiveLayer(LT_SIGN);
}

void DisplayWidget::OnSettingDlg() {
  SettingDialog dlg;
  dlg.m_gridSize = m_framework.GetGridSize() * 100;
  dlg.m_lineWidth = m_framework.GetLineWidth() * 100;
  dlg.m_pointSize = m_framework.GetPointSize() * 100;

  if (dlg.exec() == QDialog::Accepted) {
    double gridSize = dlg.m_gridSize / 100.0;
    m_framework.SetGridSize(gridSize, gridSize);
    double lineWidth = dlg.m_lineWidth / 100.0;
    m_framework.SetLineWidth(lineWidth);

    double pointSize = dlg.m_pointSize / 100.0;
    m_framework.SetPointSize(pointSize);
  }
}

std::vector<MapFeature *> DisplayWidget::GetSelected() {
  return m_framework.GetSelected();
}

void DisplayWidget::GenerateTopology() { m_framework.GenerateTopology(); }

void DisplayWidget::OnCheckRelation() { m_framework.CheckRelation(); }

int DisplayWidget::OnAutoBindTrajectory(
    double distTol, double coverageThreshold, bool dryRun,
    geditor::Framework::AutoBindDiagnostic *diag) {
  return m_framework.AutoBindTrajectory(distTol, coverageThreshold, dryRun,
                                        diag);
}
void DisplayWidget::ChangeLineType(int type) {
  m_framework.ChangeLineType(type);
}

void DisplayWidget::StartPickTrajectoryMode() {
  m_MapMode = MO_PICK_TRAJ;
  pick_traj_first_seg_ = nullptr;
  pick_traj_first_idx_ = -1;
  if (main_window_) {
    main_window_->statusBar()->showMessage(
        "生成中心线: 请点击第一条轨迹的【起点】位置 (右键/ESC取消)",
        0);
  }
  setCursor(Qt::CrossCursor);
}

void DisplayWidget::CancelPickTrajectoryMode() {
  if (m_MapMode == MO_PICK_TRAJ) {
    m_MapMode = MO_MAP_EDIT;
    pick_traj_first_seg_ = nullptr;
    pick_traj_first_idx_ = -1;
    setCursor(Qt::ArrowCursor);
    if (main_window_) {
      main_window_->statusBar()->showMessage("已退出中心线生成模式", 2000);
    }
  }
}

void DisplayWidget::StartPickDirectionMode(int direction) {
  if (direction != 1 && direction != 2) return;
  m_MapMode = MO_PICK_DIR_TRAJ;
  pick_dir_value_ = direction;
  pick_dir_first_seg_ = nullptr;
  pick_dir_first_idx_ = -1;
  if (main_window_) {
    main_window_->statusBar()->showMessage(
        QString("%1 模式：请点击起点轨迹节点 (右键/ESC取消)")
            .arg(direction == 1 ? "上山" : "下山"),
        0);
  }
  setCursor(Qt::CrossCursor);
}

void DisplayWidget::CancelPickDirectionMode() {
  if (m_MapMode == MO_PICK_DIR_TRAJ) {
    m_MapMode = MO_MAP_EDIT;
    pick_dir_value_ = 0;
    pick_dir_first_seg_ = nullptr;
    pick_dir_first_idx_ = -1;
    setCursor(Qt::ArrowCursor);
    if (main_window_) {
      main_window_->statusBar()->showMessage("已退出设定行驶方向模式", 2000);
    }
  }
}

void DisplayWidget::HandlePickDirectionClick(QPoint point) {
  double fx = 0, fy = 0;
  m_framework.MousePointToCart(point.x(), point.y(), fx, fy);
  const double kTol = 10.0;
  geditor::BoundSegment *seg = nullptr;
  int idx = -1;
  if (!m_framework.PickTrajectoryAnchor(fx, fy, kTol, seg, idx)) {
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          "未在附近找到轨迹节点，请靠近轨迹后再点 (容差10m)", 3000);
    }
    return;
  }
  if (!pick_dir_first_seg_) {
    pick_dir_first_seg_ = seg;
    pick_dir_first_idx_ = idx;
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          QString("起点已锁定 (轨迹%1 节点#%2)。请点击终点轨迹节点")
              .arg(seg->GetUniqueID())
              .arg(idx),
          0);
    }
    return;
  }
  geditor::Framework::DirectionChainDiagnostic diag;
  int n = m_framework.GenerateCenterlineChainByDirection(
      pick_dir_first_seg_, pick_dir_first_idx_, seg, idx, pick_dir_value_,
      &diag);
  if (n <= 0) {
    QMessageBox::warning(
        this, "设定行驶方向",
        QString("生成失败：未找到起止轨迹之间的连通路径。\n%1")
            .arg(QString::fromStdString(diag.detail)));
  } else {
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          QString("已生成 %1 条%2中心线 (轨迹链 %3 段)")
              .arg(n)
              .arg(pick_dir_value_ == 1 ? "上山" : "下山")
              .arg(diag.chainSegments),
          5000);
    }
  }
  pick_dir_first_seg_ = nullptr;
  pick_dir_first_idx_ = -1;
  update();
}

void DisplayWidget::HandlePickTrajectoryClick(QPoint point) {
  double fx = 0, fy = 0;
  m_framework.MousePointToCart(point.x(), point.y(), fx, fy);

  // 拾取容差：10 米 (矿区轨迹节点稀疏，宽松一点)
  const double kTol = 10.0;
  geditor::BoundSegment *seg = nullptr;
  int idx = -1;
  if (!m_framework.PickTrajectoryAnchor(fx, fy, kTol, seg, idx)) {
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          "未在附近找到轨迹节点，请靠近轨迹后再点 (容差10m)", 3000);
    }
    return;
  }

  if (!pick_traj_first_seg_) {
    pick_traj_first_seg_ = seg;
    pick_traj_first_idx_ = idx;
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          QString("起点已锁定 (轨迹id=%1 节点#%2)。请点击同一条轨迹的【终点】")
              .arg(seg->GetUniqueID())
              .arg(idx),
          0);
    }
    return;
  }

  if (seg != pick_traj_first_seg_) {
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          "终点不在同一条轨迹上，请重新点击终点 (起点仍保持)", 3000);
    }
    return;
  }

  geditor::LaneSegment *lane = m_framework.GenerateCenterlineFromAnchors(
      pick_traj_first_seg_, pick_traj_first_idx_, idx);
  if (!lane) {
    QMessageBox::warning(this, "生成中心线",
                         "生成失败：起终点索引相同或轨迹为空。");
  } else {
    if (main_window_) {
      main_window_->statusBar()->showMessage(
          QString("已生成中心线 id=%1, 节点数=%2")
              .arg(lane->GetUniqueID())
              .arg(lane->GetGeometry()->GetVertexCount()),
          5000);
    }
  }

  pick_traj_first_seg_ = nullptr;
  pick_traj_first_idx_ = -1;
  update();
}

int DisplayWidget::OnAutoCheckRoadRight(
    double threshold, geditor::Framework::RoadRightDiagnostic *diag) {
  return m_framework.AutoCheckRoadRight(threshold, diag);
}

int DisplayWidget::OnCheckLaneConnectivity(
    geditor::Framework::ConnectivityDiagnostic *diag) {
  return m_framework.CheckLaneConnectivity(diag);
}

bool DisplayWidget::OnExportRoadRight(
    const std::string &path, geditor::Framework::RoadRightIOStat *stat) {
  return m_framework.ExportRoadRightTxt(path, stat);
}

bool DisplayWidget::OnImportRoadRight(
    const std::string &path, geditor::Framework::RoadRightIOStat *stat) {
  return m_framework.ImportRoadRightTxt(path, stat);
}
