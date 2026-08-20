
#include "core/framework.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "algorithm/common.h"
#include "algorithm/matrix44.h"
#include "algorithm/ray3.h"
#include "core/bound_segment.h"
#include "core/boundary_layer.h"
#include "core/command.h"
#include "core/factory_drawable.h"
#include "core/geo_arc_line.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_rectangle.h"
#include "core/job_area.h"
#include "core/job_layer.h"
#include "core/lane_segment_layer.h"
#include "core/line_drawable.h"
#include "core/pdb_layer.h"
#include "core/point_drawable.h"
#include "core/road_area.h"
#include "core/road_area_layer.h"
#include "core/sign_board_layer.h"
#include "core/topology_layer.h"
#include "core/trajectory_layer.h"
#include "map/pdb_manage.h"
#include "map/projection_utm.h"
#include "map/vdb_manage.h"
#include "pcd/db_read_write.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_technique_manager.h"
#include "sprinkle/sprinkle.h"

// #include "Utils/String.h"
#include "coverage/toolbox.h"

using namespace geditor::sprinkle;

namespace geditor
{

  Framework::Framework()
      : m_pdbLayer(NULL),
        m_areaLayer(NULL),
        m_Segmentlayer(NULL),
        m_pHighFeature(NULL),
        m_selectFeature(NULL),
        m_keyPoint(-1),
        m_vdbmgr(NULL),
        m_pCamera(NULL),
        m_layer(NULL),
        m_boundlayer(NULL),
        m_joblayer(NULL),
        m_pDrawFeature(NULL)
  {
    geoline_type_ = Geometry::GeometryType::GT_POLYLINE;
  }

  Framework::~Framework()
  {
    if (m_vdbmgr != NULL)
    {
      delete m_vdbmgr;
      m_vdbmgr = NULL;
    }
  }

  void Framework::Init()
  {
    State *pState = new State();
    pState->initializeProcs();

    m_rendinfo.SetState(pState);

    m_pdbLayer = new PDBLayer();
    m_areaLayer = new AreaLayer();
    m_Segmentlayer = new SegmentLayer();
    // m_Segmentlayer->SetCarBodyCheck(true);
    m_pointLayer = new PointLayer();
    m_boundlayer = new BoundaryLayer();
    m_joblayer = new JobLayer();
    m_topologylayer = new TopologyLayer();
    m_trajectorylayer = new TrajectoryLayer();
    LayerHelper layer_helper;
    layer_helper.SetLayer(m_pdbLayer, LT_PDB);
    layer_helper.SetLayer(m_areaLayer, LT_ROADAREA);
    layer_helper.SetLayer(m_Segmentlayer, LT_LANE);
    layer_helper.SetLayer(m_pointLayer, LT_SIGN);
    layer_helper.SetLayer(m_boundlayer, LT_BOUNDARY);
    layer_helper.SetLayer(m_joblayer, LT_FUNAREA);
    layer_helper.SetLayer(m_topologylayer, LT_TOPO);
    layer_helper.SetLayer(m_trajectorylayer, LT_TRA);

    m_Segmentlayer->SetLayers(layer_helper);
    for (auto &layer : layer_helper.layers())
      if (layer)
        layers_.push_back(layer);

    // 激活图层
    m_layertype = LT_BOUNDARY;
    m_layer = m_Segmentlayer;

    // m_fScale = 0.125;
    // m_scaleMatrix = Matrix4x4f::MakeScale(m_fScale, m_fScale, m_fScale);
    m_fMapScale = 0.01;

    TechniqueManager::GetInstance()->Initialize();
  }

  void Framework::Zoom(float fzoom)
  {
    // m_fScale = m_fScale * fzoom;
    // m_scaleMatrix = Matrix4x4f::MakeScale(m_fScale, m_fScale, m_fScale);
    m_fMapScale = m_fMapScale * fzoom;
  }

  void Framework::GeneratePathPoint()
  {
    std::vector<MapFeature *> objects;

    m_areaLayer->GetAllMapFeature(objects);
    m_joblayer->GetAllMapFeature(objects);

    //---------------------------------------------

    std::vector<Polygon> input;

    for (int i = 0; i < objects.size(); i++)
    {
      Geometry *pGeo = objects[i]->GetGeometry();
      if (pGeo != NULL && pGeo->GetGeometryType() == Geometry::GT_POLYGON)
      {
        Polygon polygon;

        polygon.PolyginSize = pGeo->GetVertexCount();
        for (int i = 0; i < polygon.PolyginSize; i++)
        {
          const Point3d &pnt = pGeo->GetVertex(i);

          polygon.Vertex.push_back(Point2D(pnt.x, pnt.y));
        }

        input.push_back(polygon);
      }
    }
    //----------------------------------------------

    SprinkleConf sprinkle_conf;
    sprinkle_conf.Proportion = 20;
    sprinkle_conf.PointCounts = 1;
    sprinkle_conf.CellSize = 3;
    sprinkle_conf.Range = 2000;
    sprinkle_conf.DisBoundary = 2;
    sprinkle_conf.Distance = 0.6;
    sprinkle_conf.Fproportion = 20.0;
    sprinkle_conf.CompleteStatus = false;
    sprinkle_conf.ErrorCode = 0;
    sprinkle_conf.MaxDistance = 5.9;
    sprinkle_conf.MinDistance = 1.8;

    std::vector<Point2D> vector_points;
    std::vector<std::pair<int, int>> Tp_relationship;

    Sprinkle test(input, sprinkle_conf, vector_points, Tp_relationship);
    for (std::vector<Point2D>::iterator it = vector_points.begin();
         it != vector_points.end();)
    {
      bool bNotFind = true;

      for (int i = 0; i < Tp_relationship.size(); i++)
      {
        if (it->Label == Tp_relationship[i].first ||
            it->Label == Tp_relationship[i].second)
        {
          bNotFind = false;
          break;
        }
      }

      if (bNotFind)
      {
        it = vector_points.erase(it);
      }
      else
      {
        it++;
      }
    }

    for (int i = 0; i < Tp_relationship.size(); i++)
    {
      m_topologylayer->AddTopology(Tp_relationship[i].first + 1,
                                   Tp_relationship[i].second + 1);
    }

    for (int i = 0; i < vector_points.size(); i++)
    {
      GeoPolygon *pGeometry = new GeoPolygon();
      pGeometry->AppendVertex(
          Point3d(vector_points[i].x, vector_points[i].y, 0.0));

      PointElement *pElement = new PointElement();

      pElement->SetUniqueID(vector_points[i].Label + 1);
      pElement->SetGeometry(pGeometry);

      m_topologylayer->AddMapFeature(pElement);
    }
  }

  // 通过屏幕上鼠标垫反算出地图坐标
  void Framework::MousePointToCart(int x, int y, double &dx, double &dy)
  {
    V3d postion = m_pCamera->GetPostion();

    V3f vec3;
    vec3[0] = x * 2.0f / m_viewport.Width() - 1.0f;
    vec3[1] = 1.0f - y * 2.0f / m_viewport.Height();

    Matrix4x4f invViewProj = m_pCamera->GetViewProjMatrix().Inverse();

    V3f temp1 = invViewProj.TransformCoord(vec3);
    temp1.Multiply(1.0 / m_fMapScale);

    // Matrix4x4f  invscaleMatrix = Matrix4x4f::MakeScale(invMapScale,
    // invMapScale, invMapScale); invscaleMatrix.TransformCoord(temp1);

    dx = temp1[0] + postion[0];
    dy = temp1[1] + postion[1];
  }

  void Framework::ViewPanMap()
  {
    const double dMax = (std::numeric_limits<double>::max)();
    V3d vCnt;
    V3d minVec(dMax, dMax, 10000);
    V3d maxVec(-dMax, -dMax, -10000);

    if (m_pdbLayer->GetBoundary(minVec, maxVec))
    {
      vCnt = (minVec + maxVec) * 0.5;
    }
    else if (m_Segmentlayer->GetBoundary(minVec, maxVec))
    {
      vCnt = (minVec + maxVec) * 0.5;
    }
    else if (m_areaLayer->GetBoundary(minVec, maxVec))
    {
      vCnt = (minVec + maxVec) * 0.5;
    }
    else if (m_boundlayer->GetBoundary(minVec, maxVec))
    {
      vCnt = (minVec + maxVec) * 0.5;
    }
    else if (m_joblayer->GetBoundary(minVec, maxVec))
    {
      vCnt = (minVec + maxVec) * 0.5;
    }
    else if (m_trajectorylayer->GetBoundary(minVec, maxVec))
    {
      vCnt = (minVec + maxVec) * 0.5;
    }
    else
    {
      return;
    }
    m_pCamera->SetPostion(V3d(vCnt[0], vCnt[1], 0.0));
    // m_pCamera->SetPostion(V3d(374023.33, 6848220.11, 0.0));
  }

  void Framework::Pan(double dx, double dy)
  {
    V3d dir(dx, dy, 0.0);
    m_PlanCamera.MoveCamera(dir, dir.norm());
  }

  void Framework::setMapCenter(double lat, double lon)
  {
    UTMPoint utm;
    ProjectionUTM projectionUTM;
    projectionUTM.LatLonToCartesian(lat, lon, utm);

    m_pCamera->SetPostion(V3d(utm.x, utm.y, 0.0));
  }

  void Framework::setMapCenterUTM(double x, double y, double z)
  {
    if (m_pCamera)
      m_pCamera->SetPostion(V3d(x, y, z));
  }

  void Framework::Set2DView()
  {
    if (m_pCamera)
    {
      m_PlanCamera.SetPostion(m_pCamera->GetPostion());
    }
    else
    {
      m_PlanCamera.SetPostion(V3d(0.0, 0.0, 0.0));
    }
    m_PlanCamera.SetOrtho(m_viewport.AspectRatio(), 0.5f, 500.0f);
    if (m_pCamera == &m_StereoCamera)
      m_fMapScale *= 2.414;
    m_pCamera = &m_PlanCamera;
  }

  void Framework::Set3DView()
  {
    m_trackball.SetTransformation(V3f(0.0f, 0.0f, 1.0f), V3f(0.0f, 0.0f, 0.0f),
                                  V3f(0.0f, 1.0f, 0.0f));

    m_StereoCamera.SetViewMatrix(m_trackball.GetInverseMatrix());

    m_StereoCamera.SetPostion(m_pCamera->GetPostion());
    m_StereoCamera.SetPerspective(45.0f, m_viewport.AspectRatio(), 0.5f, 500.0f);
    if (m_pCamera == &m_PlanCamera)
      m_fMapScale /= 2.414;
    m_pCamera = &m_StereoCamera;
  }

  void Framework::LButton(float x, float y)
  {
    if (m_pCamera == &m_PlanCamera)
    {
      m_StartMouse[0] = x;
      m_StartMouse[1] = y;
    }
    else
    {
      m_trackball.LButtonDown(x, y);
      m_StartMouse[0] = x;
      m_StartMouse[1] = y;
    }
  }

  void Framework::MouseRotate(float x, float y)
  {
    if (m_pCamera != &m_PlanCamera)
    {
      m_trackball.MouseRotate(x, y);
    }
  }

  void Framework::MouseMove(float x, float y)
  {
    if (m_pCamera == &m_PlanCamera)
    {
      V3f vec1, vec2;

      vec1[0] = x * 2.0f / m_viewport.Width() - 1.0f;
      vec1[1] = 1.0f - y * 2.0f / m_viewport.Height();
      vec2[0] = m_StartMouse[0] * 2.0f / m_viewport.Width() - 1.0f;
      vec2[1] = 1.0f - m_StartMouse[1] * 2.0f / m_viewport.Height();

      Matrix4x4f invProj = m_pCamera->GetViewProjMatrix().Inverse();
      V3f vec = invProj.TransformNormal(vec1 - vec2);
      vec.Multiply(1.0 / m_fMapScale);

      V3d dir(vec[0], vec[1], vec[2]);
      m_pCamera->MoveCamera(dir, -dir.norm());

      //----------------------------------------
      m_StartMouse[0] = x;
      m_StartMouse[1] = y;
    }
    else
    {
      V3f vec1, vec2;

      vec1[0] = x * 2.0f / m_viewport.Width() - 1.0f;
      vec1[1] = 1.0f - y * 2.0f / m_viewport.Height();
      vec2[0] = m_StartMouse[0] * 2.0f / m_viewport.Width() - 1.0f;
      vec2[1] = 1.0f - m_StartMouse[1] * 2.0f / m_viewport.Height();

      Matrix4x4f invProj = m_pCamera->GetViewProjMatrix().Inverse();
      V3f vec = invProj.TransformNormal(vec1 - vec2);
      vec.Multiply(1.0 / m_fMapScale);

      V3d dir(vec[0], vec[1], vec[2]);
      m_pCamera->MoveCamera(dir, -dir.norm());

      m_StartMouse[0] = x;
      m_StartMouse[1] = y;
    }
  }

  void Framework::KeyMove(float x, float y)
  {
    if (m_pCamera == &m_PlanCamera)
    {
      float cntX = m_viewport.CenterX();
      float cntY = m_viewport.CenterY();

      V3d endPostion, startPostion;
      MousePointToCart(cntX + x, cntY + y, endPostion[0], endPostion[1]);
      MousePointToCart(cntX, cntY, startPostion[0], startPostion[1]);

      V3d dir = startPostion - endPostion;

      m_PlanCamera.MoveCamera(dir, dir.norm());
    }
  }

  Geometry *polyline = NULL;
  int nValildCount = 0;

  Geometry *CreateGeoLine(int type)
  {
    Geometry *polyline = NULL;
    using GT = Geometry::GeometryType;
    if (type == GT::GT_POLYLINE)
    {
      polyline = new GeoPolyline();
    }
    else if (type == GT::GT_CIRCULAR_ARC)
    {
      polyline = new GeoCircularArc();
    }
    else if (type == GT::GT_BEZIER_CURVE)
    {
      polyline = new GeoBezierCurve3();
    }
    else if (type == GT::GT_BSPLINE_CURVE)
    {
      polyline = new GeoBSplineCurve3();
    }
    else if (type == GT::GT_HERMITE_CURVE)
    {
      // polyline = new GeoHermiteCurve3();
    }
    else if (type == GT::GT_ARC_LINE)
    {
      polyline = new GeoArcLine();
    }

    return polyline;
  }

  bool Framework::StartNewGeoemtry(DrawType featureType, double fx, double fy,
                                   bool llink)
  {
    // m_layertype = 4;

    // 清空之前选中的
    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      m_selectedSet[i]->SetSelectedState(false);
    }
    m_selectedSet.clear();
    m_selectFeature = NULL;

    if (m_layertype == LT_LANE)
    {
      return StartNewLine(featureType, fx, fy, llink);
    }
    else if (m_layertype == LT_ROADAREA)
    {
      StartNewArea(featureType, fx, fy, llink);
      return false;
    }
    else if (m_layertype == LT_SIGN)
    {
      StartNewPoint(featureType, fx, fy);
      return true;
    }
    else if (m_layertype == LT_BOUNDARY)
    {
      return StartNewBoundary(featureType, fx, fy, llink);
    }
    else if (m_layertype == LT_FUNAREA)
    {
      return StartJobArea(featureType, fx, fy, llink);
    }
    else if (m_layertype == LT_TOPO)
    {
      MapFeature *preDrawFeature = m_pDrawFeature;
      StartNewElement(featureType, fx, fy);
      if (preDrawFeature && llink)
      {
        m_topologylayer->AddTopology(preDrawFeature->GetUniqueID(),
                                     m_pDrawFeature->GetUniqueID());
      }

      return true;
    }

    return false;
  }

  void Framework::StartNewElement(DrawType type, double fx, double fy)
  {
    if (m_pCamera != &m_PlanCamera)
    {
      return;
    }

    if (polyline == NULL)
    {
      polyline = new GeoPolygon();

      PointElement *pSignBoard = new PointElement();
      pSignBoard->SetGeometry(polyline);

      m_pDrawFeature = pSignBoard;
      IMapCommand *pCommand = new DrawCommand(m_layer, pSignBoard);
      pCommand->Execute();
      m_commands.push(pCommand);
      polyline->AppendVertex(fx, fy, 0.0f);
      polyline = NULL;
    }
  }

  void Framework::StartNewPoint(DrawType type, double fx, double fy)
  {
    if (m_pCamera != &m_PlanCamera)
    {
      return;
    }

    if (polyline == NULL)
    {
      polyline = new GeoPolygon();

      SignBoard *pSignBoard = new SignBoard();
      pSignBoard->SetGeometry(polyline);
      pSignBoard->SetSignboardType(type.subtype);

      m_pDrawFeature = pSignBoard;

      IMapCommand *pCommand = new DrawCommand(m_layer, pSignBoard);
      pCommand->Execute();
      m_commands.push(pCommand);

      polyline->AppendVertex(fx, fy, 0.0f);
      polyline = NULL;
    }
  }

  void Framework::StartNewArea(DrawType type, double fx, double fy, bool llink)
  {
    if (m_pCamera != &m_PlanCamera)
    {
      return;
    }
    if (polyline == NULL)
    {
      int area_type = type.subtype;

      polyline = new GeoPolygon();

      RoadArea *roadArea = new RoadArea();
      roadArea->SetGeometry(polyline);
      roadArea->SetAreaType(area_type);

      m_pDrawFeature = roadArea;

      IMapCommand *pCommand = new DrawCommand(m_layer, roadArea);
      pCommand->Execute();
      m_commands.push(pCommand);

      polyline->AppendVertex(fx, fy, 0.0f);
      nValildCount = polyline->GetVertexCount();
    }
    else
    {
      int nSize = polyline->GetVertexCount();
      if (nValildCount == nSize)
      {
        polyline->AppendVertex(fx, fy, 0.0f);
      }
      else
      {
        polyline->MoveVertex(Point3d(fx, fy, 0.0f), nSize - 1);
      }
      nValildCount = polyline->GetVertexCount();
    }
  }

  bool Framework::StartJobArea(DrawType type, double fx, double fy, bool llink)
  {
    if (m_pCamera != &m_PlanCamera)
    {
      return false;
    }
    if (polyline == NULL)
    {
      int area_type = type.subtype;
      if (area_type < TV::jobpoint)
      {
        polyline = new GeoPolygon();
      }
      else
      {
        polyline = new GeoPolyline();
      }

      JobArea *roadArea = new JobArea();
      roadArea->SetGeometry(polyline);
      roadArea->SetAreaType(area_type);

      m_pDrawFeature = roadArea;

      IMapCommand *pCommand = new DrawCommand(m_layer, roadArea);
      pCommand->Execute();
      m_commands.push(pCommand);

      polyline->AppendVertex(fx, fy, 0.0f);
      nValildCount = polyline->GetVertexCount();
    }
    else
    {
      int nSize = polyline->GetVertexCount();
      if (nValildCount == nSize)
      {
        polyline->AppendVertex(fx, fy, 0.0f);
      }
      else
      {
        polyline->MoveVertex(Point3d(fx, fy, 0.0f), nSize - 1);
      }

      nValildCount = polyline->GetVertexCount();
      if (polyline->GetGeometryType() == Geometry::GT_RECTANGLE)
      {
        if (nValildCount == 2)
        {
          polyline = NULL;
          return true;
        }
      }
    }
    return false;
  }

  bool Framework::StartNewLine(DrawType type, double fx, double fy, bool llink)
  {
    SegmentLayer *player = (SegmentLayer *)m_layer;

    if (m_pCamera != &m_PlanCamera)
    {
      return false;
    }
    if (polyline == NULL)
    {
      int index = -1;
      Geometry *pline = NULL; // player->GetHightLine(index);

      if (pline != NULL && index != -1)
      {
        if (llink)
        {
          polyline = pline;
        }
        else
        {
          polyline = CreateGeoLine(geoline_type_);

          LaneSegment *roadSeg = new LaneSegment();
          roadSeg->SetGeometry(polyline);
          roadSeg->GetProperty()->turnType = type.subtype;
          m_pDrawFeature = roadSeg;

          IMapCommand *pCommand = new DrawCommand(player, roadSeg);
          pCommand->Execute();
          m_commands.push(pCommand);

          Point3d point = pline->GetVertex(index);
          polyline->AppendVertex(point);
        }

        nValildCount = polyline->GetVertexCount();
      }
      else
      {
        polyline = CreateGeoLine(geoline_type_);
        polyline->AppendVertex(fx, fy, 0.0f);

        LaneSegment *roadSeg = new LaneSegment();
        roadSeg->SetGeometry(polyline);
        roadSeg->GetProperty()->turnType = type.subtype;
        m_pDrawFeature = roadSeg;

        IMapCommand *pCommand = new DrawCommand(player, roadSeg);
        pCommand->Execute();
        m_commands.push(pCommand);

        /*
        polyline->AppendVertex(30.85, 83.7,	0);
        polyline->AppendVertex(32.1384, 84.1703, 0);
        polyline->AppendVertex(38.1633, 22.2983, 0);
        polyline->AppendVertex(30.85, 83.7, 0);
        */

        nValildCount = polyline->GetVertexCount();
      }
    }
    else
    {
      int nSize = polyline->GetVertexCount();
      if (nValildCount == nSize)
      {
        polyline->AppendVertex(fx, fy, 0.0f);
      }
      else
      {
        polyline->MoveVertex(Point3d(fx, fy, 0.0f), nSize - 1);
      }
      nValildCount = polyline->GetVertexCount();

      if (polyline->GetGeometryType() == Geometry::GT_CIRCULAR_ARC)
      {
        if (nValildCount == 3)
        {
          polyline = NULL;
          return true;
        }
      }
    }

    return false;
  }

  bool Framework::StartNewBoundary(DrawType type, double fx, double fy,
                                   bool llink)
  {
    BoundaryLayer *player = (BoundaryLayer *)m_layer;

    if (m_pCamera != &m_PlanCamera)
    {
      return false;
    }
    if (polyline == NULL)
    {
      int index = -1;
      Geometry *pline = NULL; // player->GetHightLine(index);

      if (pline != NULL && index != -1)
      {
        if (llink)
        {
          polyline = pline;
        }
        else
        {
          polyline = CreateGeoLine(geoline_type_);

          BoundSegment *boundary = new BoundSegment();
          boundary->GetProperty()->boundType = type.subtype;
          boundary->SetGeometry(polyline);
          player->AddMapFeature(boundary);

          m_pDrawFeature = boundary;

          Point3d point = pline->GetVertex(index);
          polyline->AppendVertex(point);
        }

        nValildCount = polyline->GetVertexCount();
      }
      else
      {
        polyline = CreateGeoLine(geoline_type_);

        BoundSegment *boundary = new BoundSegment();
        boundary->GetProperty()->boundType = type.subtype;
        boundary->SetGeometry(polyline);

        player->AddMapFeature(boundary);
        m_pDrawFeature = boundary;

        polyline->AppendVertex(fx, fy, 0.0f);
        nValildCount = polyline->GetVertexCount();
      }
    }
    else
    {
      int nSize = polyline->GetVertexCount();
      if (nValildCount == nSize)
      {
        polyline->AppendVertex(fx, fy, 0.0f);
      }
      else
      {
        polyline->MoveVertex(Point3d(fx, fy, 0.0f), nSize - 1);
      }
      nValildCount = polyline->GetVertexCount();

      if (polyline->GetGeometryType() == Geometry::GT_CIRCULAR_ARC)
      {
        if (nValildCount == 3)
        {
          polyline = NULL;
          return true;
        }
      }
    }

    return false;
  }

  void Framework::DrawPointToGeoemtry(double fx, double fy)
  {
    if (m_pCamera != &m_PlanCamera)
    {
      return;
    }

    if (polyline != NULL)
    {
      int nSize = polyline->GetVertexCount();
      if (nSize == nValildCount)
      {
        polyline->AppendVertex(fx, fy, 0.0f);
      }
      else
      {
        polyline->MoveVertex(Point3d(fx, fy, 0.0f), nSize - 1);
      }
    }
  }

  void Framework::EndDrawGeoemtry()
  {
    if (m_pCamera != &m_PlanCamera)
    {
      return;
    }

    if (polyline != NULL)
    {
      polyline->Resize(nValildCount);
      if (!polyline->IsVaild())
      {
        Layer *pLayer = m_pDrawFeature->GetMapLayer();
        pLayer->DeleteMapFeature(m_pDrawFeature);
      }

      m_pDrawFeature = NULL;
      polyline = NULL;
    }
  }

  bool Framework::IsEndGeoemtry() { return (polyline == NULL); }

  bool Framework::DeleteDrawGeomtry()
  {
    if (m_pDrawFeature)
    {
      Layer *pLayer = m_pDrawFeature->GetMapLayer();
      pLayer->DeleteMapFeature(m_pDrawFeature);

      m_pDrawFeature = NULL;
      polyline = NULL;

      return true;
    }

    return false;
  }

  void Framework::HighlightSegment(double x, double y, int type)
  {
    // return;
    if (type == 2)
    {
      if (m_pHighFeature != NULL)
      {
        // 取消之前高亮对象和高亮点
        m_pHighFeature->SetHighlightState(false);
        m_pHighFeature->SetHighlightPoint(-1);
        m_pHighFeature = NULL;
      }
      return;
    }
    double tolerance = (2.0 / m_fMapScale) / m_viewport.Width() * 30;
    PickupResult reslut;
    for (int i = 0; i < layers_.size(); i++)
    {
      layers_[i]->PickupObject(Point3d(x, y, 0), tolerance, reslut);
    }

    if (m_pHighFeature != reslut.pFeatureObject)
    {
      if (m_pHighFeature != NULL)
      {
        // 取消之前高亮对象和高亮点
        m_pHighFeature->SetHighlightState(false);
        m_pHighFeature->SetHighlightPoint(-1);
      }

      m_pHighFeature = reslut.pFeatureObject;

      if (m_pHighFeature)
      {
        if (type == 0)
          m_pHighFeature->SetHighlightState(true);
        m_pHighFeature->SetHighlightPoint(reslut.nKeyPoint);
      }
    }
    else
    {
      if (m_pHighFeature)
      {
        m_pHighFeature->SetHighlightPoint(reslut.nKeyPoint);
      }
    }
  }
  int Framework::HighlightPoint(double &x, double &y)
  {
    if (!m_pHighFeature)
      return -1;
    int idx = m_pHighFeature->GetHighlightPoint();
    if (idx < 0)
      return -1;
    auto *geo = m_pHighFeature->GetGeometry();
    if (!geo || idx >= geo->GetVertexCount())
      return -1;
    auto p = geo->GetVertex(idx);
    x = p.x, y = p.y;
    return 0;
  }

  MoveObjectCommand *pMoveCommand = NULL;

  void Framework::EndMoveObject()
  {
    if (pMoveCommand)
    {
      m_commands.push(pMoveCommand);
      pMoveCommand = NULL;
    }
  }

  void Framework::MoveMapObject(double x, double y)
  {
    std::vector<MapFeature *> moveSet;
    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      if (m_selectedSet[i]->GetMapLayer()->IsEnableEdit())
      {
        moveSet.push_back(m_selectedSet[i]);
      }
    }

    if (moveSet.size() > 0)
    {
      if (pMoveCommand)
      {
        pMoveCommand->Update(Point3d(x, y, 0.0));
        pMoveCommand->Execute();
      }
      else
      {
        pMoveCommand = new MoveObjectCommand(moveSet, Point3d(x, y, 0.0));
      }
    }
  }

  void Framework::AddSegment(std::vector<Lane *> segArray)
  {
    if (m_Segmentlayer)
    {
      m_Segmentlayer->AddMapObject(segArray);
    }
  }

  void Framework::AddTrajectory(std::vector<Lane *> segArray)
  {
    if (m_trajectorylayer)
    {
      m_trajectorylayer->AddMapObject(segArray);
    }
  }

  void Framework::AddTrajectory(std::vector<BoundSegment *> segArray)
  {
    if (m_trajectorylayer)
    {
      for (int i = 0; i < segArray.size(); i++)
      {
        m_trajectorylayer->AddMapFeature(segArray[i]);
      }
    }
  }

  void Framework::AddBoundary(std::vector<Lane *> segArray)
  {
    if (m_boundlayer)
    {
      m_boundlayer->AddMapObject(segArray);
    }
  }

  void Framework::AddBoundary(std::vector<BoundSegment *> segArray)
  {
    if (m_boundlayer)
    {
      for (int i = 0; i < segArray.size(); i++)
      {
        m_boundlayer->AddMapFeature(segArray[i]);
      }
    }
  }

  void Framework::AddJobArea(std::vector<JobArea *> segArray)
  {
    if (m_joblayer)
    {
      m_joblayer->AddMapObject(segArray);
    }
  }

  double OnDistance(const Point3d &P1, const Point3d &P2, const Point3d &pt,
                    Point3d &outPt)
  {
    double fDot = (P2.x - P1.x) * (pt.x - P1.x) + (P2.y - P1.y) * (pt.y - P1.y);
    if (fDot <= 0.0f)
    {
      outPt = P1;
      return sqrt((P1.x - pt.x) * (P1.x - pt.x) + (P1.y - pt.y) * (P1.y - pt.y));
    }

    double d2AB = (P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y);
    if (fDot >= d2AB)
    {
      outPt = P1;
      return sqrt((P2.x - pt.x) * (P2.x - pt.x) + (P2.y - pt.y) * (P2.y - pt.y));
    }

    double u = fDot / d2AB;

    outPt.x = P1.x + (P2.x - P1.x) * u;
    outPt.y = P1.y + (P2.y - P1.y) * u;
    outPt.z = 0.0;

    return sqrt((pt.x - outPt.x) * (pt.x - outPt.x) +
                (pt.y - outPt.y) * (pt.y - outPt.y));
  }

  bool IsRightBoundary(Geometry *newSegment, Geometry *pBoundary)
  {
    int nLeftCount = 0;
    int nRightCount = 0;

    int iSize = newSegment->GetVertexCount();
    for (int i = 0; i < iSize - 1; i++)
    {
      Point3d point = newSegment->GetVertex(i);

      Point3d minPoint;
      int nIndex = -1;
      double dDistane = 1000000;

      int iCount = pBoundary->GetVertexCount();
      for (int j = 0; j < iCount - 1; j++)
      {
        const Point3d &Pi = pBoundary->GetVertex(j);
        const Point3d &Pj = pBoundary->GetVertex(j + 1);

        Point3d outPt;
        double dist = OnDistance(Pi, Pj, point, outPt);
        if (dDistane > dist)
        {
          dDistane = dist;
          minPoint = outPt;
          nIndex = j;
        }
      }

      Point3d point2 = newSegment->GetVertex(i + 1);

      V3d vec1((point2.x - point.x), (point2.y - point.y), 0.0);
      V3d vec2((minPoint.x - point.x), (minPoint.y - point.y), 0.0);

      double dCross = vec1[0] * vec2[1] - vec1[1] * vec2[0];
      if (dCross > 0)
      {
        nLeftCount++;
      }
      else
      {
        nRightCount++;
      }
    }

    return (nLeftCount < nRightCount);
  }

  void Framework::ConvertToBoundary(int type)
  {
    std::vector<BoundSegment *> boundarys;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];
      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        boundarys.push_back((BoundSegment *)pFeature);
      }
    }

    for (int x = 0; x < boundarys.size(); x++)
    {
      GeoPolygon *pPolygon = new GeoPolygon();

      Geometry *pGeomtry = boundarys[x]->GetGeometry();
      for (int i = 0; i < pGeomtry->GetVertexCount(); i++)
      {
        const Point3d &pt = pGeomtry->GetVertex(i);
        pPolygon->AppendVertex(pt);
      }

      RoadArea *pArea = new RoadArea();
      pArea->SetGeometry(pPolygon);
      pArea->SetAreaType(type);

      m_areaLayer->AddMapFeature(pArea);
    }
  }

  PositionTransformNode *m_pDistaneNode = NULL;

  GeoPolyline *pDistaneLine = NULL;
  int hasValidCount = 0;
  double dMeasureDistance = 0.0;

  void Framework::BeginCliperPolygon(double x, double y)
  {
    if (pDistaneLine == NULL)
    {
      dMeasureDistance = 0;
      pDistaneLine = new GeoPolyline();

      pDistaneLine->AppendVertex(Point3d(x, y, 0));
    }
    else
    {
      int nCount = pDistaneLine->GetVertexCount();
      if (hasValidCount < nCount)
      {
        pDistaneLine->MoveVertex(Point3d(x, y, 0), nCount - 1);
      }
      else
      {
        pDistaneLine->AppendVertex(Point3d(x, y, 0));
      }
    }

    hasValidCount = pDistaneLine->GetVertexCount();
  }

  void Framework::MoveCliperPolygon(double x, double y)
  {
    if (pDistaneLine != NULL)
    {
      int nCount = pDistaneLine->GetVertexCount();

      if (hasValidCount < nCount)
      {
        pDistaneLine->MoveVertex(Point3d(x, y, 0), nCount - 1);
      }
      else
      {
        pDistaneLine->AppendVertex(Point3d(x, y, 0));
      }
    }
  }

  void Framework::EndCliperPolygon()
  {
    if (pDistaneLine != NULL)
    {
      pDistaneLine->Resize(hasValidCount);

      // 直线线和多边形切分
      m_joblayer->CliperLayer(pDistaneLine);

      delete pDistaneLine;
      pDistaneLine = NULL;
    }

    if (m_pDistaneNode != NULL)
    {
      delete m_pDistaneNode;
      m_pDistaneNode = NULL;
    }
  }

  bool Framework::GenerateSweepArea()
  {
    std::vector<Geometry *> boundaryArea;
    m_boundlayer->GetBoundaryArea(boundaryArea);

    if (boundaryArea.size() > 0)
    {
      JobArea *roadArea = new JobArea();
      roadArea->SetGeometry(boundaryArea[0]);
      roadArea->SetAreaType(104);

      m_joblayer->AddMapFeature(roadArea);

      return true;
    }
    return false;
  }

  void Framework::MergeObject()
  {
    std::vector<BoundSegment *> arrayBoundary;
    std::vector<LaneSegment *> arraySegment;
    std::vector<JobArea *> arrayJobArea;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];
      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        arrayBoundary.push_back((BoundSegment *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        arraySegment.push_back((LaneSegment *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_JOB_AREA)
      {
        arrayJobArea.push_back((JobArea *)pFeature);
      }
    }

    m_boundlayer->MergeBoundary(arrayBoundary);
    m_Segmentlayer->MergeObject(arraySegment);
    m_joblayer->MergeObject(arrayJobArea);
  }

  static bool Orientation(Geometry *pPolygon)
  {
    int size = (int)pPolygon->GetVertexCount();
    if (size < 3)
    {
      return 0;
    }

    double a = 0;
    for (int i = 0, j = size - 1; i < size; ++i)
    {
      Point3d const &U0 = pPolygon->GetVertex(i);
      Point3d const &U1 = pPolygon->GetVertex(j);

      a += ((double)U1.x + U0.x) * ((double)U1.y - U0.y);
      j = i;
    }
    return (-a * 0.5) >= 0;
  }

  bool Framework::GenerateSweepPath(int type, int mouseX, int mouseY)
  {
    std::vector<JobArea *> boundarys;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_JOB_AREA)
      {
        boundarys.push_back((JobArea *)pFeature);
      }
    }

    if (boundarys.size() > 0)
    {
      Geometry *geometry = boundarys[0]->GetGeometry();

      std::vector<double> x_contour;
      std::vector<double> y_contour;

      if (!Orientation(geometry))
      {
        geometry->ReverseVertex();
      }

      int nVCount = geometry->GetVertexCount();
      for (int i = 0; i < nVCount; i++)
      {
        const Point3d &pt3d = geometry->GetVertex(i);

        x_contour.push_back(pt3d.x);
        y_contour.push_back(pt3d.y);
      }

      Point3d startPt;
      // MousePointToCart(mouseX, mouseY, startPt.x, startPt.y);
      geometry->CalculateBoundBox();
      BoundBox3d bound = geometry->GetBound();
      V3d vecPt = bound.GetCenter();

      startPt.x = vecPt[0];
      startPt.y = vecPt[1];

      Geometry *geoPoly = NULL;

      if (type == 0)
      {
        coverage::HybridTrack test;

        bool ret = test.Interface(x_contour, y_contour);
        test.Planning(m_keyPoint >= 0 ? m_keyPoint : 0);

        int iPathCount = test.final_path_.size();
        //================================================
        geoPoly = CreateGeoLine(4);
        auto final_path = test.final_path_;
        for (int i = 0; i < iPathCount; i++)
        {
          Site pt = test.final_path_[i];
          geoPoly->AppendVertex(pt.x, pt.y, 0.0);
        }
      }
      else if (type == 1)
      {
        coverage::HybridRing test;
        test.Interface(x_contour, y_contour);
        test.Planning();

        int iPathCount = test.final_path_.size();

        //================================================
        geoPoly = CreateGeoLine(4);
        for (int i = 0; i < iPathCount; i++)
        {
          Site pt = test.final_path_[i];
          geoPoly->AppendVertex(pt.x, pt.y, 0.0);
        }
      }
      else if (type == 2)
      {
        coverage::PipePlanner test;

        std::vector<double> x_edge;
        std::vector<double> y_edge;
        if (x_contour.size() > 4 && y_contour.size() > 4)
        {
          x_edge.push_back(x_contour[0]);
          x_edge.push_back(x_contour[1]);
          x_edge.push_back(x_contour[x_contour.size() - 2]);
          x_edge.push_back(x_contour[x_contour.size() - 1]);
          y_edge.push_back(y_contour[0]);
          y_edge.push_back(y_contour[1]);
          y_edge.push_back(y_contour[y_contour.size() - 2]);
          y_edge.push_back(y_contour[y_contour.size() - 1]);
          test.Interface(x_contour, y_contour, x_edge, y_edge);
          test.Planning();
        }

        int iPathCount = test.final_path_.size();
        //================================================
        geoPoly = CreateGeoLine(1);
        for (int i = 0; i < iPathCount; i++)
        {
          Site pt = test.final_path_[i];
          geoPoly->AppendVertex(pt.x, pt.y, 0.0);
        }
      }
      else
      {
        coverage::HybridTrack test;

        bool ret = test.Interface(x_contour, y_contour);
        test.Planning(m_keyPoint >= 0 ? m_keyPoint : 0);

        int iPathCount = test.final_path_.size();
        //================================================
        geoPoly = CreateGeoLine(1);
        for (int i = 0; i < iPathCount; i++)
        {
          Site pt = test.final_path_[i];
          geoPoly->AppendVertex(pt.x, pt.y, 0.0);
        }
      }

      if (geoPoly != NULL && geoPoly->GetVertexCount() > 0)
      {
        LaneSegment *roadSeg = new LaneSegment();
        roadSeg->SetGeometry(geoPoly);

        IMapCommand *pCommand = new DrawCommand(m_Segmentlayer, roadSeg);
        pCommand->Execute();
        m_commands.push(pCommand);

        return true;
      }
      return false;
    }
    return true;
  }

  void Framework::SetPathPointEdge(bool bDelete)
  {
    std::vector<PointElement *> boundarys;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_POINT_ELEMENT)
      {
        boundarys.push_back((PointElement *)pFeature);
      }
    }

    if (boundarys.size() >= 2)
    {
      if (bDelete)
      {
        m_topologylayer->DeleteTopology(boundarys[0]->GetUniqueID(),
                                        boundarys[1]->GetUniqueID());
      }
      else
      {
        m_topologylayer->AddTopology(boundarys[0]->GetUniqueID(),
                                     boundarys[1]->GetUniqueID());
      }
    }
  }

  int Framework::SetLaneBoundary()
  {
    std::vector<BoundSegment *> boundarys;
    std::vector<LaneSegment *> laneSegments;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        boundarys.push_back((BoundSegment *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        laneSegments.push_back((LaneSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }
    if (laneSegments.size() != 1 || boundarys.size() != 2)
    {
      LOG(WARNING) << "Please select 1 lane and 2 boundarys!";
      return -1;
    }

    // for (int x = 0; x < boundarys.size(); x++) {
    //   for (int i = 0; i < laneSegments.size(); i++) {
    //     SegmentProperty *pProperty = laneSegments[i]->GetProperty();
    //     if (IsRightBoundary(laneSegments[i]->GetGeometry(),
    //                         boundarys[x]->GetGeometry())) {
    //       pProperty->rightBoundary = boundarys[x]->GetUniqueID();
    //     } else {
    //       pProperty->leftBoundary = boundarys[x]->GetUniqueID();
    //     }
    //   }
    // }

    int r = -1, l = -1;
    SegmentProperty *pProperty = laneSegments[0]->GetProperty();
    if (IsRightBoundary(laneSegments[0]->GetGeometry(),
                        boundarys[0]->GetGeometry()))
      r = 0;
    else
      l = 0;
    if (IsRightBoundary(laneSegments[0]->GetGeometry(),
                        boundarys[1]->GetGeometry()))
      r = 1;
    else
      l = 1;
    if (r * l < 0)
      return 0;
    else
    {
      // pProperty->rightBoundary = boundarys[r]->GetUniqueID();
      // pProperty->leftBoundary = boundarys[l]->GetUniqueID();
      laneSegments[0]->SetLeftBoundary(boundarys[l]);
      laneSegments[0]->SetRightBoundary(boundarys[r]);
    }
    return 1;
  }

  MovePointCommand *pPointCommand = NULL;

  void Framework::MoveSegment(double x, double y)
  {
    if (m_selectFeature != NULL && m_keyPoint != -1)
    {
      if (m_selectFeature->GetMapLayer()->IsEnableEdit())
      {
        if (!pPointCommand)
        {
          pPointCommand = new MovePointCommand(m_selectFeature, m_keyPoint);
        }

        pPointCommand->Update(Point3d(x, y, 0.0));
        pPointCommand->Execute();
      }
    }
  }

  void Framework::EndMovePoint()
  {
    if (pPointCommand)
    {
      m_commands.push(pPointCommand);
      pPointCommand = NULL;
    }
  }

  void Framework::BreakSegment(double x, double y)
  {
    double tolerance = (2.0 / m_fMapScale) / m_viewport.Width() * 30;

    {
      PickupResult reslut;
      int ret =
          m_Segmentlayer->PickupObject(Point3d(x, y, 0.0), tolerance, reslut);

      if (reslut.pFeatureObject)
      {
        m_Segmentlayer->BreakPolyline(reslut.pFeatureObject, reslut.nSegmentIdx,
                                      reslut.ptNearPoint);
        return;
      }
    }

    {
      PickupResult reslut;
      int ret = m_boundlayer->PickupObject(Point3d(x, y, 0.0), tolerance, reslut);

      if (reslut.pFeatureObject)
      {
        m_boundlayer->BreakPolyline(reslut.pFeatureObject, reslut.nSegmentIdx,
                                    reslut.ptNearPoint);
      }
    }
  }

  void Framework::InsertPoint(double x, double y)
  {
    double tolerance = (2.0 / m_fMapScale) / m_viewport.Width() * 20;

    PickupResult reslut;
    for (int i = 0; i < layers_.size(); i++)
    {
      layers_[i]->PickupObject(Point3d(x, y, 0), tolerance, reslut);
    }

    m_selectFeature = reslut.pFeatureObject;
    m_keyPoint = reslut.nKeyPoint;

    // 插入点
    if (m_selectFeature != NULL)
    {
      InsertPointCommand *pCommand = new InsertPointCommand(
          m_selectFeature, reslut.nSegmentIdx, reslut.ptNearPoint);
      pCommand->Execute();
      m_commands.push(pCommand);

      for (int i = 0; i < m_selectedSet.size(); i++)
      {
        m_selectedSet[i]->SetSelectedState(false);
      }
      m_selectedSet.clear();

      reslut.pFeatureObject->SetSelectedState(true);
      m_selectedSet.push_back(reslut.pFeatureObject);
    }
  }

  bool Framework::SelectTrajectoryImage(double x, double y, MapFeature *&pFeature,
                                        int &keyPoint)
  {
    PickupResult trajReslut;

    double tolerance = (2.0 / m_fMapScale) / m_viewport.Width() * 30;
    m_trajectorylayer->PickupObject(Point3d(x, y, 0), tolerance, trajReslut);

    if (trajReslut.pFeatureObject != NULL)
    {
      pFeature = trajReslut.pFeatureObject;
      keyPoint = trajReslut.nKeyPoint;

      m_selectTrajecFeature = pFeature;
      m_keyTrajecPoint = keyPoint;
      // m_selectFeature = pFeature;
      // m_keyPoint = keyPoint;
      return true;
    }

    return false;
  }

  void Framework::SelectSegment(bool mulselect, bool bLink, double x, double y)
  {
    double tolerance = (2.0 / m_fMapScale) / m_viewport.Width() * 30;
    //-----------------------------------------------------------------------

    PickupResult reslut;
    for (int i = 0; i < layers_.size(); i++)
    {
      // if (m_trajectorylayer == layers_[i]) {
      //   continue;
      // } else {
      layers_[i]->PickupObject(Point3d(x, y, 0), tolerance, reslut);
      // }
    }

    m_selectFeature = reslut.pFeatureObject;
    m_keyPoint = reslut.nKeyPoint;

    // 拾取到对象
    if (reslut.pFeatureObject != NULL)
    {
      // 查找上次选中的拓扑点
      MapFeature *prePointFeature = NULL;
      for (int i = 0; i < m_selectedSet.size(); i++)
      {
        if (m_selectedSet[i]->GetType() == MapFeature::MFT_POINT_ELEMENT)
        {
          prePointFeature = m_selectedSet[i];
        }
      }

      // 非多选情况下，清除以前的
      if (!mulselect)
      {
        for (int i = 0; i < m_selectedSet.size(); i++)
        {
          m_selectedSet[i]->SetSelectedState(false);
        }

        m_selectedSet.clear();
      }

      // 保存选择的对象
      if (std::find(m_selectedSet.begin(), m_selectedSet.end(),
                    reslut.pFeatureObject) == m_selectedSet.end())
      {
        reslut.pFeatureObject->SetSelectedState(true);
        m_selectedSet.push_back(reslut.pFeatureObject);
      }

      if (bLink)
      {
        if (reslut.pFeatureObject->GetType() == MapFeature::MFT_POINT_ELEMENT &&
            prePointFeature != NULL)
        {
          m_topologylayer->AddTopology(prePointFeature->GetUniqueID(),
                                       reslut.pFeatureObject->GetUniqueID());
        }
      }
    }
    else
    {
      for (int i = 0; i < m_selectedSet.size(); i++)
      {
        m_selectedSet[i]->SetSelectedState(false);
      }

      m_selectedSet.clear();
    }
  }

  void Framework::SetPointCloudFilter(float minVal, float maxVal)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->SetPointCloudFilter(minVal, maxVal);
    }
  }

  void Framework::SetPointHighFilter(float minVal, float maxVal)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->SetPointHighFilter(minVal, maxVal);
    }
  }

  bool Framework::GetPointHighFilter(float &minVal, float &maxVal)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->GetPointHighFilter(minVal, maxVal);
      return true;
    }
    return false;
  }

  void Framework::SetColorType(int type, int r)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->SetColorType(type, r);
    }
  }
  bool Framework::GetColorType(int &type, int &r)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->GetColorType(type, r);
      return true;
    }
    return false;
  }

  bool Framework::GetPointCloudFilter(float &minVal, float &maxVal)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->GetPointCloudFilter(minVal, maxVal);

      return true;
    }
    return false;
  }

  void Framework::ReverseSegment()
  {
    std::vector<MapFeature *> lanes;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];
      if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        lanes.push_back(pFeature);
      }
    }

    if (lanes.size() > 0)
    {
      ReverseCommand *pCommand = new ReverseCommand(lanes);
      pCommand->Execute();
      m_commands.push(pCommand);
    }
  }

  bool Framework::OptimizeCurve3(double x, double y, int &counter)
  {
    if (m_selectFeature != NULL &&
        m_selectFeature->GetType() == MapFeature::MFT_LANE_SEG)
    {
      Geometry *pGeo = m_selectFeature->GetGeometry();

      if (pGeo != NULL && pGeo->GetGeometryType() == Geometry::GT_BSPLINE_CURVE)
      {
        counter =
            ((GeoBSplineCurve3 *)pGeo)->OptimizeCurvature(Point3d(x, y, 0.0));

        ((LaneSegment *)m_selectFeature)->SetChanged(true);

        return true;
      }
    }
    return false;
  }

  LaneSegment *Framework::GetLaneSegment()
  {
    if (m_selectFeature != NULL)
    {
      if (m_selectFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        return (LaneSegment *)m_selectFeature;
      }
    }
    return NULL;
  }

  MapFeature *Framework::GetMapFeature() { return m_selectFeature; }

  MapFeature *Framework::GetTrajecMapFeature() { return m_selectTrajecFeature; }

  void Framework::ClearTrajecMapFeature() { m_selectTrajecFeature = NULL; }

  void Framework::SetTrajecKeyPoint(int nKeyPoint) { m_keyPoint = nKeyPoint; }

  void Framework::DeleteMapFeature()
  {
    size_t nsize = m_selectedSet.size();
    if (nsize > 0)
    {
      // 选中的一条线，且选中了关键点，则删除关键点
      if (nsize == 1 && m_keyPoint != -1)
      {
        if (m_selectFeature->GetMapLayer()->IsEnableEdit())
        {
          Geometry *pGeometry = m_selectFeature->GetGeometry();
          int iCount = pGeometry->GetVertexCount();

          // 删除关键点之后，必须大于2
          if (iCount > 2)
          {
            pGeometry->RemoveVertex(m_keyPoint);
            return;
          }
        }
      }

      std::vector<MapFeature *> deleteSet;
      for (int i = 0; i < m_selectedSet.size(); i++)
      {
        if (m_selectedSet[i]->GetMapLayer()->IsEnableEdit())
        {
          deleteSet.push_back(m_selectedSet[i]);
        }
      }

      if (deleteSet.size() > 0)
      {
        IMapCommand *pCommand = new DeleteCommand(deleteSet);
        pCommand->Execute();

        m_commands.push(pCommand);
      }
    }
  }

  int Framework::SetParallelSegment()
  {
    std::vector<LaneSegment *> laneSegments;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        laneSegments.push_back((LaneSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }

    int nSize = laneSegments.size();
    if (nSize > 1)
    {
      m_Segmentlayer->SetParallelSegment(laneSegments);

      return nSize;
    }

    return 0;
  }

  int Framework::SetReverseSegment()
  {
    std::vector<LaneSegment *> laneSegments;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        laneSegments.push_back((LaneSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }

    int nSize = laneSegments.size();
    if (nSize == 2)
    {
      m_Segmentlayer->SetReverseSegment(laneSegments);
      return nSize;
    }

    return 0;
  }
  int OverlapType(Geometry *area, Geometry *obj) { return 0; }

  void Framework::GenerateTopology() { m_Segmentlayer->GenerateTopology(); }

  // 设置附加对象
  int Framework::SetSegmentRelation()
  {
    std::vector<LaneSegment *> laneSegments;

    std::vector<RoadArea *> attchRoadAreas;
    std::vector<SignBoard *> attchSignBoards;
    std::vector<JobArea *> attchJobAreas;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        laneSegments.push_back((LaneSegment *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_ROAD_AREA)
      {
        attchRoadAreas.push_back((RoadArea *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_SIGNBORAD)
      {
        attchSignBoards.push_back((SignBoard *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_JOB_AREA)
      {
        attchJobAreas.push_back((JobArea *)pFeature);
      }
    }
    if (laneSegments.empty())
      return -1;
    if (attchRoadAreas.empty() && attchSignBoards.empty())
      return 0;
    for (int i = 0; i < laneSegments.size(); i++)
    {
      // for (int j = 0; j < attchRoadAreas.size(); j++) {
      //   laneSegments[i]->AddAttachObject(attchRoadAreas[j]);
      // }

      // for (int j = 0; j < attchSignBoards.size(); j++) {
      //   laneSegments[i]->AddAttachObject(attchSignBoards[j]);
      //   attchSignBoards[j]->AddRelationSegment(laneSegments[i]);
      // }
      for (int j = 0; j < attchJobAreas.size(); j++)
      {
        laneSegments[i]->AddAttachObject(attchJobAreas[j]);
        attchJobAreas[j]->AddAttachObject(laneSegments[i]);
      }
    }
    return 1;
  }

  int Framework::SetTrafficStopline()
  {
    std::vector<RoadArea *> arrStopline;
    std::vector<SignBoard *> arrSignboard;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_ROAD_AREA)
      {
        arrStopline.push_back((RoadArea *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_SIGNBORAD)
      {
        arrSignboard.push_back((SignBoard *)pFeature);
      }
    }
    if (arrSignboard.empty())
      return -1;
    if (arrStopline.empty())
      return 0;

    for (int i = 0; i < arrSignboard.size(); i++)
    {
      for (int j = 0; j < arrStopline.size(); j++)
      {
        arrSignboard[i]->SetRelationStopline(arrStopline[j]);
      }
    }

    return 1;
  }

  LayerType Framework::GetActiveLayer() const { return m_layertype; }

  bool Framework::ShowHideTrackLayer(bool show)
  {
    if (m_trajectorylayer)
    {
      m_trajectorylayer->SetVisible(show);
    }

    return true;
  }

  bool Framework::ShowHideJobLayer(bool show)
  {
    if (m_joblayer)
    {
      m_joblayer->SetVisible(show);
    }
    return true;
  }

  bool Framework::ShowHideSegmentlayerLayer(bool show)
  {
    if (m_Segmentlayer)
    {
      m_Segmentlayer->SetVisible(show);
    }
    return true;
  }

  bool Framework::ShowHideAreaLayer(bool show)
  {
    if (m_areaLayer)
    {
      m_areaLayer->SetVisible(show);
    }
    return true;
  }

  bool Framework::ShowHideBoundaryLayer(bool show)
  {
    if (m_boundlayer)
    {
      m_boundlayer->SetVisible(show);
    }
    return true;
  }

  bool Framework::ShowHidePDBLayer(bool show)
  {
    if (m_pdbLayer)
    {
      m_pdbLayer->SetVisible(show);
    }
    return true;
  }

  bool Framework::ShowHideSignLayer(bool show)
  {
    if (m_pointLayer)
    {
      m_pointLayer->SetVisible(show);
    }
    return true;
  }

  DBReadWrite dbwrite;

  int Framework::ReadNextCrubImage(MapFeature *pMapFeature, int keyPoint,
                                   char *&blockData, int &length)
  {
    if (pMapFeature != NULL && keyPoint != -1)
    {
      if (pMapFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        BoundSegment *pFeature = (BoundSegment *)pMapFeature;

        int iCount = pFeature->GetGeometry()->GetVertexCount();
        for (int i = keyPoint; i < iCount; i++)
        {
          int retImageId = pFeature->GetImageIndex(i);
          if (retImageId != -1)
          {
            if (ReadCrubImage(retImageId, blockData, length))
            {
              return i;
            }
          }
        }
      }
    }

    return -1;
  }

  int Framework::ReadPreCrubImage(MapFeature *pMapFeature, int keyPoint,
                                  char *&blockData, int &length)
  {
    if (pMapFeature != NULL && keyPoint != -1)
    {
      if (pMapFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        BoundSegment *pFeature = (BoundSegment *)pMapFeature;

        int iCount = pFeature->GetGeometry()->GetVertexCount();
        for (int i = keyPoint; i >= 0; i--)
        {
          int retImageId = pFeature->GetImageIndex(i);
          if (retImageId != -1)
          {
            if (ReadCrubImage(retImageId, blockData, length))
            {
              return i;
            }
          }
        }
      }
    }

    return -1;
  }

  int Framework::ReadCrubImage(char *&blockData, int &length)
  {
    if (m_selectFeature != NULL && m_keyPoint != -1)
    {
      if (m_selectFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        BoundSegment *pFeature = (BoundSegment *)m_selectFeature;

        int iCount = pFeature->GetGeometry()->GetVertexCount();
        for (int i = m_keyPoint; i < iCount; i++)
        {
          int retImageId = pFeature->GetImageIndex(i);
          if (retImageId != -1)
          {
            if (ReadCrubImage(retImageId, blockData, length))
            {
              return i;
            }
          }
        }
      }
    }

    return -1;
  }

  void Framework::CloseDBMap() { dbwrite.CloseDB(); }

  bool Framework::LoadDBMap(const char *dbfilename)
  {
    if (dbwrite.IsOpen())
    {
      dbwrite.CloseDB();
    }

    if (dbwrite.OpenDB(dbfilename))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Framework::ReadCrubsTrack(std::vector<CurbsTrack *> &segmentArray)
  {
    ProjectionUTM projectionUTM;
    if (dbwrite.IsOpen())
    {
      std::vector<int> trackIdx;
      if (!dbwrite.QueryAllTrackLine(trackIdx))
      {
        return false;
      }

      for (int m = 0; m < trackIdx.size(); m++)
      {
        char *pMemBuf = NULL;
        int length = 0.0;

        // 解析文件
        if (dbwrite.ReadTrackLine(trackIdx[m], pMemBuf, length))
        {
          int iCount = length / 32;
          char *pData = pMemBuf;

          CurbsTrack *pTrack = new CurbsTrack();

          for (int i = 0; i < iCount; i++)
          {
            TrackPoint trackPnt;

            trackPnt.pnt = *(Point3d *)(pData);
            pData += sizeof(Point3d);

            trackPnt.ndt = *(float *)(pData);
            pData += sizeof(float);

            trackPnt.img = *(int *)(pData);
            pData += sizeof(int);

            //-------------------------------------------------

            UTMPoint utmxy;
            projectionUTM.LatLonToCartesian(trackPnt.pnt.y, trackPnt.pnt.x,
                                            utmxy);
            trackPnt.pnt = Point3d(utmxy.x, utmxy.y, 0.0);

            //-----------------------------------------------

            pTrack->trackSet.push_back(trackPnt);
          }
          delete[] pMemBuf;

          segmentArray.push_back(pTrack);
        }
      }
      return true;
    }

    return false;
  }

  bool Framework::ReadCrubImage(int imageId, char *&blockData, int &length)
  {
    // 解析文件
    if (dbwrite.ReOpen())
    {
      if (dbwrite.ReadTrackImage(imageId, blockData, length))
      {
        return true;
      }
    }
    return false;
  }

  void Framework::ActiveLayer(LayerType layertype)
  {
    if (m_pCamera != &m_PlanCamera)
      Set2DView();
    // 判断当前图层的绘制是否结束，否则结束绘制。
    if (!IsEndGeoemtry())
    {
      EndDrawGeoemtry();
    }
    EndMeasureDistance();

    m_layertype = layertype;
    if (layertype == LT_LANE)
    {
      m_layer = m_Segmentlayer;
    }
    else if (layertype == LT_ROADAREA)
    {
      m_layer = m_areaLayer;
    }
    else if (layertype == LT_SIGN)
    {
      m_layer = m_pointLayer;
    }
    else if (layertype == LT_BOUNDARY)
    {
      m_layer = m_boundlayer;
    }
    else if (layertype == LT_FUNAREA)
    {
      m_layer = m_joblayer;
    }
    else if (layertype == LT_TOPO)
    {
      m_layer = m_topologylayer;
    }
  }

  bool Framework::IsOpen() { return (m_vdbmgr != NULL); }

  bool Framework::Create(const char *filename)
  {
    if (m_vdbmgr == NULL)
    {
      m_vdbmgr = new VDBManage();
      if (m_vdbmgr->Create(filename))
      {
        m_vdbmgr->Close();
        return true;
      }
    }
    else
    {
      m_vdbmgr->Close();
      if (m_vdbmgr->Create(filename))
      {
        m_vdbmgr->Close();
        return true;
      }
      return false;
    }
    return false;
  }

  void Framework::SetDataSource(const char *szFileName)
  {
    m_pdbLayer->SetDataSource(szFileName);
  }

  void Framework::CloseDataSource() { m_pdbLayer->CloseDataSource(); }

  void Framework::Close()
  {
    std::stack<IMapCommand *> tmp_command, tmp_history;
    m_commands.swap(tmp_command);
    m_histroys.swap(tmp_history);

    m_selectedSet.clear();

    m_selectFeature = NULL;
    m_keyPoint = -1;

    // 数据库已经，关闭之前数据库
    if (m_vdbmgr != NULL)
    {
      m_Segmentlayer->ClearLayer();

      m_boundlayer->ClearLayer();

      m_areaLayer->ClearLayer();

      m_pointLayer->ClearLayer();

      m_joblayer->ClearLayer();

      m_topologylayer->ClearLayer();

      m_vdbmgr->Close();
    }
  }

  bool Framework::Read(const char *filename)
  {
    std::stack<IMapCommand *> tmp_command, tmp_history;
    m_commands.swap(tmp_command);
    m_histroys.swap(tmp_history);

    m_selectedSet.clear();

    m_selectFeature = NULL;
    m_keyPoint = -1;

    // 数据库已经，关闭之前数据库
    if (m_vdbmgr != NULL)
    {
      m_vdbmgr->Close();
    }
    else
    {
      m_vdbmgr = new VDBManage();
    }

    if (m_vdbmgr->Create(filename))
    {
      m_boundlayer->Read(m_vdbmgr);

      m_areaLayer->Read(m_vdbmgr);

      m_pointLayer->Read(m_vdbmgr, m_Segmentlayer);

      m_joblayer->Read(m_vdbmgr, m_boundlayer);

      m_Segmentlayer->Read(m_vdbmgr);

      m_topologylayer->Read(m_vdbmgr);

      m_vdbmgr->Close();

      return true;
    }

    return false;
  }

  void Framework::Save()
  {
    if (m_vdbmgr != NULL && m_vdbmgr->Open())
    {
      m_Segmentlayer->Save(m_vdbmgr);

      m_boundlayer->Save(m_vdbmgr);

      m_areaLayer->Save(m_vdbmgr);

      m_pointLayer->Save(m_vdbmgr);

      m_joblayer->Save(m_vdbmgr);

      m_topologylayer->Save(m_vdbmgr);

      m_vdbmgr->Close();
    }
  }

  void Framework::Resize(int x, int y, int width, int height)
  {
    m_viewport.SetViewport(0.0f, 0.0f, (float)width, (float)height);

    m_trackball.SetViewport(m_viewport);

    m_PlanCamera.SetViewport(m_viewport);
    m_PlanCamera.SetOrtho(m_viewport.AspectRatio(), 0.5f, 500.0f);

    m_StereoCamera.SetViewport(m_viewport);
    m_StereoCamera.SetPerspective(45.0f, m_viewport.AspectRatio(), 0.5f, 500.0f);
  }

  // 计算线条

  PositionTransformNode *m_pTransformNode = NULL;

  double m_dminX = 0, m_dminY = 0;
  double m_dmaxX = 0, m_dmaxY = 0;
  double gridSize = 1;

  double m_gridSize = 0;

  void Framework::SetGridSize(double x, double y) { gridSize = x; }

  double Framework::GetGridSize() { return gridSize; }

  double Framework::GetPointSize() { return m_Segmentlayer->GetPointSize(); }

  void Framework::SetPointSize(double x)
  {
    m_Segmentlayer->SetPointSize(x);
    m_joblayer->SetPointSize(x);
  }

  double Framework::GetLineWidth() { return m_Segmentlayer->GetLineWidth(); }

  bool Framework::GetCarBodyCheck() { return m_Segmentlayer->GetCarBodyCheck(); }

  void Framework::SetCarBodyCheck(bool bCheck)
  {
    m_Segmentlayer->SetCarBodyCheck(bCheck);
  }

  void Framework::SetVizRoadRight(bool on)
  {
    m_Segmentlayer->SetVizRoadRight(on);
  }

  bool Framework::GetVizRoadRight()
  {
    return m_Segmentlayer->GetVizRoadRight();
  }

  void Framework::SetShowDirectionOverlay(bool on)
  {
    m_Segmentlayer->SetShowDirectionOverlay(on);
  }

  bool Framework::GetShowDirectionOverlay()
  {
    return m_Segmentlayer->GetShowDirectionOverlay();
  }

  void Framework::SetLineWidth(double x) { m_Segmentlayer->SetLineWidth(x); }

  void BackgroundCull(double minX, double minY, double maxX, double maxY)
  {
    double dminX = gridSize * (int(minX / gridSize) - 1);
    double dminY = gridSize * (int(minY / gridSize) - 1);

    double dmaxX = gridSize * (int(maxX / gridSize) + 1);
    double dmaxY = gridSize * (int(maxY / gridSize) + 1);

    if (m_dminX == dminX && m_dminY == dminY && m_dmaxX == dmaxX &&
        m_dmaxY == dmaxY && m_gridSize == gridSize)
    {
      return;
    }
    else
    {
      m_dminX = dminX;
      m_dminY = dminY;
      m_dmaxX = dmaxX;
      m_dmaxY = dmaxY;
      m_gridSize = gridSize;
    }

    std::vector<Point3d> m_lineSeg[2];
    for (double i = dminX; i <= dmaxX; i += gridSize)
    {
      Point3d pt1, pt2;
      pt1.x = i;
      pt1.y = dminY;
      pt2.x = i;
      pt2.y = dmaxY;

      m_lineSeg[0].push_back(pt1);
      m_lineSeg[0].push_back(pt2);
    }

    for (double j = dminY; j <= dmaxY; j += gridSize)
    {
      Point3d pt1, pt2;
      pt1.x = dminX;
      pt1.y = j;
      pt2.x = dmaxX;
      pt2.y = j;

      m_lineSeg[1].push_back(pt1);
      m_lineSeg[1].push_back(pt2);
    }

    //------------------------------------------

    double cntx = (minX + maxX) * 0.5;
    double cnty = (minY + maxY) * 0.5;

    if (m_pTransformNode == NULL)
    {
      m_pTransformNode = new PositionTransformNode();
    }
    m_pTransformNode->SetPosition(V3d(cntx, cnty, 0.0));

    m_pTransformNode->RemoveAllChild();
    for (int j = 0; j < 2; j++)
    {
      if (m_lineSeg[j].size() > 1)
      {
        LineDrawable *pLine = new LineDrawable();

        for (int i = 0; i < m_lineSeg[j].size(); i += 2)
        {
          Point3d p1, p2;

          p1 = m_lineSeg[j][i];
          p2 = m_lineSeg[j][i + 1];

          if ((i / 2) % 2 == 0)
          {
            // V3f(0.24, 0.24, 0.42));
            pLine->add(p1.x - cntx, p1.y - cnty, 0.0, V3f(0.2, 0.2, 0.2));
            pLine->add(p2.x - cntx, p2.y - cnty, 0.0, V3f(0.2, 0.2, 0.2));
          }
          else
          {
            pLine->add(p2.x - cntx, p2.y - cnty, 0.0, V3f(0.2, 0.2, 0.2));
            pLine->add(p1.x - cntx, p1.y - cnty, 0.0, V3f(0.2, 0.2, 0.2));
          }
        }

        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pLine);

        m_pTransformNode->AddChild(pRenderLeaf);
      }
    }
  }

  void UpdateGrid(const Matrix4x4f &svMatrix, Camera *pCamera)
  {
    if (m_pTransformNode != NULL)
    {
      const V3d &position = m_pTransformNode->GetPosition();

      for (int i = 0; i < m_pTransformNode->GetNumChildren(); i++)
      {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)m_pTransformNode->GetChild(i);

        // 生成平移矩阵
        V3d origin = pCamera->GetPostion();
        Viewport viewport = pCamera->GetViewport();

        V3d vecT = position - origin;
        Matrix4x4f mat = Matrix4x4f::MakeTrans(vecT[0], vecT[1], vecT[2]);

        Matrix4x4f mvMatrix = mat * svMatrix;
        Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(4);

        // 更新变换矩阵
        pRenderLeaf->SetRenderTechnique(pTechnique);
        pRenderLeaf->SetModelViewMatrix(&mvMatrix);
        pRenderLeaf->SetProjectionMatrix(&prjMatrix);
        pRenderLeaf->SetViewport(viewport);
      }
    }
  }

  double Framework::GetMeasureDistance() const { return dMeasureDistance; }

  void Framework::BeginMeasureDistance(double x, double y)
  {
    if (pDistaneLine == NULL)
    {
      dMeasureDistance = 0;
      pDistaneLine = new GeoPolyline();

      pDistaneLine->AppendVertex(Point3d(x, y, 0));
    }
    else
    {
      int nCount = pDistaneLine->GetVertexCount();
      if (hasValidCount < nCount)
      {
        pDistaneLine->MoveVertex(Point3d(x, y, 0), nCount - 1);
      }
      else
      {
        pDistaneLine->AppendVertex(Point3d(x, y, 0));
      }
    }

    hasValidCount = pDistaneLine->GetVertexCount();
  }

  void Framework::MoveMeasureDistance(double x, double y)
  {
    if (pDistaneLine != NULL)
    {
      int nCount = pDistaneLine->GetVertexCount();

      if (hasValidCount < nCount)
      {
        pDistaneLine->MoveVertex(Point3d(x, y, 0), nCount - 1);
      }
      else
      {
        pDistaneLine->AppendVertex(Point3d(x, y, 0));
      }

      // 计算长度
      double dSumDist = 0;
      if (nCount > 1)
      {
        Point3d startPt = pDistaneLine->GetVertex(0);
        for (int i = 1; i < nCount; i++)
        {
          const Point3d &endPt = pDistaneLine->GetVertex(i);

          double dist = sqrt((endPt.x - startPt.x) * (endPt.x - startPt.x) +
                             (endPt.y - startPt.y) * (endPt.y - startPt.y));
          dSumDist += dist;

          startPt = endPt;
        }
      }

      dMeasureDistance = dSumDist;
    }
  }

  void Framework::EndMeasureDistance()
  {
    if (pDistaneLine != NULL)
    {
      pDistaneLine->Resize(hasValidCount);

      delete pDistaneLine;
      pDistaneLine = NULL;
    }

    if (m_pDistaneNode != NULL)
    {
      delete m_pDistaneNode;
      m_pDistaneNode = NULL;
    }
  }

  void MeasureCull(double minX, double minY, double maxX, double maxY)
  {
    if (pDistaneLine != NULL && pDistaneLine->IsBoundDirty())
    {
      pDistaneLine->CalculateBoundBox();
      BoundBox3d bound = pDistaneLine->GetBound();

      V3d vCnt = bound.GetCenter();

      ///----------------------------
      Drawable *pLine = FactoryDrawable::CreateLineDrawable(
          pDistaneLine, vCnt, V4f(1.0, 0.0, 0.0, 1.0));
      if (!pLine)
      {
        return;
      }

      RenderLeaf *pRenderLeaf = new RenderLeaf();
      pRenderLeaf->SetDrawable(pLine);

      //---------------------------------

      if (m_pDistaneNode == NULL)
      {
        m_pDistaneNode = new PositionTransformNode();
      }
      else
      {
        m_pDistaneNode->RemoveAllChild();
      }
      m_pDistaneNode->SetPosition(vCnt);
      m_pDistaneNode->AddChild(pRenderLeaf);
    }
  }

  void UpdateMeasure(const Matrix4x4f &svMatrix, Camera *pCamera)
  {
    if (m_pDistaneNode != NULL)
    {
      const V3d &position = m_pDistaneNode->GetPosition();

      for (int i = 0; i < m_pDistaneNode->GetNumChildren(); i++)
      {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)m_pDistaneNode->GetChild(i);

        // 生成平移矩阵
        V3d origin = pCamera->GetPostion();
        Viewport viewport = pCamera->GetViewport();

        V3d vecT = position - origin;
        Matrix4x4f mat = Matrix4x4f::MakeTrans(vecT[0], vecT[1], vecT[2]);

        Matrix4x4f mvMatrix = mat * svMatrix;
        Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(4);

        // 更新变换矩阵
        pRenderLeaf->SetRenderTechnique(pTechnique);
        pRenderLeaf->SetViewport(viewport);
        pRenderLeaf->SetModelViewMatrix(&mvMatrix);
        pRenderLeaf->SetProjectionMatrix(&prjMatrix);
      }
    }
  }

  void Framework::Draw()
  {
    double minDx, minDy;
    double maxDx, maxDy;

    MousePointToCart((int)m_viewport.Width(), 0, maxDx, maxDy);
    MousePointToCart(0, (int)m_viewport.Height(), minDx, minDy);
    if (minDx > maxDx)
    {
      std::swap(minDx, maxDx);
    }
    if (minDy > maxDy)
    {
      std::swap(minDy, maxDy);
    }

    m_Segmentlayer->Cull(minDx, minDy, maxDx, maxDy);

    m_pdbLayer->Cull(minDx, minDy, maxDx, maxDy, m_fMapScale);
    m_areaLayer->Cull(minDx, minDy, maxDx, maxDy);
    m_pointLayer->Cull(minDx, minDy, maxDx, maxDy);

    m_boundlayer->Cull(minDx, minDy, maxDx, maxDy);
    m_trajectorylayer->Cull(minDx, minDy, maxDx, maxDy);
    m_joblayer->Cull(minDx, minDy, maxDx, maxDy);
    m_topologylayer->Cull(minDx, minDy, maxDx, maxDy);

    if (m_fMapScale > 0.01)
    {
      BackgroundCull(minDx, minDy, maxDx, maxDy);
    }
    MeasureCull(minDx, minDy, maxDx, maxDy);

    //---------------------------------------------------

    // 生成观察矩阵
    if (m_pCamera == &m_StereoCamera)
    {
      m_pCamera->SetViewMatrix(m_trackball.GetInverseMatrix());
    }
    Matrix4x4f matScale =
        Matrix4x4f::MakeScale(m_fMapScale, m_fMapScale, m_fMapScale);
    Matrix4x4f svMatrix = matScale * m_pCamera->GetViewMatrix();

    V3d postion = m_pCamera->GetPostion();

    m_areaLayer->Update(svMatrix, m_pCamera);
    m_pointLayer->Update(svMatrix, m_pCamera);
    m_Segmentlayer->Update(svMatrix, m_pCamera);
    m_pdbLayer->Update(svMatrix, m_pCamera);
    m_boundlayer->Update(svMatrix, m_pCamera);
    m_trajectorylayer->Update(svMatrix, m_pCamera);
    m_joblayer->Update(svMatrix, m_pCamera);
    m_topologylayer->Update(svMatrix, m_pCamera);

    //----------------------------------------------
    if (m_fMapScale > 0.07)
    {
      UpdateGrid(svMatrix, m_pCamera);
    }
    UpdateMeasure(svMatrix, m_pCamera);

    State *pState = m_rendinfo.GetState();
    pState->ApplyViewort(&m_viewport);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_fMapScale > 0.01)
    {
      if (m_pTransformNode != nullptr)
      {
        for (int i = 0; i < m_pTransformNode->GetNumChildren(); i++)
        {
          RenderLeaf *pRenderLeaf = (RenderLeaf *)m_pTransformNode->GetChild(i);
          if (pRenderLeaf != NULL)
          {
            pRenderLeaf->Render(m_rendinfo, NULL);
          }
        }
      }
    }

    m_areaLayer->Draw(m_rendinfo);
    m_joblayer->Draw(m_rendinfo);
    m_Segmentlayer->Draw(m_rendinfo);
    m_pdbLayer->Draw(m_rendinfo);
    m_pointLayer->Draw(m_rendinfo);
    m_boundlayer->Draw(m_rendinfo);
    m_topologylayer->Draw(m_rendinfo);
    m_trajectorylayer->Draw(m_rendinfo);

    if (m_pDistaneNode != NULL)
    {
      for (int i = 0; i < m_pDistaneNode->GetNumChildren(); i++)
      {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)m_pDistaneNode->GetChild(i);
        if (pRenderLeaf != NULL)
        {
          pRenderLeaf->Render(m_rendinfo, NULL);
        }
      }
    }
  }

  void Framework::UndoOperate()
  {
    if (!m_commands.empty())
    {
      IMapCommand *pCommand = m_commands.top();
      m_commands.pop();

      pCommand->Undo();
      m_histroys.push(pCommand);
    }
  }

  void Framework::RedoOperate()
  {
    if (!m_histroys.empty())
    {
      IMapCommand *pCommand = m_histroys.top();
      m_histroys.pop();

      pCommand->Execute();
      m_commands.push(pCommand);
    }
  }

  void Framework::Destroy()
  {
    for (int i = 0; i < layers_.size(); i++)
    {
      if (layers_[i] != NULL)
      {
        delete layers_[i];
      }
    }
    layers_.clear();
  }

  int Framework::SetRegionCurb()
  {
    std::vector<BoundSegment *> boundarys;
    std::vector<JobArea *> jobAreas;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        boundarys.push_back((BoundSegment *)pFeature);
      }
      else if (pFeature->GetType() == MapFeature::MFT_JOB_AREA)
      {
        jobAreas.push_back((JobArea *)pFeature);
      }
    }

    int total = 0;

    for (int x = 0; x < jobAreas.size(); x++)
    {
      JobArea *pJobArea = jobAreas[x];

      if (pJobArea->GetProperty()->areaType == 104)
      {
        for (int i = 0; i < boundarys.size(); i++)
        {
          pJobArea->AddAttachObject(boundarys[i]);
          total++;
        }
      } // end if
    }

    return total;
  }
  int Framework::CreateLaneByBoundary()
  {
    std::vector<BoundSegment *> boundarys;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        boundarys.push_back((BoundSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }
    if (boundarys.size() != 2)
    {
      LOG(WARNING) << "Please select  2 boundarys!";
      return -1;
    }
    CreateLaneByBoundary(boundarys[0], boundarys[1]);
    return 1;
  }

  LaneSegment *Framework::CreateLaneByBoundary(BoundSegment *b0,
                                               BoundSegment *b1)
  {
    auto *temp = b0->GetGeometry();
    auto *ori = b1->GetGeometry();
    if (ori->GetVertexCount() > temp->GetVertexCount())
    {
      std::swap(temp, ori);
    }
    auto *centerline = CreateGeoLine(geoline_type_);
    centerline->AppendVertex(
        Point3d::MiddlePoint(ori->GetStartVertex(), temp->GetStartVertex()));
    for (size_t i = 1; i < ori->GetVertexCount() - 1; ++i)
    {
      Point3d p = ori->GetVertex(i);
      Point3d out;
      if (temp->GetNeartPoint(p, out) != 0)
      {
        out.x = p.x * 0.5 + out.x * 0.5;
        out.y = p.y * 0.5 + out.y * 0.5;
        out.z = p.z * 0.5 + out.z * 0.5;
        centerline->AppendVertex(out);
      }
    }
    centerline->AppendVertex(
        Point3d::MiddlePoint(ori->GetEndVertex(), temp->GetEndVertex()));

    LaneSegment *lane = new LaneSegment();
    lane->SetGeometry(centerline);
    m_Segmentlayer->AddMapFeature(lane);

    SegmentProperty *pProperty = lane->GetProperty();
    if (IsRightBoundary(b0->GetGeometry(), b1->GetGeometry()))
    {
      // pProperty->rightBoundary = boundarys[0]->GetUniqueID();
      // pProperty->leftBoundary = boundarys[1]->GetUniqueID();
      lane->SetLeftBoundary(b0);
      lane->SetRightBoundary(b1);
    }
    else
    {
      // pProperty->rightBoundary = boundarys[1]->GetUniqueID();
      // pProperty->leftBoundary = boundarys[0]->GetUniqueID();
      lane->SetLeftBoundary(b1);
      lane->SetRightBoundary(b0);
    }

    return lane;
  }

  int Framework::CreateReverseLaneGroup()
  {
    std::vector<BoundSegment *> boundarys;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        boundarys.push_back((BoundSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }
    if (boundarys.size() != 2)
    {
      LOG(WARNING) << "Please select  2 boundarys!";
      return -1;
    }

    auto *temp = boundarys[0]->GetGeometry();
    auto *ori = boundarys[1]->GetGeometry();
    if (ori->GetVertexCount() > temp->GetVertexCount())
    {
      std::swap(temp, ori);
    }
    {
      int positive = 0;
      int negative = 0;
      for (int x = 0; x < temp->GetVertexCount(); x++)
      {
        Point3d point3d = temp->GetVertex(x);
        double nearPt = ori->GetNeartPoint(point3d);
        if (nearPt > 0.0)
        {
          positive++;
        }
        else if (nearPt < 0.0)
        {
          negative++;
        }
      }
      if (positive < negative)
        return 0;
    }
    {
      int positive = 0;
      int negative = 0;
      for (int x = 0; x < ori->GetVertexCount(); x++)
      {
        Point3d point3d = ori->GetVertex(x);
        double nearPt = temp->GetNeartPoint(point3d);
        if (nearPt > 0.0)
        {
          positive++;
        }
        else if (nearPt < 0.0)
        {
          negative++;
        }
      }
      if (positive < negative)
        return 0;
    }
    double dis = Point3d::Distance(ori->GetStartVertex(), temp->GetEndVertex());
    if (dis < 3)
      return 1;

    std::vector<Geometry *> bl(4);
    for (auto &b : bl)
      b = CreateGeoLine(geoline_type_);

    auto fi0 = [](const Point3d &pa, const Point3d &pb, float xs)
    {
      Point3d out;
      out.x = pa.x * xs + pb.x * (1 - xs);
      out.y = pa.y * xs + pb.y * (1 - xs);
      out.z = pa.z * xs + pb.z * (1 - xs);
      return out;
    };
    auto fi = [fi0, this](const Point3d &pa, const Point3d &pb,
                          std::vector<Geometry *> &bl)
    {
      float dis = Point3d::Distance(pa, pb);
      float dlt = w_middle_;
      float xs0 = dlt / dis;
      float xs1 = (dis * 0.5 - dlt * 0.5) / dis;
      float xs2 = (dis * 0.5 + dlt * 0.5) * 0.5 / dis;
      bl[0]->AppendVertex(fi0(pa, pb, xs0));
      bl[1]->AppendVertex(fi0(pa, pb, xs1));
      bl[2]->AppendVertex(fi0(pa, pb, 1 - xs1));
      bl[3]->AppendVertex(fi0(pa, pb, 1 - xs0));
    };
    fi(ori->GetStartVertex(), temp->GetEndVertex(), bl);
    for (size_t i = 1; i < ori->GetVertexCount() - 1; ++i)
    {
      Point3d p = ori->GetVertex(i);
      Point3d out;
      if (temp->GetNeartPoint(p, out) != 0)
      {
        fi(p, out, bl);
      }
    }
    fi(ori->GetEndVertex(), temp->GetStartVertex(), bl);
    bl[0]->ReverseVertex();
    bl[1]->ReverseVertex();

    std::vector<BoundSegment *> bds;
    for (auto &b : bl)
    {
      auto *bd = new BoundSegment();
      bd->SetGeometry(b);
      m_boundlayer->AddMapFeature(bd);
      bds.push_back(bd);
    }
    std::vector<LaneSegment *> lanes(2);
    lanes[0] = CreateLaneByBoundary(bds[0], bds[1]);
    lanes[1] = CreateLaneByBoundary(bds[2], bds[3]);
    m_Segmentlayer->SetReverseSegment(lanes);
    boundarys[0]->GetProperty()->boundType = 3;
    boundarys[1]->GetProperty()->boundType = 3;
    bds[1]->GetProperty()->boundType = 2;
    bds[2]->GetProperty()->boundType = 2;
    return 1;
  }

  int Framework::CreateReverseLaneGroup2()
  {
    std::vector<BoundSegment *> boundarys;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_BOUNDARY)
      {
        boundarys.push_back((BoundSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }
    if (boundarys.size() != 1)
    {
      LOG(WARNING) << "Please select  1 boundary!";
      return -1;
    }

    auto *ori = boundarys[0]->GetGeometry();

    std::vector<Geometry *> bl(4);
    for (auto &b : bl)
      b = CreateGeoLine(geoline_type_);
    float wm = w_middle_ * 0.5;
    float wl = w_lane_ + wm;
    if (ori->CopyGeometry(wm, bl[0]) < 0)
      return 0;
    if (ori->CopyGeometry(wl, bl[1]) < 0)
      return 0;
    if (ori->CopyGeometry(-wm, bl[2]) < 0)
      return 0;
    if (ori->CopyGeometry(-wl, bl[3]) < 0)
      return 0;
    bl[0]->ReverseVertex();
    bl[1]->ReverseVertex();
    std::vector<BoundSegment *> bds;
    for (auto &b : bl)
    {
      auto *bd = new BoundSegment();
      bd->SetGeometry(b);
      m_boundlayer->AddMapFeature(bd);
      bds.push_back(bd);
    }
    std::vector<LaneSegment *> lanes(2);
    lanes[0] = CreateLaneByBoundary(bds[0], bds[1]);
    lanes[1] = CreateLaneByBoundary(bds[2], bds[3]);
    m_Segmentlayer->SetReverseSegment(lanes);

    bds[0]->GetProperty()->boundType = 2;
    bds[2]->GetProperty()->boundType = 2;
    return 1;
  }

  int Framework::CreateBoundaryByLane()
  {
    std::vector<LaneSegment *> lanes;

    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];

      if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        lanes.push_back((LaneSegment *)pFeature);
      }
      else
      {
        return -1;
      }
    }
    if (lanes.size() != 1)
    {
      LOG(WARNING) << "Please select  1 lane!";
      return -1;
    }

    auto *ori = lanes[0]->GetGeometry();

    std::vector<Geometry *> bl(2);
    for (auto &b : bl)
      b = CreateGeoLine(geoline_type_);
    float wl = w_lane_;
    if (ori->CopyGeometry(wl, bl[0]) < 0)
      return 0;
    if (ori->CopyGeometry(-wl, bl[1]) < 0)
      return 0;
    std::vector<BoundSegment *> bds;
    for (auto &b : bl)
    {
      auto *bd = new BoundSegment();
      bd->SetGeometry(b);
      m_boundlayer->AddMapFeature(bd);
      bds.push_back(bd);
    }
    lanes[0]->SetLeftBoundary(bds[0]);
    lanes[0]->SetRightBoundary(bds[1]);

    return 1;
  }
  int Framework::ReverseObj()
  {
    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];
      // auto t = pFeature->GetGeometry()->GetGeometryType();
      // if (t == Geometry::GT_POLYLINE)
      (pFeature->GetGeometry())->ReverseVertex();
    }
    return 0;
  }
  int Framework::CheckRelation()
  {
    if (m_selectedSet.empty())
    {
      m_Segmentlayer->CheckDeletedRealtion();
      return 0;
    }
    std::vector<LaneSegment *> laneSegments;
    for (int i = 0; i < m_selectedSet.size(); i++)
    {
      MapFeature *pFeature = m_selectedSet[i];
      if (pFeature->GetType() == MapFeature::MFT_LANE_SEG)
      {
        laneSegments.push_back((LaneSegment *)pFeature);
      }
    }
    for (auto &lane : laneSegments)
    {
      lane->CheckDeletedRealtion();
    }
    return laneSegments.size();
  }

  void Framework::ChangeLineType(int type)
  {
    if (type == 1)
      geoline_type_ = Geometry::GeometryType::GT_BSPLINE_CURVE;
    else if (type == 2)
      geoline_type_ = Geometry::GeometryType::GT_ARC_LINE;
    else if (type == 3)
      geoline_type_ = Geometry::GeometryType::GT_BEZIER_CURVE;
    else
      geoline_type_ = Geometry::GeometryType::GT_POLYLINE;
  }

  // 👇 先定义辅助函数（放在 framework.cc 文件顶部或 FillPointsOnSelectedBoundary 之前）
  static Point3d ProjectPointOnLine(const Point3d &a, const Point3d &b, const Point3d &p)
  {
    Point3d ab = Point3d(b.x - a.x, b.y - a.y, b.z - a.z);
    Point3d ap = Point3d(p.x - a.x, p.y - a.y, p.z - a.z);
    double len2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
    if (len2 < 1e-12)
    {
      return a;
    }
    double t = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / len2;
    t = std::max(0.0, std::min(1.0, t));
    return Point3d(
        a.x + t * ab.x,
        a.y + t * ab.y,
        a.z + t * ab.z);
  }

  // =============================================
  // ✅ FillPointsOnSelectedBoundary 完整实现
  // =============================================
  void Framework::FillPointsOnSelectedBoundary(double interval)
  {
    if (interval <= 0)
      return;

    // 只处理单一边界
    BoundSegment *pBoundary = nullptr;
    int count = 0;
    for (auto *feat : m_selectedSet)
    {
      if (feat && feat->GetType() == MapFeature::MFT_BOUNDARY)
      {
        pBoundary = static_cast<BoundSegment *>(feat);
        count++;
      }
    }
    if (count != 1 || !pBoundary)
    {
      LOG(WARNING) << "Please select exactly one boundary.";
      return;
    }

    Geometry *pGeom = pBoundary->GetGeometry();
    if (!pGeom || pGeom->GetVertexCount() < 2)
    {
      LOG(WARNING) << "Invalid boundary geometry.";
      return;
    }

    std::vector<Point3d> origVertices;
    for (int i = 0; i < pGeom->GetVertexCount(); ++i)
    {
      origVertices.push_back(pGeom->GetVertex(i));
    }

    // 计算累计距离
    std::vector<double> cumDist(1, 0.0);
    double totalLen = 0.0;
    for (size_t i = 1; i < origVertices.size(); ++i)
    {
      totalLen += Point3d::Distance(origVertices[i - 1], origVertices[i]);
      cumDist.push_back(totalLen);
    }
    if (totalLen < 1e-6)
      return;

    // 生成计划点（按路径顺序）
    struct InsertPlan
    {
      int segIdx;
      double pathDist; // 用于排序
      Point3d pt;
    };
    std::vector<InsertPlan> plans;

    int n = static_cast<int>(totalLen / interval);
    for (int i = 1; i <= n; ++i)
    {
      double target = i * interval;
      if (target >= totalLen - 1e-5)
        break;

      size_t seg = 0;
      for (size_t j = 0; j < cumDist.size() - 1; ++j)
      {
        if (target <= cumDist[j + 1])
        {
          seg = j;
          break;
        }
      }

      const Point3d &A = origVertices[seg];
      const Point3d &B = origVertices[seg + 1];
      double segLen = cumDist[seg + 1] - cumDist[seg];
      double t = (segLen < 1e-12) ? 0.0 : (target - cumDist[seg]) / segLen;
      t = std::clamp(t, 0.0, 1.0);

      // ⭐ 关键1: 跳过端点附近
      if (t < 1e-5 || t > 1.0 - 1e-5)
        continue;

      Point3d pt(
          A.x + t * (B.x - A.x),
          A.y + t * (B.y - A.y),
          A.z + t * (B.z - A.z));

      // ⭐ 关键2: 跳过与原始顶点重合的点
      bool tooCloseToVertex = false;
      for (const auto &v : origVertices)
      {
        if (Point3d::Distance(pt, v) < 1e-3)
        {
          tooCloseToVertex = true;
          break;
        }
      }
      if (tooCloseToVertex)
        continue;

      plans.push_back({static_cast<int>(seg), target, pt});
    }

    if (plans.empty())
    {
      LOG(INFO) << "No valid points to insert.";
      return;
    }

    // 按路径距离排序（从前到后）
    std::sort(plans.begin(), plans.end(),
              [](const InsertPlan &a, const InsertPlan &b)
              {
                return a.pathDist < b.pathDist;
              });

    // ⭐ 关键3: 去除重复点（包括与起点重复）
    std::vector<InsertPlan> uniquePlans;
    Point3d lastPt = origVertices.front();
    for (const auto &p : plans)
    {
      if (Point3d::Distance(lastPt, p.pt) > 1e-3)
      {
        uniquePlans.push_back(p);
        lastPt = p.pt;
      }
    }

    if (uniquePlans.empty())
    {
      LOG(INFO) << "All points filtered out.";
      return;
    }

    // 转为插入顺序：按 (segIdx desc, 路径距离 desc)
    std::vector<std::pair<int, Point3d>> insertions;
    for (const auto &p : uniquePlans)
    {
      insertions.emplace_back(p.segIdx, p.pt);
    }
    std::sort(insertions.begin(), insertions.end(),
              [](const auto &a, const auto &b)
              {
                if (a.first != b.first)
                  return a.first > b.first;
                // 同一段内，按路径距离降序（即从后往前）
                // 这里简化：因为 uniquePlans 已按路径升序，反向遍历即可
                return false; // 我们将反向遍历
              });

    // 清除选择
    for (auto *f : m_selectedSet)
      f->SetSelectedState(false);
    m_selectedSet.clear();

    // ⭐ 从后往前插入
    for (auto it = insertions.rbegin(); it != insertions.rend(); ++it)
    {
      const auto &[segIdx, pt] = *it;

      // ⭐ 最终保险：再次检查不与端点重合
      const Point3d &A = origVertices[segIdx];
      const Point3d &B = origVertices[segIdx + 1];
      if (Point3d::Distance(pt, A) < 1e-3 || Point3d::Distance(pt, B) < 1e-3)
      {
        continue;
      }

      InsertPointCommand *cmd = new InsertPointCommand(pBoundary, segIdx, pt);
      cmd->Execute();
      m_commands.push(cmd);
    }

    pBoundary->SetSelectedState(true);
    m_selectedSet.push_back(pBoundary);

    LOG(INFO) << "Inserted " << uniquePlans.size() << " points.";
  }

  // 点到折线的最近距离
  static double PointToPolylineMinDist(const Point3d &p,
                                       const std::vector<Point3d> &pts)
  {
    double minD = std::numeric_limits<double>::max();
    for (const auto &q : pts)
    {
      double d = Point3d::Distance(p, q);
      if (d < minD)
        minD = d;
    }
    return minD;
  }

  // 计算折线的主方向向量（首点→尾点单位向量）
  static std::pair<double, double> PolylineDirection(
      const std::vector<Point3d> &pts)
  {
    if (pts.size() < 2)
      return {0.0, 0.0};
    double dx = pts.back().x - pts.front().x;
    double dy = pts.back().y - pts.front().y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6)
      return {0.0, 0.0};
    return {dx / len, dy / len};
  }

  // 方向一致性：返回 cos(夹角)，<0 表示反向
  static double DirectionConsistency(const std::vector<Point3d> &a,
                                     const std::vector<Point3d> &b)
  {
    auto [ax, ay] = PolylineDirection(a);
    auto [bx, by] = PolylineDirection(b);
    return ax * bx + ay * by;
  }

  // 匹配分数：双向覆盖率（值越高越匹配，范围 0~1）
  // 思路：中心线采样点有多大比例落在轨迹的"走廊"内，
  //       同时轨迹采样点有多大比例落在中心线走廊内，取调和均值。
  // distTol：走廊宽度容忍（米），设为车道宽度量级即可，对绝对偏移不敏感
  static double CoverageScore(const std::vector<Point3d> &lanePts,
                              const std::vector<Point3d> &trajPts,
                              double distTol)
  {
    if (lanePts.empty() || trajPts.empty())
      return 0.0;

    // 中心线采样点覆盖率：落在轨迹走廊内的比例
    int laneHit = 0;
    for (const auto &p : lanePts)
      if (PointToPolylineMinDist(p, trajPts) <= distTol)
        ++laneHit;
    double laneCov = (double)laneHit / lanePts.size();

    // 轨迹采样点覆盖率：落在中心线走廊内的比例
    // 轨迹比中心线长时只看与中心线重叠部分，避免轨迹延伸段拉低分数
    int trajHit = 0;
    for (const auto &p : trajPts)
      if (PointToPolylineMinDist(p, lanePts) <= distTol)
        ++trajHit;
    double trajCov = (double)trajHit / trajPts.size();

    // 调和均值：两端都要覆盖才能得高分
    if (laneCov + trajCov < 1e-6)
      return 0.0;
    return 2.0 * laneCov * trajCov / (laneCov + trajCov);
  }

  int Framework::AutoBindTrajectory(double distTol, double coverageThreshold,
                                    bool dryRun, AutoBindDiagnostic *diag)
  {
    auto lanes = m_Segmentlayer->GetAllLane();

    std::ostringstream detail;

    if (diag)
    {
      diag->totalLanes = (int)lanes.size();
      diag->unboundLanes = 0;
      diag->refTotal = 0;
      diag->refFromTrajectoryLayer = 0;
      diag->refFromBoundaryLayer = 0;
      diag->refFromSegmentLayer = 0;
      diag->bound = 0;
      diag->detail.clear();
    }

    if (lanes.empty())
    {
      LOG(WARNING) << "AutoBindTrajectory: no lanes loaded.";
      detail << "[错误] 当前没有任何中心线(车道)，请先绘制或导入中心线。\n";
      if (diag)
        diag->detail = detail.str();
      return 0;
    }

    struct TrajInfo
    {
      std::vector<Point3d> pts;
      std::vector<Point3d> sampledPts;
      int mineIndex;
      char mineCode[64];
      const char *source;
    };
    std::vector<TrajInfo> trajInfos;

    auto collectPts = [](const std::vector<Point3d> &pts, int maxN)
        -> std::vector<Point3d>
    {
      if ((int)pts.size() <= maxN)
        return pts;
      std::vector<Point3d> out;
      out.reserve(maxN);
      size_t step = pts.size() / maxN;
      if (step == 0)
        step = 1;
      for (size_t i = 0; i < pts.size(); i += step)
        out.push_back(pts[i]);
      return out;
    };

    auto addBoundSeg = [&](BoundSegment *seg, const char *source) -> bool
    {
      auto *prop = seg->GetProperty();
      if (prop->mineSegmentIndex <= 0 && strlen(prop->mineSegmentCode) == 0)
        return false;
      TrajInfo ti;
      seg->GetGeometry()->Hermite(ti.pts);
      if (ti.pts.empty())
        return false;
      ti.sampledPts = collectPts(ti.pts, 100);
      ti.mineIndex = prop->mineSegmentIndex;
      std::strncpy(ti.mineCode, prop->mineSegmentCode, sizeof(ti.mineCode) - 1);
      ti.mineCode[sizeof(ti.mineCode) - 1] = '\0';
      ti.source = source;
      trajInfos.push_back(std::move(ti));
      return true;
    };

    int trajLayerCnt = 0;
    for (auto *seg : m_trajectorylayer->GetAllTrajectory())
      if (addBoundSeg(seg, "TrajectoryLayer"))
        ++trajLayerCnt;
    int boundLayerCnt = 0;
    for (auto *seg : m_boundlayer->GetAllBoundary())
      if (addBoundSeg(seg, "BoundaryLayer"))
        ++boundLayerCnt;
    int segLayerCnt = 0;
    for (auto *lane : lanes)
    {
      auto *prop = lane->GetProperty();
      if (prop->mineSegmentIndex <= 0)
        continue;
      TrajInfo ti;
      lane->GetGeometry()->Hermite(ti.pts);
      if (ti.pts.empty())
        continue;
      ti.sampledPts = collectPts(ti.pts, 100);
      ti.mineIndex = prop->mineSegmentIndex;
      std::strncpy(ti.mineCode, prop->mineSegmentCode, sizeof(ti.mineCode) - 1);
      ti.mineCode[sizeof(ti.mineCode) - 1] = '\0';
      ti.source = "SegmentLayer";
      trajInfos.push_back(std::move(ti));
      ++segLayerCnt;
    }

    int unboundCnt = 0;
    for (auto *lane : lanes)
      if (lane->GetProperty()->mineSegmentIndex <= 0)
        ++unboundCnt;

    if (diag)
    {
      diag->refTotal = (int)trajInfos.size();
      diag->refFromTrajectoryLayer = trajLayerCnt;
      diag->refFromBoundaryLayer = boundLayerCnt;
      diag->refFromSegmentLayer = segLayerCnt;
      diag->unboundLanes = unboundCnt;
    }

    detail << "参数: distTol=" << distTol
           << "m, coverageThreshold=" << coverageThreshold
           << (dryRun ? ", 模式=预览" : ", 模式=执行") << "\n";
    detail << "中心线总数=" << lanes.size() << ", 未绑定=" << unboundCnt << "\n";
    detail << "参考轨迹 总数=" << trajInfos.size()
           << " (TrajectoryLayer=" << trajLayerCnt
           << ", BoundaryLayer=" << boundLayerCnt
           << ", SegmentLayer=" << segLayerCnt << ")\n";

    if (trajInfos.empty())
    {
      LOG(WARNING) << "AutoBindTrajectory: no reference trajectories found.";
      detail << "[错误] 未收集到任何带有 mineSegmentIndex 的参考轨迹。\n"
                "       请确认 mapping.txt 已正确导入 (TrajectoryLayer 路径 "
                "或 BoundaryLayer 路径)。\n";
      if (diag)
        diag->detail = detail.str();
      return 0;
    }

    LOG(INFO) << "AutoBindTrajectory: " << trajInfos.size()
              << " reference trajectories, " << lanes.size() << " lanes.";

    int bindCount = 0;
    detail << "--- 每条未绑定中心线的最佳匹配 ---\n";
    for (auto *lane : lanes)
    {
      auto *prop = lane->GetProperty();
      if (prop->mineSegmentIndex > 0)
        continue;

      std::vector<Point3d> lanePts;
      lane->GetGeometry()->Hermite(lanePts);
      if (lanePts.empty())
        continue;
      auto laneSampled = collectPts(lanePts, 50);

      double bestScore = -1.0;
      int bestIdx = -1;
      double bestCos = 0.0;

      for (size_t ti = 0; ti < trajInfos.size(); ++ti)
      {
        double cosA = DirectionConsistency(lanePts, trajInfos[ti].pts);
        if (cosA < 0.0)
          continue;

        double score =
            CoverageScore(laneSampled, trajInfos[ti].sampledPts, distTol);
        if (score > bestScore)
        {
          bestScore = score;
          bestIdx = (int)ti;
          bestCos = cosA;
        }
      }

      detail << "lane " << lane->GetUniqueID();
      if (bestIdx < 0)
      {
        detail << " 无任何同向候选 (跳过)\n";
        continue;
      }
      const TrajInfo &best = trajInfos[bestIdx];
      detail << " best=[" << best.mineCode << "] idx=" << best.mineIndex
             << " src=" << best.source << " 覆盖=" << std::fixed
             << std::setprecision(3) << bestScore
             << " cos=" << std::setprecision(2) << bestCos;

      if (bestScore < coverageThreshold)
      {
        detail << " < 阈值 跳过\n";
        continue;
      }

      if (!dryRun)
      {
        prop->mineSegmentIndex = best.mineIndex;
        std::strncpy(prop->mineSegmentCode, best.mineCode,
                     sizeof(prop->mineSegmentCode) - 1);
        prop->mineSegmentCode[sizeof(prop->mineSegmentCode) - 1] = '\0';
        detail << " [已绑定]\n";
      }
      else
      {
        detail << " [可绑定-预览]\n";
      }
      ++bindCount;
    }

    detail << "--- 完成: " << (dryRun ? "可绑定" : "已绑定") << " " << bindCount
           << "/" << unboundCnt << " 条 ---\n";

    if (diag)
    {
      diag->bound = bindCount;
      diag->detail = detail.str();
    }

    LOG(INFO) << "AutoBindTrajectory: " << (dryRun ? "would bind " : "bound ")
              << bindCount << "/" << unboundCnt << " lanes.";
    return bindCount;
  }

  // ================== 快捷功能实现 ==================

  bool Framework::PickTrajectoryAnchor(double x, double y, double tolerance,
                                       BoundSegment *&segOut,
                                       int &vertexIdxOut)
  {
    segOut = nullptr;
    vertexIdxOut = -1;
    double bestDist = std::numeric_limits<double>::max();
    if (!m_trajectorylayer)
      return false;
    const auto &trajs = m_trajectorylayer->GetAllTrajectory();
    for (auto *seg : trajs)
    {
      auto *geo = seg->GetGeometry();
      int n = geo->GetVertexCount();
      for (int i = 0; i < n; ++i)
      {
        Point3d v = geo->GetVertex(i);
        double dx = v.x - x;
        double dy = v.y - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < bestDist)
        {
          bestDist = d2;
          segOut = seg;
          vertexIdxOut = i;
        }
      }
    }
    if (segOut && std::sqrt(bestDist) <= tolerance)
    {
      return true;
    }
    segOut = nullptr;
    vertexIdxOut = -1;
    return false;
  }

  LaneSegment *Framework::GenerateCenterlineFromAnchors(BoundSegment *traj,
                                                        int idxStart,
                                                        int idxEnd)
  {
    if (!traj)
      return nullptr;
    auto *geo = traj->GetGeometry();
    if (!geo)
      return nullptr;
    int n = geo->GetVertexCount();
    if (n < 2)
      return nullptr;
    idxStart = std::max(0, std::min(idxStart, n - 1));
    idxEnd = std::max(0, std::min(idxEnd, n - 1));
    if (idxStart == idxEnd)
      return nullptr;

    auto *centerline = CreateGeoLine(Geometry::GT_POLYLINE);
    int step = (idxStart < idxEnd) ? 1 : -1;
    for (int i = idxStart;; i += step)
    {
      centerline->AppendVertex(geo->GetVertex(i));
      if (i == idxEnd)
        break;
    }

    LaneSegment *lane = new LaneSegment();
    lane->SetGeometry(centerline);
    m_Segmentlayer->AddMapFeature(lane);

    SegmentProperty *prop = lane->GetProperty();
    BoundaryProperty *tprop = traj->GetProperty();
    prop->mineSegmentIndex = tprop->mineSegmentIndex;
    std::strncpy(prop->mineSegmentCode, tprop->mineSegmentCode,
                 sizeof(prop->mineSegmentCode) - 1);
    prop->mineSegmentCode[sizeof(prop->mineSegmentCode) - 1] = '\0';
    prop->road_right = 0;

    return lane;
  }

  int Framework::GenerateCenterlineChainByDirection(
      BoundSegment *segA, int idxA, BoundSegment *segB, int idxB, int direction,
      DirectionChainDiagnostic *diag)
  {
    if (diag)
    {
      diag->chainSegments = 0;
      diag->generatedLanes = 0;
      diag->detail.clear();
    }
    if (!segA || !segB)
      return 0;
    if (direction != 1 && direction != 2)
      return 0;
    if (!m_trajectorylayer)
      return 0;

    const auto &trajs = m_trajectorylayer->GetAllTrajectory();
    if (trajs.empty())
      return 0;

    // 轨迹端点相邻判据：端点间距 < kEndEps
    const double kEndEps = 2.0; // 米

    auto endpoint = [](BoundSegment *s, bool head) -> Point3d
    {
      auto *g = s->GetGeometry();
      int n = g->GetVertexCount();
      return g->GetVertex(head ? 0 : n - 1);
    };
    auto dist2 = [](const Point3d &a, const Point3d &b)
    {
      double dx = a.x - b.x, dy = a.y - b.y;
      return dx * dx + dy * dy;
    };

    // 简化情况：同一条轨迹，直接单段生成
    if (segA == segB)
    {
      if (idxA == idxB)
        return 0;
      LaneSegment *lane = GenerateCenterlineFromAnchors(segA, idxA, idxB);
      if (!lane)
        return 0;
      lane->GetProperty()->direction = direction;
      if (diag)
      {
        diag->chainSegments = 1;
        diag->generatedLanes = 1;
        std::ostringstream os;
        os << "单段生成: traj=" << segA->GetUniqueID() << " idx " << idxA
           << " -> " << idxB << " direction=" << direction;
        diag->detail = os.str();
      }
      return 1;
    }

    // 轨迹图 BFS：节点 = trajectory，边 = 两条轨迹端点距离 < kEndEps
    std::unordered_map<BoundSegment *, std::vector<BoundSegment *>> adj;
    for (auto *s : trajs)
      adj[s]; // 确保每个节点存在
    double eps2 = kEndEps * kEndEps;
    for (size_t i = 0; i < trajs.size(); ++i)
    {
      BoundSegment *a = trajs[i];
      Point3d ah = endpoint(a, true), at = endpoint(a, false);
      for (size_t j = i + 1; j < trajs.size(); ++j)
      {
        BoundSegment *b = trajs[j];
        Point3d bh = endpoint(b, true), bt = endpoint(b, false);
        if (dist2(ah, bh) < eps2 || dist2(ah, bt) < eps2 ||
            dist2(at, bh) < eps2 || dist2(at, bt) < eps2)
        {
          adj[a].push_back(b);
          adj[b].push_back(a);
        }
      }
    }

    // BFS segA -> segB
    std::queue<BoundSegment *> q;
    std::unordered_map<BoundSegment *, BoundSegment *> prev;
    prev[segA] = nullptr;
    q.push(segA);
    bool found = false;
    while (!q.empty())
    {
      BoundSegment *cur = q.front();
      q.pop();
      if (cur == segB)
      {
        found = true;
        break;
      }
      for (auto *nb : adj[cur])
      {
        if (prev.find(nb) == prev.end())
        {
          prev[nb] = cur;
          q.push(nb);
        }
      }
    }
    if (!found)
    {
      if (diag)
        diag->detail = "无法在轨迹图中找到连通路径";
      return 0;
    }

    std::vector<BoundSegment *> chain;
    for (BoundSegment *p = segB; p; p = prev[p])
      chain.push_back(p);
    std::reverse(chain.begin(), chain.end());
    if (diag)
      diag->chainSegments = (int)chain.size();

    std::ostringstream os;
    os << "轨迹链长度 = " << chain.size() << "\n";

    // 为链内每条轨迹生成完整"段级"中心线 + 入/出索引决策
    // - 链中 s0 = segA: 起点索引 = idxA; 终点索引 = 与下一段最近的端点(0 或 n-1)
    // - 链中 sk = segB: 起点索引 = 与上一段最近的端点; 终点索引 = idxB
    // - 中间段: 起点/终点索引分别靠近上一段/下一段
    auto nearestEndIdx = [&](BoundSegment *self, BoundSegment *neighbor)
    {
      auto *g = self->GetGeometry();
      int n = g->GetVertexCount();
      if (n < 1)
        return 0;
      Point3d head = g->GetVertex(0);
      Point3d tail = g->GetVertex(n - 1);
      double bestDist = std::numeric_limits<double>::max();
      int bestIdx = 0;
      auto *gn = neighbor->GetGeometry();
      int nn = gn->GetVertexCount();
      if (nn < 1)
        return 0;
      Point3d nh = gn->GetVertex(0);
      Point3d nt = gn->GetVertex(nn - 1);
      double dhh = dist2(head, nh), dht = dist2(head, nt);
      double dth = dist2(tail, nh), dtt = dist2(tail, nt);
      double headBest = std::min(dhh, dht);
      double tailBest = std::min(dth, dtt);
      if (headBest <= tailBest)
      {
        bestDist = headBest;
        bestIdx = 0;
      }
      else
      {
        bestDist = tailBest;
        bestIdx = n - 1;
      }
      (void)bestDist;
      return bestIdx;
    };

    std::vector<LaneSegment *> generated;
    for (size_t i = 0; i < chain.size(); ++i)
    {
      BoundSegment *s = chain[i];
      auto *g = s->GetGeometry();
      int n = g->GetVertexCount();
      if (n < 2)
        continue;
      int startIdx = 0, endIdx = n - 1;
      if (i == 0)
        startIdx = idxA;
      if (i == chain.size() - 1)
        endIdx = idxB;
      if (i > 0)
        startIdx = nearestEndIdx(s, chain[i - 1]);
      if (i + 1 < chain.size())
        endIdx = nearestEndIdx(s, chain[i + 1]);
      if (startIdx == endIdx)
      {
        os << "  skip seg " << s->GetUniqueID()
           << " (start==end idx=" << startIdx << ")\n";
        continue;
      }
      LaneSegment *lane = GenerateCenterlineFromAnchors(s, startIdx, endIdx);
      if (!lane)
        continue;
      lane->GetProperty()->direction = direction;
      generated.push_back(lane);
      os << "  seg " << s->GetUniqueID() << " [" << startIdx << "->" << endIdx
         << "] -> lane " << lane->GetUniqueID() << "\n";
    }

    // 串联前驱/后继
    for (size_t i = 0; i + 1 < generated.size(); ++i)
    {
      generated[i]->AddSuccessorSegment(generated[i + 1]);
      generated[i + 1]->AddPredecessorSegment(generated[i]);
    }

    // 接缝点对齐：相邻中心线首尾顶点改为同一个点(取中点)，保证物理连通
    for (size_t i = 0; i + 1 < generated.size(); ++i)
    {
      Geometry *ga = generated[i]->GetGeometry();
      Geometry *gb = generated[i + 1]->GetGeometry();
      if (!ga || !gb)
        continue;
      int na = ga->GetVertexCount();
      int nb = gb->GetVertexCount();
      if (na < 1 || nb < 1)
        continue;
      Point3d pa = ga->GetVertex(na - 1);
      Point3d pb = gb->GetVertex(0);
      Point3d mid;
      mid.x = 0.5 * (pa.x + pb.x);
      mid.y = 0.5 * (pa.y + pb.y);
      mid.z = 0.5 * (pa.z + pb.z);
      ga->MoveVertex(mid, na - 1);
      gb->MoveVertex(mid, 0);
      ga->CalculateBoundBox();
      gb->CalculateBoundBox();
    }
    // 强制所有新生成车道重渲染，保证方向高亮色带覆盖
    for (auto *ln : generated)
      ln->SetChanged(true);

    if (diag)
    {
      diag->generatedLanes = (int)generated.size();
      diag->detail = os.str();
    }
    return (int)generated.size();
  }

  int Framework::AutoCheckRoadRight(double threshold,
                                    RoadRightDiagnostic *diag)
  {
    auto lanes = m_Segmentlayer->GetAllLane();
    std::ostringstream detail;
    if (diag)
    {
      diag->totalLanes = (int)lanes.size();
      diag->compliant = 0;
      diag->nonCompliant = 0;
      diag->missingBoundary = 0;
      diag->detail.clear();
    }
    detail << "阈值 A = " << threshold << " 米\n";
    detail << "中心线总数 = " << lanes.size() << "\n";
    detail << "--- 每条中心线判定 ---\n";

    auto minDistToPolyline = [](const Point3d &p, Geometry *g)
    {
      double best = std::numeric_limits<double>::max();
      if (!g)
        return best;
      int m = g->GetVertexCount();
      for (int j = 0; j < m - 1; ++j)
      {
        Point3d a = g->GetVertex(j);
        Point3d b = g->GetVertex(j + 1);
        double abx = b.x - a.x, aby = b.y - a.y;
        double len2 = abx * abx + aby * aby;
        double t = 0.0;
        if (len2 > 1e-12)
        {
          t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
          if (t < 0)
            t = 0;
          if (t > 1)
            t = 1;
        }
        double qx = a.x + t * abx;
        double qy = a.y + t * aby;
        double d2 = (p.x - qx) * (p.x - qx) + (p.y - qy) * (p.y - qy);
        if (d2 < best)
          best = d2;
      }
      return std::sqrt(best);
    };

    int updated = 0;
    for (auto *lane : lanes)
    {
      auto *lgeo = lane->GetGeometry();
      auto *lb = lane->GetLeftBoundary();
      auto *rb = lane->GetRightBoundary();
      detail << "lane " << lane->GetUniqueID();
      if (!lb || !rb)
      {
        if (diag)
          ++diag->missingBoundary;
        lane->GetProperty()->road_right = 0;
        ++updated;
        detail << " 缺少左/右边界 -> road_right=0\n";
        continue;
      }
      double worstLeft = std::numeric_limits<double>::max();
      double worstRight = std::numeric_limits<double>::max();
      int nv = lgeo->GetVertexCount();
      for (int i = 0; i < nv; ++i)
      {
        Point3d p = lgeo->GetVertex(i);
        double dl = minDistToPolyline(p, lb->GetGeometry());
        double dr = minDistToPolyline(p, rb->GetGeometry());
        if (dl < worstLeft)
          worstLeft = dl;
        if (dr < worstRight)
          worstRight = dr;
      }
      bool compliant = (worstLeft > threshold) && (worstRight > threshold);
      lane->GetProperty()->road_right = compliant ? 1 : 0;
      ++updated;
      detail << " minLeft=" << std::fixed << std::setprecision(2) << worstLeft
             << " minRight=" << worstRight << " -> road_right="
             << (compliant ? "1" : "0") << "\n";
      if (diag)
      {
        if (compliant)
          ++diag->compliant;
        else
          ++diag->nonCompliant;
      }
    }
    if (diag)
      diag->detail = detail.str();
    return updated;
  }

  int Framework::CheckLaneConnectivity(ConnectivityDiagnostic *diag)
  {
    auto lanes = m_Segmentlayer->GetAllLane();
    std::ostringstream detail;
    int starts = 0, ends = 0, middles = 0, isolated = 0;
    std::vector<int> isolatedIds;
    detail << "--- 连通性校验 ---\n";
    for (auto *lane : lanes)
    {
      std::vector<int> suc, pre;
      lane->GetSuccessorSegment(suc);
      lane->GetPredecessorSegment(pre);
      bool hasSuc = !suc.empty();
      bool hasPre = !pre.empty();
      const char *role = nullptr;
      if (hasSuc && hasPre)
      {
        ++middles;
        role = "中间";
      }
      else if (hasSuc && !hasPre)
      {
        ++starts;
        role = "起始";
      }
      else if (!hasSuc && hasPre)
      {
        ++ends;
        role = "终止";
      }
      else
      {
        ++isolated;
        isolatedIds.push_back(lane->GetUniqueID());
        role = "孤立(异常)";
      }
      detail << "lane " << lane->GetUniqueID() << " 前续=" << pre.size()
             << " 后继=" << suc.size() << " -> " << role << "\n";
    }
    detail << "--- 汇总 ---\n";
    detail << "总数=" << lanes.size() << "  起始=" << starts << "  终止=" << ends
           << "  中间=" << middles << "  孤立=" << isolated << "\n";
    if (isolated > 0)
    {
      detail << "⚠ 孤立车道ID: ";
      for (int id : isolatedIds)
        detail << id << " ";
      detail << "\n";
    }
    // 校验结果直接驱动渲染层。空结果同样写入，以清除上一次校验遗留的
    // 黄色蒙版；不依赖结果对话框是否仍然打开。
    m_Segmentlayer->SetConnectivityWarnings(isolatedIds);
    if (diag)
    {
      diag->totalLanes = (int)lanes.size();
      diag->starts = starts;
      diag->ends = ends;
      diag->middles = middles;
      diag->isolated = isolated;
      diag->isolatedIds = std::move(isolatedIds);
      diag->detail = detail.str();
    }
    return isolated;
  }

  bool Framework::ExportRoadRightTxt(const std::string &path,
                                     RoadRightIOStat *stat)
  {
    std::ofstream ofs(path);
    if (!ofs)
      return false;
    auto lanes = m_Segmentlayer->GetAllLane();
    std::unordered_map<std::string, int> seen; // code -> road_right
    std::ostringstream detail;
    int written = 0;
    for (auto *lane : lanes)
    {
      auto *p = lane->GetProperty();
      if (p->mineSegmentCode[0] == '\0')
        continue;
      std::string code = p->mineSegmentCode;
      auto it = seen.find(code);
      if (it == seen.end())
      {
        seen[code] = p->road_right;
      }
      else if (it->second != p->road_right)
      {
        detail << "⚠ 路段 " << code << " 存在不一致的 road_right: " << it->second
               << " vs " << p->road_right << "，按第一个为准\n";
      }
    }
    std::vector<std::pair<std::string, int>> aligned_rows(seen.begin(),
                                                          seen.end());
    std::sort(aligned_rows.begin(), aligned_rows.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    size_t code_width = std::string("mineSegmentCode").size();
    for (const auto &row : aligned_rows)
      code_width = std::max(code_width, row.first.size());

    // Keep the 0/1 value at one fixed column so large road lists remain easy
    // to inspect in a plain-text editor. Import remains whitespace-compatible.
    for (const auto &row : aligned_rows)
    {
      ofs << std::left << std::setw(static_cast<int>(code_width)) << row.first
          << "  " << row.second << "\n";
      ++written;
    }
    ofs.close();
    if (stat)
    {
      stat->written = written;
      stat->detail = detail.str() + "已写入 " + std::to_string(written) +
                     " 条路段到: " + path + "\n";
    }
    return true;
  }

  bool Framework::ImportRoadRightTxt(const std::string &path,
                                     RoadRightIOStat *stat)
  {
    std::ifstream ifs(path);
    if (!ifs)
      return false;
    std::unordered_map<std::string, int> table;
    std::string line;
    while (std::getline(ifs, line))
    {
      if (line.empty())
        continue;
      std::istringstream iss(line);
      std::string code;
      int rr = 0;
      if (!(iss >> code >> rr))
        continue;
      table[code] = rr ? 1 : 0;
    }
    auto lanes = m_Segmentlayer->GetAllLane();
    int matched = 0, unmatched = 0;
    std::ostringstream detail;
    for (auto *lane : lanes)
    {
      auto *p = lane->GetProperty();
      if (p->mineSegmentCode[0] == '\0')
      {
        ++unmatched;
        continue;
      }
      auto it = table.find(p->mineSegmentCode);
      if (it != table.end())
      {
        p->road_right = it->second;
        ++matched;
      }
      else
      {
        ++unmatched;
      }
    }
    detail << "读取文件: " << path << "\n";
    detail << "文件内路段数=" << table.size() << "  匹配=" << matched
           << "  未匹配中心线=" << unmatched << "\n";
    if (stat)
    {
      stat->matched = matched;
      stat->unmatched = unmatched;
      stat->detail = detail.str();
    }
    return true;
  }

  // ===== R3 + R4: 自适应边界生成 + 接缝 taper 平滑 =====
  namespace
  {

    // 返回多边线段 g 上距离 p 最近的点距离（平面 xy）
    double MinDistToPolylineFlat(const Point3d &p, Geometry *g)
    {
      double best = std::numeric_limits<double>::max();
      if (!g)
        return best;
      int m = g->GetVertexCount();
      for (int j = 0; j < m - 1; ++j)
      {
        Point3d a = g->GetVertex(j);
        Point3d b = g->GetVertex(j + 1);
        double abx = b.x - a.x, aby = b.y - a.y;
        double len2 = abx * abx + aby * aby;
        double t = 0.0;
        if (len2 > 1e-12)
        {
          t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
          if (t < 0)
            t = 0;
          if (t > 1)
            t = 1;
        }
        double qx = a.x + t * abx;
        double qy = a.y + t * aby;
        double d2 = (p.x - qx) * (p.x - qx) + (p.y - qy) * (p.y - qy);
        if (d2 < best)
          best = d2;
      }
      return std::sqrt(best);
    }

    // 单位方向向量
    inline Point3d UnitVec2(const Point3d &a, const Point3d &b, bool &ok)
    {
      Point3d v = b - a;
      double l = std::sqrt(v.x * v.x + v.y * v.y);
      ok = (l > 1e-9);
      if (ok)
      {
        v.x /= l;
        v.y /= l;
      }
      return v;
    }

    // 顶点局部左法向（单位向量）与曲率半径估计。
    // 切线使用"5 点加权中心差分"：在合法范围内取 P_{i-2..i+2}，以边长倒数做权重
    // 求得平滑切线；端点使用"镜像法"外推一个虚拟点，再用相同公式，避免一阶单
    // 边差分引起的切线跳变（末端外扩的根因）。
    struct LocalFrame
    {
      double lnx, lny; // 左单位法向
      double radius;   // 曲率半径估计（米），大值 ≈ 直线
    };
    LocalFrame ComputeLocalFrame(const std::vector<Point3d> &in, int i)
    {
      LocalFrame f{0.0, 1.0, 1e9};
      int n = (int)in.size();
      if (n < 2)
        return f;

      // 取 i-1, i+1 的位置；端点用镜像法 p_{-1}=2p_0-p_1, p_n=2p_{n-1}-p_{n-2}
      Point3d pm; // P_{i-1}
      Point3d pp; // P_{i+1}
      if (i - 1 >= 0)
      {
        pm = in[i - 1];
      }
      else
      {
        pm = Point3d(2 * in[0].x - in[1].x, 2 * in[0].y - in[1].y,
                     2 * in[0].z - in[1].z);
      }
      if (i + 1 < n)
      {
        pp = in[i + 1];
      }
      else
      {
        pp = Point3d(2 * in[n - 1].x - in[n - 2].x,
                     2 * in[n - 1].y - in[n - 2].y,
                     2 * in[n - 1].z - in[n - 2].z);
      }
      // 中心差分切线 t = (pp - pm) / |pp - pm|
      double tx = pp.x - pm.x;
      double ty = pp.y - pm.y;
      double L = std::sqrt(tx * tx + ty * ty);
      if (L > 1e-9)
      {
        tx /= L;
        ty /= L;
      }
      else
      {
        tx = 1;
        ty = 0;
      }

      // 曲率半径估计（仅对内部点做）
      if (i >= 1 && i + 1 < n)
      {
        bool ok1 = false, ok2 = false;
        Point3d v1 = UnitVec2(in[i - 1], in[i], ok1);
        Point3d v2 = UnitVec2(in[i], in[i + 1], ok2);
        if (ok1 && ok2)
        {
          double turn = std::sqrt((v2.x - v1.x) * (v2.x - v1.x) +
                                  (v2.y - v1.y) * (v2.y - v1.y));
          double dsA =
              std::sqrt((in[i].x - in[i - 1].x) * (in[i].x - in[i - 1].x) +
                        (in[i].y - in[i - 1].y) * (in[i].y - in[i - 1].y));
          double dsB =
              std::sqrt((in[i + 1].x - in[i].x) * (in[i + 1].x - in[i].x) +
                        (in[i + 1].y - in[i].y) * (in[i + 1].y - in[i].y));
          double ds = 0.5 * (dsA + dsB);
          if (turn > 1e-6 && ds > 1e-6)
          {
            f.radius = ds / turn;
          }
        }
      }

      // 左法向（tangent 顺时针 90°）
      f.lnx = -ty;
      f.lny = tx;
      return f;
    }

    // 沿方向 dir 从点 P 发射半直线，与 polyline pts 的各边求交点；返回最近交
    // 点到 P 的距离；无交点返回 -1
    // 这正是"当前中心线顶点往对向中心线做垂线的距离"所需的核心工具。
    double RayPolylineHit(const Point3d &P, double dx, double dy,
                          const std::vector<Point3d> &pts)
    {
      double best = -1.0;
      int m = (int)pts.size();
      for (int j = 0; j + 1 < m; ++j)
      {
        const Point3d &A = pts[j];
        const Point3d &B = pts[j + 1];
        double ex = B.x - A.x;
        double ey = B.y - A.y;
        // 求 P + t*(dx,dy) = A + s*(ex,ey)，其中 t>=0, 0<=s<=1
        // 行列式 D = dx*(-ey) - dy*(-ex) = -dx*ey + dy*ex = dy*ex - dx*ey
        double D = dy * ex - dx * ey;
        if (std::fabs(D) < 1e-12)
          continue; // 平行
        double rx = A.x - P.x;
        double ry = A.y - P.y;
        // t = (rx * (-ey) - ry * (-ex)) / D = (ry*ex - rx*ey) / D
        double t = (ry * ex - rx * ey) / D;
        if (t <= 1e-9)
          continue;
        // s = (rx * (-dy) - ry * (-dx)) / D = (ry*dx - rx*dy) / D
        double s = (ry * dx - rx * dy) / D;
        if (s < -1e-6 || s > 1.0 + 1e-6)
          continue;
        if (best < 0 || t < best)
          best = t;
      }
      return best;
    }

    // 接缝局部加密：保留输入所有顶点；只在接缝区域内、且段长 > kMinSegLen
    // 的边上插入一个中点。目标点数 ≈ 中心线点数的 1.0~1.5 倍。
    // hasStartSeam/hasEndSeam 控制哪一端是"接缝"（有前驱/后继），非接缝端不加密。
    void DensifySeamLocal(const std::vector<Point3d> &in,
                          std::vector<Point3d> &out, double seamZone,
                          bool hasStartSeam, bool hasEndSeam)
    {
      const double kMinSegLen = 5.0; // 仅长边在接缝区插中点
      int n = (int)in.size();
      out.clear();
      if (n == 0)
        return;
      std::vector<double> cum(n, 0.0);
      for (int i = 1; i < n; ++i)
      {
        double dx = in[i].x - in[i - 1].x;
        double dy = in[i].y - in[i - 1].y;
        cum[i] = cum[i - 1] + std::sqrt(dx * dx + dy * dy);
      }
      double total = cum.back();
      auto inSeam = [&](double s)
      {
        if (hasStartSeam && s <= seamZone)
          return true;
        if (hasEndSeam && (total - s) <= seamZone)
          return true;
        return false;
      };
      out.push_back(in[0]);
      for (int i = 0; i + 1 < n; ++i)
      {
        const Point3d &a = in[i];
        const Point3d &b = in[i + 1];
        double segLen = cum[i + 1] - cum[i];
        double midS = 0.5 * (cum[i] + cum[i + 1]);
        if (inSeam(midS) && segLen > kMinSegLen)
        {
          Point3d m(0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z));
          out.push_back(m);
        }
        out.push_back(b);
      }
    }

    // Hermite 平滑：对已算好的一维宽度序列 w 做一次 3 点均值平滑（保端点）
    void SmoothWidth(std::vector<double> &w, int iters = 2)
    {
      int n = (int)w.size();
      if (n < 3)
        return;
      std::vector<double> t(n);
      for (int k = 0; k < iters; ++k)
      {
        t[0] = w[0];
        t[n - 1] = w[n - 1];
        for (int i = 1; i < n - 1; ++i)
        {
          t[i] = 0.25 * w[i - 1] + 0.5 * w[i] + 0.25 * w[i + 1];
        }
        w.swap(t);
      }
    }

    // 在 polyline g 上求最近点投影：返回最近点、投影弧长 s（从起点计）及最近距离
    struct ProjResult
    {
      Point3d cp;
      double s = 0.0;          // 弧长参数
      double dist = 0.0;       // 最近距离
      double nx = 0.0;         // 在投影点处的切线 x
      double ny = 0.0;         // 在投影点处的切线 y
      bool atEndpoint = false; // 最近点是否为 polyline 的端点（t 被夹到 0 或 1 且发生在首/末段）
    };
    ProjResult ProjectOntoPolyline(const Point3d &p,
                                   const std::vector<Point3d> &pts)
    {
      ProjResult r;
      r.dist = std::numeric_limits<double>::max();
      int m = (int)pts.size();
      if (m < 2)
        return r;
      double cum = 0.0;
      int bestJ = -1;
      double bestT = 0.0;
      for (int j = 0; j + 1 < m; ++j)
      {
        const Point3d &a = pts[j];
        const Point3d &b = pts[j + 1];
        double abx = b.x - a.x, aby = b.y - a.y;
        double len2 = abx * abx + aby * aby;
        double segLen = std::sqrt(len2);
        double tRaw = 0.0;
        double t = 0.0;
        if (len2 > 1e-12)
        {
          tRaw = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
          t = tRaw;
          if (t < 0)
            t = 0;
          if (t > 1)
            t = 1;
        }
        Point3d q(a.x + t * abx, a.y + t * aby, a.z + t * (b.z - a.z));
        double dx = p.x - q.x, dy = p.y - q.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < r.dist)
        {
          r.dist = d;
          r.cp = q;
          r.s = cum + t * segLen;
          if (segLen > 1e-9)
          {
            r.nx = abx / segLen;
            r.ny = aby / segLen;
          }
          bestJ = j;
          bestT = t;
        }
        cum += segLen;
      }
      // 端点判定：投影落在首段 t=0 或末段 t=1
      if (bestJ == 0 && bestT <= 1e-9)
        r.atEndpoint = true;
      else if (bestJ == m - 2 && bestT >= 1.0 - 1e-9)
        r.atEndpoint = true;
      return r;
    }

  } // namespace

  int Framework::GenerateAdaptiveBoundaries(
      const AdaptiveBoundaryParams &params, AdaptiveBoundaryDiagnostic *diag)
  {
    return GenerateAdaptiveBoundariesForSelected(params, {}, diag);
  }

  int Framework::GenerateAdaptiveBoundariesForSelected(
      const AdaptiveBoundaryParams &params,
      const std::vector<LaneSegment *> &targetLanes,
      AdaptiveBoundaryDiagnostic *diag)
  {
    // targetLanes 为空 → 全局模式（处理所有车道）
    std::unordered_set<LaneSegment *> targetSet(targetLanes.begin(),
                                                targetLanes.end());
    bool globalMode = targetSet.empty();
    if (diag)
    {
      diag->totalLanes = 0;
      diag->processed = 0;
      diag->dualWide = 0;
      diag->singleWide = 0;
      diag->skipped = 0;
      diag->detail.clear();
    }
    if (!m_Segmentlayer)
      return 0;
    auto allLanes = m_Segmentlayer->GetAllLane();
    if (diag)
      diag->totalLanes = (int)allLanes.size();
    // 局部模式：参与计算的全集严格限定为 targetSet 内的中心线，
    // 这样对向匹配、shrink 迭代、接缝对齐都不会扩散到选中之外的车道，
    // 避免岔路口邻近车道相互干扰导致边界生成错乱。
    std::vector<LaneSegment *> lanes;
    if (globalMode)
    {
      lanes = allLanes;
    }
    else
    {
      lanes.reserve(targetSet.size());
      for (auto *l : allLanes)
      {
        if (targetSet.find(l) != targetSet.end())
          lanes.push_back(l);
      }
    }

    std::ostringstream os;
    os << std::fixed << std::setprecision(2);
    os << "参数(半宽语义): 双开最低=" << params.min_lane_width_dual
       << "m, 双开最高=" << params.max_lane_width_dual
       << "m, 单开最高=" << params.max_lane_width_single
       << "m, 边界间距=" << params.boundary_gap
       << "m, 平滑区间=" << params.taper_len << "m\n";

    // =========================================================
    // 术语：所有 "车道宽" = 中心线到边界的距离（=车道半宽）
    // =========================================================
    // 设对向 A(上山)、B(下山)，d = 两中心线/轨迹间横向距离。
    // 模式判定（以最低半宽为门槛，d/2 需 > min_lane_width_dual）：
    //   双开：d/2 >= min_lane_width_dual ；半宽 halfW = min(d/2 - gap/2, max_lane_width_dual)
    //   单开：下山 halfW = min(max_lane_width_single, d - gap)
    //         上山 halfW = d - max_lane_width_single - gap（下山优先占位）
    // =========================================================

    // ---- 选取对向中心线：优先 ParallelSegment 反向引用，否则按 direction 相反
    // 的中心线最近邻回退 ----
    auto findOpposite = [&](LaneSegment *self) -> LaneSegment *
    {
      auto *prop = self->GetProperty();
      if (!prop)
        return nullptr;
      int wantDir = 0;
      if (prop->direction == 1)
        wantDir = 2;
      else if (prop->direction == 2)
        wantDir = 1;
      ParallelSegment *pp = self->GetParallelSegment();
      if (pp)
      {
        for (int revId : {pp->leftReverseSegment, pp->rightReverseSegment})
        {
          if (revId == 0)
            continue;
          // 局部模式下 lanes 已经被限制到 targetSet，这里在 lanes 内查找
          // 即可天然地避免选中集合之外的对向被引入。
          for (auto *cand : lanes)
          {
            if (cand != self && cand->GetUniqueID() == revId)
              return cand;
          }
        }
      }
      if (wantDir == 0)
        return nullptr;
      auto *sg = self->GetGeometry();
      if (!sg || sg->GetVertexCount() == 0)
        return nullptr;
      Point3d mid = sg->GetVertex(sg->GetVertexCount() / 2);
      LaneSegment *best = nullptr;
      double bestD = std::numeric_limits<double>::max();
      for (auto *cand : lanes)
      {
        if (cand == self)
          continue;
        auto *cp = cand->GetProperty();
        if (!cp || cp->direction != wantDir)
          continue;
        double d = MinDistToPolylineFlat(mid, cand->GetGeometry());
        if (d < bestD)
        {
          bestD = d;
          best = cand;
        }
      }
      return best;
    };

    // ---- 建立 lane → 源轨迹 的映射（按 mineSegmentIndex）----
    // 距离计算以轨迹几何为准（轨迹更完整，中心线可能被截短或错位）
    std::vector<geditor::BoundSegment *> allTrajs;
    if (m_trajectorylayer)
      allTrajs = m_trajectorylayer->GetAllTrajectory();
    auto trajForLane = [&](LaneSegment *lane) -> geditor::BoundSegment *
    {
      if (!lane)
        return nullptr;
      auto *prop = lane->GetProperty();
      if (!prop)
        return nullptr;
      int idx = prop->mineSegmentIndex;
      for (auto *tr : allTrajs)
      {
        if (!tr)
          continue;
        auto *tp = tr->GetProperty();
        if (!tp)
          continue;
        if (tp->mineSegmentIndex == idx && idx > 0)
          return tr;
      }
      return nullptr;
    };

    // =========================================================
    // 新算法：显式中线 M + 平行偏移
    // =========================================================
    // 对每一对对向车道 (A, B)：
    //   1. 取 A 和 B 的源轨迹 trajA / trajB（距离基准，不用中心线）；
    //   2. 对 A 的每个顶点 a_i，投影到 trajB 得到 q_i，构造 m_i = (a_i+q_i)/2
    //      与 d_i = |a_i - q_i|；同样对 B 做一次得到 B 侧的 m'/d'；
    //   3. 分别对 A 侧的 m 序列和 B 侧的 m' 序列做多轮平滑 + 曲线平滑；
    //   4. A 内边界 = (A 侧平滑后中线) 沿其局部法向偏移 gap/2，方向朝 A；
    //      B 内边界 = (B 侧平滑后中线) 沿其局部法向偏移 gap/2，方向朝 B；
    //      因两条中线分别紧贴 A、B 的弧长，并都沿"朝 self 侧"偏 gap/2，
    //      实际落地：A 的内边界和 B 的内边界之间的最小距离 ≈ gap（几何闭式）。
    //   5. 外边界 = A 中心线沿 self 局部法向偏移 halfOuter（外侧宽度），与
    //      oppo 无关，不会越界；halfOuter 按双开/单开规则 + 方向定。
    //   6. 纵向超出对向的段：没有有效 m，改用 self 法向从 last-valid-inner 处
    //      沿 self 切线延伸（内边界就按外边界相对 self 中心线的对称距离算，
    //      避免外扩和凸出锯齿）。
    // =========================================================
    struct LaneData
    {
      std::vector<Point3d> pts;         // self 中心线顶点（接缝 ×2 密化）
      std::vector<Point3d> selfTrajPts; // self 源轨迹顶点（完整）
      std::vector<Point3d> oppTrajPts;  // 对向源轨迹顶点（完整）
      std::vector<double> d;            // 每个 pts[i] 到 oppTraj 的最近距离
      std::vector<bool> inOverlap;      // pts[i] 投影是否为 oppTraj 内部（非端点）
      std::vector<Point3d> M;           // pts[i] 的中线点 m_i（=(p_i+q_i)/2）
      std::vector<double> halfInner;    // pts[i] 的内半宽（作为公式记录）
      std::vector<double> halfOuter;    // pts[i] 的外半宽
      std::vector<double> wL;           // seam-taper 工作向量 (halfInner 临时)
      std::vector<double> wR;           // seam-taper 工作向量 (halfOuter 临时)
      bool dualWide = false;
      bool hasOpposite = false;
      LaneSegment *opposite = nullptr;
      int direction = 0;
      bool innerOnLeft = true; // 本车道内侧在左(true)或右(false)
    };
    std::unordered_map<LaneSegment *, LaneData> table;

    for (auto *lane : lanes)
    {
      auto *g = lane->GetGeometry();
      if (!g || g->GetVertexCount() < 2)
        continue;
      std::vector<Point3d> raw;
      int n = g->GetVertexCount();
      raw.reserve(n);
      for (int i = 0; i < n; ++i)
        raw.push_back(g->GetVertex(i));
      std::vector<LaneSegment *> preds, succs;
      lane->GetPredecessorSegment(preds);
      lane->GetSuccessorSegment(succs);
      LaneData ld;
      DensifySeamLocal(raw, ld.pts, params.taper_len,
                       /*hasStartSeam=*/!preds.empty(),
                       /*hasEndSeam=*/!succs.empty());
      // 填 selfTrajPts（优先用 self 的源轨迹）
      BoundSegment *selfTraj = trajForLane(lane);
      if (selfTraj && selfTraj->GetGeometry() &&
          selfTraj->GetGeometry()->GetVertexCount() >= 2)
      {
        int m = selfTraj->GetGeometry()->GetVertexCount();
        ld.selfTrajPts.reserve(m);
        for (int j = 0; j < m; ++j)
          ld.selfTrajPts.push_back(selfTraj->GetGeometry()->GetVertex(j));
      }
      else
      {
        // 回退：用 self 中心线
        ld.selfTrajPts = raw;
      }
      auto *prop = lane->GetProperty();
      ld.direction = prop ? prop->direction : 0;
      table[lane] = std::move(ld);
    }

    // 对每条 lane 求对向轨迹，计算 d[i]、inOverlap、M[i]
    for (auto *lane : lanes)
    {
      auto it = table.find(lane);
      if (it == table.end())
        continue;
      LaneData &ld = it->second;
      int n = (int)ld.pts.size();
      if (n < 2)
        continue;
      LaneSegment *oppo = findOpposite(lane);
      ld.opposite = oppo;
      if (!oppo)
        continue;

      BoundSegment *oppTraj = trajForLane(oppo);
      std::vector<Point3d> oppPts;
      if (oppTraj && oppTraj->GetGeometry() &&
          oppTraj->GetGeometry()->GetVertexCount() >= 2)
      {
        int m = oppTraj->GetGeometry()->GetVertexCount();
        oppPts.reserve(m);
        for (int j = 0; j < m; ++j)
          oppPts.push_back(oppTraj->GetGeometry()->GetVertex(j));
      }
      else if (oppo->GetGeometry() &&
               oppo->GetGeometry()->GetVertexCount() >= 2)
      {
        int m = oppo->GetGeometry()->GetVertexCount();
        oppPts.reserve(m);
        for (int j = 0; j < m; ++j)
          oppPts.push_back(oppo->GetGeometry()->GetVertex(j));
      }
      else
      {
        continue;
      }
      ld.hasOpposite = true;
      ld.oppTrajPts = std::move(oppPts);

      ld.d.assign(n, 0.0);
      ld.inOverlap.assign(n, false);
      ld.M.assign(n, Point3d());
      int leftVote = 0, rightVote = 0;
      for (int i = 0; i < n; ++i)
      {
        ProjResult pr = ProjectOntoPolyline(ld.pts[i], ld.oppTrajPts);
        ld.d[i] = pr.dist;
        ld.inOverlap[i] = !pr.atEndpoint;
        ld.M[i] = Point3d(0.5 * (ld.pts[i].x + pr.cp.x),
                          0.5 * (ld.pts[i].y + pr.cp.y),
                          0.5 * (ld.pts[i].z + pr.cp.z));
        // 判定 inner 在 self 左法向还是右法向：u = (M - pts[i]) 指向 oppo（= inner 方向）
        LocalFrame f = ComputeLocalFrame(ld.pts, i);
        double ux = ld.M[i].x - ld.pts[i].x;
        double uy = ld.M[i].y - ld.pts[i].y;
        double dotLeft = ux * f.lnx + uy * f.lny;
        if (dotLeft > 0)
          leftVote++;
        else
          rightVote++;
      }
      ld.innerOnLeft = (leftVote >= rightVote);
      SmoothWidth(ld.d, 1);
    }

    // ---- 第一遍：按"对偶全局"判定模式 (PAIR GLOBAL)，让 A/B 一起决策 ----
    std::unordered_map<LaneSegment *, bool> pairDecided;
    for (auto *lane : lanes)
    {
      auto it = table.find(lane);
      if (it == table.end())
        continue;
      if (pairDecided[lane])
        continue;
      LaneData &ld = it->second;
      if (!ld.hasOpposite)
      {
        pairDecided[lane] = true;
        continue;
      }
      LaneSegment *oppo = ld.opposite;
      auto oit = table.find(oppo);
      if (oit == table.end())
      {
        pairDecided[lane] = true;
        continue;
      }
      LaneData &od = oit->second;

      // 双开阈值：d/2 >= min_lane_width_dual  ⇔  d >= 2*min_lane_width_dual
      double threshold = 2.0 * params.min_lane_width_dual;
      double minA = std::numeric_limits<double>::max();
      double minB = std::numeric_limits<double>::max();
      for (int i = 0; i < (int)ld.d.size(); ++i)
        if (ld.inOverlap[i] && ld.d[i] < minA)
          minA = ld.d[i];
      for (int i = 0; i < (int)od.d.size(); ++i)
        if (od.inOverlap[i] && od.d[i] < minB)
          minB = od.d[i];
      bool dual = (minA >= threshold) && (minB >= threshold);
      ld.dualWide = dual;
      od.dualWide = dual;
      pairDecided[lane] = true;
      pairDecided[oppo] = true;
    }

    // ---- 第二遍：填 halfInner/halfOuter（均为半宽语义）----
    double maxSingle = params.max_lane_width_single;
    double maxDual = params.max_lane_width_dual;
    double gap = params.boundary_gap;
    auto fillHalves = [&](LaneData &ld)
    {
      int n = (int)ld.pts.size();
      ld.halfInner.assign(n, 0.0);
      ld.halfOuter.assign(n, 0.0);
      if (!ld.hasOpposite)
      {
        double fw = w_lane_;
        for (int i = 0; i < n; ++i)
        {
          ld.halfInner[i] = fw;
          ld.halfOuter[i] = fw;
        }
        return;
      }
      auto halfAt = [&](int i, double &hIn, double &hOut)
      {
        double d = ld.d[i];
        if (ld.dualWide)
        {
          // 双开：halfW = min(d/2 - gap/2, max_lane_width_dual)
          double h = d * 0.5 - gap * 0.5;
          if (h > maxDual)
            h = maxDual; // 封顶；此时内-内间距放宽
          if (h < 0.2)
            h = 0.2;
          hIn = h;
          hOut = h;
        }
        else if (ld.direction == 2)
        {
          // 单开 · 下山优先：halfW = min(max_lane_width_single, d - gap)
          double h = maxSingle;
          double cap = d - gap; // 不能越过对向中心线
          if (h > cap)
            h = cap;
          if (h < 0.2)
            h = 0.2;
          hIn = h;
          hOut = h;
        }
        else if (ld.direction == 1)
        {
          // 单开 · 上山：halfW = d - max_lane_width_single - gap
          double h = d - maxSingle - gap;
          if (h < 0.2)
            h = 0.2;
          hIn = h;
          hOut = h;
        }
        else
        {
          // 未标方向：取中线约束下的单开最高半宽
          double cap = d * 0.5 - gap * 0.5;
          if (cap < 0.2)
            cap = 0.2;
          double h = std::min(maxSingle, cap);
          hIn = h;
          hOut = h;
        }
      };
      int firstIn = -1, lastIn = -1;
      for (int i = 0; i < n; ++i)
      {
        if (ld.inOverlap[i])
        {
          halfAt(i, ld.halfInner[i], ld.halfOuter[i]);
          if (firstIn < 0)
            firstIn = i;
          lastIn = i;
        }
      }
      if (firstIn < 0)
      {
        for (int i = 0; i < n; ++i)
        {
          ld.halfInner[i] = maxSingle;
          ld.halfOuter[i] = maxSingle;
        }
      }
      else
      {
        for (int i = 0; i < firstIn; ++i)
        {
          ld.halfInner[i] = ld.halfInner[firstIn];
          ld.halfOuter[i] = ld.halfOuter[firstIn];
        }
        for (int i = lastIn + 1; i < n; ++i)
        {
          ld.halfInner[i] = ld.halfInner[lastIn];
          ld.halfOuter[i] = ld.halfOuter[lastIn];
        }
      }
    };
    for (auto *lane : lanes)
    {
      auto it = table.find(lane);
      if (it == table.end())
        continue;
      fillHalves(it->second);
      if (it->second.hasOpposite)
      {
        if (it->second.dualWide)
        {
          if (diag)
            ++diag->dualWide;
        }
        else
        {
          if (diag)
            ++diag->singleWide;
        }
      }
    }

    // 为接缝平滑阶段提供兼容的 wL/wR（沿用原 Hermite taper 管道），这里 wL/wR
    // 都取 halfInner 的值做端点过渡；halfOuter 同理另存
    for (auto &kv : table)
    {
      LaneData &ld = kv.second;
      ld.wL = ld.halfInner;
      ld.wR = ld.halfOuter;
    }

    // ---- 第三遍：接缝 Hermite 平滑过渡 + 宽度 3 点均值滤波 ----
    auto cumLen = [](const std::vector<Point3d> &p, std::vector<double> &out)
    {
      out.assign(p.size(), 0.0);
      for (size_t i = 1; i < p.size(); ++i)
      {
        double dx = p[i].x - p[i - 1].x;
        double dy = p[i].y - p[i - 1].y;
        out[i] = out[i - 1] + std::sqrt(dx * dx + dy * dy);
      }
    };
    auto hermBlend = [](double seam, double inner, double u)
    {
      if (u <= 0)
        return seam;
      if (u >= 1)
        return inner;
      double h00 = 2 * u * u * u - 3 * u * u + 1;
      double h01 = -2 * u * u * u + 3 * u * u;
      return seam * h00 + inner * h01;
    };
    for (auto *lane : lanes)
    {
      auto it = table.find(lane);
      if (it == table.end())
        continue;
      LaneData &ld = it->second;
      int n = (int)ld.pts.size();
      if (n < 2)
        continue;
      std::vector<LaneSegment *> preds, succs;
      lane->GetPredecessorSegment(preds);
      lane->GetSuccessorSegment(succs);
      double seamStartL = ld.wL.front();
      double seamStartR = ld.wR.front();
      for (auto *pr : preds)
      {
        auto pit = table.find(pr);
        if (pit == table.end() || pit->second.wL.empty())
          continue;
        seamStartL = std::min(seamStartL, pit->second.wL.back());
        seamStartR = std::min(seamStartR, pit->second.wR.back());
      }
      double seamEndL = ld.wL.back();
      double seamEndR = ld.wR.back();
      for (auto *sc : succs)
      {
        auto sit = table.find(sc);
        if (sit == table.end() || sit->second.wL.empty())
          continue;
        seamEndL = std::min(seamEndL, sit->second.wL.front());
        seamEndR = std::min(seamEndR, sit->second.wR.front());
      }
      std::vector<double> cum;
      cumLen(ld.pts, cum);
      double total = cum.back();
      double taper = params.taper_len;
      if (taper > total * 0.5)
        taper = total * 0.5;
      if (taper < 0.3)
        taper = std::min(total * 0.3, 0.5);
      std::vector<double> wL2(n), wR2(n);
      for (int i = 0; i < n; ++i)
      {
        double distStart = cum[i];
        double distEnd = total - cum[i];
        double wl = ld.wL[i];
        double wr = ld.wR[i];
        if (distStart < taper && distStart <= distEnd)
        {
          double u = distStart / taper;
          wl = hermBlend(seamStartL, wl, u);
          wr = hermBlend(seamStartR, wr, u);
        }
        else if (distEnd < taper)
        {
          double u = distEnd / taper;
          wl = hermBlend(seamEndL, wl, u);
          wr = hermBlend(seamEndR, wr, u);
        }
        wL2[i] = wl;
        wR2[i] = wr;
      }
      ld.wL.swap(wL2);
      ld.wR.swap(wR2);
      SmoothWidth(ld.wL, 1);
      SmoothWidth(ld.wR, 1);
      // wL/wR 即此时的 halfInner/halfOuter（seam-taper + smooth 之后）
      ld.halfInner = ld.wL;
      ld.halfOuter = ld.wR;
    }

    // ---- 第四遍：严格对称的边界构造 ----
    // 铁律：left[i] = self[i] + n[i] * halfW[i]
    //       right[i] = self[i] - n[i] * halfW[i]
    // halfW 是**单一值**（左右相等）——保证左右边界关于 self 中心线**精确对称**。
    // 法向 n 基于 self 折线的**平滑副本**计算，避免采样抖动导致法向跳变。
    // 两条边界都按 halfW[i] 对称放置；再由 innerOnLeft 贴标签 left/right 归属。
    struct BoundaryPts
    {
      std::vector<Point3d> left;
      std::vector<Point3d> right;
    };
    std::unordered_map<LaneSegment *, BoundaryPts> bpts;

    double maxLaneW = params.max_lane_width_dual;

    // 对每条中心线 pts，先做强平滑得到 smPts 用于切向估计（保端点）
    auto smoothSelfPts = [&](const std::vector<Point3d> &src, int iters)
    {
      std::vector<Point3d> out = src;
      int nn = (int)out.size();
      if (nn < 3)
        return out;
      std::vector<Point3d> tmp(nn);
      for (int k = 0; k < iters; ++k)
      {
        tmp[0] = out[0];
        tmp[nn - 1] = out[nn - 1];
        for (int i = 1; i < nn - 1; ++i)
        {
          tmp[i].x = 0.25 * out[i - 1].x + 0.5 * out[i].x + 0.25 * out[i + 1].x;
          tmp[i].y = 0.25 * out[i - 1].y + 0.5 * out[i].y + 0.25 * out[i + 1].y;
          tmp[i].z = 0.25 * out[i - 1].z + 0.5 * out[i].z + 0.25 * out[i + 1].z;
        }
        out.swap(tmp);
      }
      return out;
    };

    for (auto *lane : lanes)
    {
      auto it = table.find(lane);
      if (it == table.end())
        continue;
      LaneData &ld = it->second;
      int n = (int)ld.pts.size();
      if (n < 2)
      {
        if (diag)
          ++diag->skipped;
        continue;
      }
      BoundaryPts b;
      b.left.resize(n);
      b.right.resize(n);

      // 1) 平滑 self 中心线（用于切向估计）；边界仍锚在原始 ld.pts[i]
      std::vector<Point3d> smPts = smoothSelfPts(ld.pts, 6);

      // 2) 计算每个顶点的单位法向：基于 smPts 中心差分
      std::vector<double> nxArr(n, 0), nyArr(n, 1);
      for (int i = 0; i < n; ++i)
      {
        Point3d pm, pp;
        if (i - 1 >= 0)
          pm = smPts[i - 1];
        else
          pm = Point3d(2 * smPts[0].x - smPts[1].x,
                       2 * smPts[0].y - smPts[1].y,
                       2 * smPts[0].z - smPts[1].z);
        if (i + 1 < n)
          pp = smPts[i + 1];
        else
          pp = Point3d(2 * smPts[n - 1].x - smPts[n - 2].x,
                       2 * smPts[n - 1].y - smPts[n - 2].y,
                       2 * smPts[n - 1].z - smPts[n - 2].z);
        double tx = pp.x - pm.x, ty = pp.y - pm.y;
        double L = std::sqrt(tx * tx + ty * ty);
        if (L > 1e-9)
        {
          tx /= L;
          ty /= L;
        }
        else
        {
          tx = 1;
          ty = 0;
        }
        nxArr[i] = -ty; // 左法向
        nyArr[i] = tx;
      }

      // 3) 计算 halfW[i]（左右共用同一个值）：
      //    - 无对向：w_lane_
      //    - 有对向按 halfInner / halfOuter 规则；此处要求 halfW = halfInner = halfOuter
      //      因为对称生成时左右必须等宽。我们已经在 fillHalves 里让它们相等；
      //      若不等（未来扩展），取平均值。
      std::vector<double> halfW(n, 0.0);
      for (int i = 0; i < n; ++i)
      {
        double hw = 0.5 * (ld.halfInner[i] + ld.halfOuter[i]);
        if (!ld.hasOpposite)
          hw = w_lane_;
        // max_lane_width_dual 封顶（双开模式）：参数本身就是半宽上限
        if (ld.hasOpposite && ld.dualWide)
        {
          if (hw > maxLaneW)
            hw = maxLaneW;
        }
        if (hw < 0.2)
          hw = 0.2;
        halfW[i] = hw;
      }

      // 4) 对称偏移生成 left / right
      for (int i = 0; i < n; ++i)
      {
        double hw = halfW[i];
        double nx = nxArr[i], ny = nyArr[i];
        b.left[i] = Point3d(ld.pts[i].x + nx * hw, ld.pts[i].y + ny * hw,
                            ld.pts[i].z);
        b.right[i] = Point3d(ld.pts[i].x - nx * hw, ld.pts[i].y - ny * hw,
                             ld.pts[i].z);
      }
      bpts[lane] = std::move(b);
    }

    // ---- 第五遍：接缝顶点对齐（拓扑前驱/后继首尾顶点同点）----
    // 提前到硬约束之前：先让接缝处的端点协调一致，再统一做 gap 约束，
    // 否则约束完之后再均值会重新破坏 gap。
    for (auto *lane : lanes)
    {
      auto bit = bpts.find(lane);
      if (bit == bpts.end())
        continue;
      BoundaryPts &self = bit->second;
      std::vector<LaneSegment *> preds, succs;
      lane->GetPredecessorSegment(preds);
      lane->GetSuccessorSegment(succs);
      for (auto *pr : preds)
      {
        auto pit = bpts.find(pr);
        if (pit == bpts.end())
          continue;
        BoundaryPts &bp = pit->second;
        if (bp.left.empty() || bp.right.empty())
          continue;
        Point3d &sL = self.left.front();
        Point3d &sR = self.right.front();
        Point3d &pL = bp.left.back();
        Point3d &pR = bp.right.back();
        Point3d mL((sL.x + pL.x) * 0.5, (sL.y + pL.y) * 0.5,
                   (sL.z + pL.z) * 0.5);
        Point3d mR((sR.x + pR.x) * 0.5, (sR.y + pR.y) * 0.5,
                   (sR.z + pR.z) * 0.5);
        sL = mL;
        pL = mL;
        sR = mR;
        pR = mR;
      }
      for (auto *sc : succs)
      {
        auto sit = bpts.find(sc);
        if (sit == bpts.end())
          continue;
        BoundaryPts &bp = sit->second;
        if (bp.left.empty() || bp.right.empty())
          continue;
        Point3d &sL = self.left.back();
        Point3d &sR = self.right.back();
        Point3d &pL = bp.left.front();
        Point3d &pR = bp.right.front();
        Point3d mL((sL.x + pL.x) * 0.5, (sL.y + pL.y) * 0.5,
                   (sL.z + pL.z) * 0.5);
        Point3d mR((sR.x + pR.x) * 0.5, (sR.y + pR.y) * 0.5,
                   (sR.z + pR.z) * 0.5);
        sL = mL;
        pL = mL;
        sR = mR;
        pR = mR;
      }
    }

    // ---- 第六遍·硬约束：保证对向内边界最小距离 ≥ gap ----
    // 思路：在每条 lane 的"半宽缩进量 shrink[i]"维度上做迭代收缩。
    //   - shrink[i] >= 0：表示该顶点的左/右边界沿法向各向中心线收缩多少。
    //   - 用 segment-segment 最小距离检测对向内边界；任何 self 顶点 i 或
    //     self 内边界折线段上发现违规处，对应的 shrink[i]/shrink[i+1] 提升。
    //   - 每轮对 shrink 做一次 3 点平滑，避免相邻顶点收缩跳变带来的折线段交叉。
    //   - 使用"上一轮的内边界副本"作为对向参照（避免左右影响打架）。
    //   - 最多 8 轮；每轮无新增收缩则退出。
    auto segSegDist2D = [](const Point3d &p1, const Point3d &p2,
                           const Point3d &q1, const Point3d &q2) -> double
    {
      // 2D 平面两线段最小距离（z 忽略）
      auto clamp01 = [](double v)
      { return v < 0 ? 0 : (v > 1 ? 1 : v); };
      double ux = p2.x - p1.x, uy = p2.y - p1.y;
      double vx = q2.x - q1.x, vy = q2.y - q1.y;
      double wx = p1.x - q1.x, wy = p1.y - q1.y;
      double a = ux * ux + uy * uy;
      double b = ux * vx + uy * vy;
      double c = vx * vx + vy * vy;
      double d = ux * wx + uy * wy;
      double e = vx * wx + vy * wy;
      double D = a * c - b * b;
      double sc, tc;
      if (D < 1e-12)
      {
        sc = 0;
        tc = (b > c ? d / b : e / c);
      }
      else
      {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
      }
      sc = clamp01(sc);
      tc = clamp01(tc);
      // 钳制后再修正
      // 重新求最近点
      double x1 = p1.x + sc * ux, y1 = p1.y + sc * uy;
      double x2 = q1.x + tc * vx, y2 = q1.y + tc * vy;
      double dx = x1 - x2, dy = y1 - y2;
      return std::sqrt(dx * dx + dy * dy);
    };

    // shrink[lane][i]：每个顶点的"半宽缩进量"（>=0）
    std::unordered_map<LaneSegment *, std::vector<double>> shrinkMap;
    for (auto &kv : bpts)
    {
      shrinkMap[kv.first].assign(kv.second.left.size(), 0.0);
    }

    // 每个顶点 i 处，外/内边界相对中心线的法向单位向量（朝外为 +）
    auto unitOutNormal = [](const Point3d &center, const Point3d &outerPt,
                            double &ux, double &uy) -> double
    {
      double vx = outerPt.x - center.x;
      double vy = outerPt.y - center.y;
      double L = std::sqrt(vx * vx + vy * vy);
      if (L < 1e-9)
      {
        ux = 0;
        uy = 0;
        return 0;
      }
      ux = vx / L;
      uy = vy / L;
      return L;
    };

    // 应用 shrinkMap 把 bpts 复位到"基础对称偏移 - shrink"的状态
    // 基础偏移点 = 当前 bpts 中的 left/right；要重置到 baseline 需先备份
    // 这里只把"未来的 shrink 增量"应用到 bpts 上，所以 baseline = 上一轮 bpts。
    // 我们采用增量式：每轮把 dShrink 加到 shrink，并对 bpts 沿法向同步移动。
    for (int iter = 0; iter < 8; ++iter)
    {
      // 1) 拷贝当前每条 lane 的内边界折线作为对向参照
      std::unordered_map<LaneSegment *, std::vector<Point3d>> innerSnapshot;
      for (auto &kv : bpts)
      {
        auto it = table.find(kv.first);
        if (it == table.end())
          continue;
        const LaneData &ld = it->second;
        if (!ld.hasOpposite)
          continue;
        const BoundaryPts &b = kv.second;
        innerSnapshot[kv.first] = ld.innerOnLeft ? b.left : b.right;
      }

      // 2) 计算每条 lane 每个顶点应增加的收缩量 dShrink[i]
      bool anyDelta = false;
      std::unordered_map<LaneSegment *, std::vector<double>> dShrinkMap;
      for (auto *lane : lanes)
      {
        auto it = table.find(lane);
        if (it == table.end())
          continue;
        LaneData &ld = it->second;
        if (!ld.hasOpposite || !ld.opposite)
          continue;
        auto bit = bpts.find(lane);
        if (bit == bpts.end())
          continue;
        auto oppoIt = innerSnapshot.find(ld.opposite);
        if (oppoIt == innerSnapshot.end())
          continue;
        const std::vector<Point3d> &oppoInner = oppoIt->second;
        if ((int)oppoInner.size() < 2)
          continue;
        const BoundaryPts &b = bit->second;
        const std::vector<Point3d> &innerSelf =
            ld.innerOnLeft ? b.left : b.right;
        int n = (int)innerSelf.size();
        std::vector<double> dShrink(n, 0.0);

        // (a) self 顶点 vs oppo 内边界折线 → 顶点违规
        for (int i = 0; i < n; ++i)
        {
          ProjResult pr = ProjectOntoPolyline(innerSelf[i], oppoInner);
          if (pr.dist < gap - 1e-6)
          {
            double need = gap - pr.dist;
            if (need > dShrink[i])
              dShrink[i] = need;
          }
        }
        // (b) self 内边界**折线段** vs oppo 内边界**折线段** → 段间最近距离
        // 段 i 由顶点 i, i+1 组成；违规时把 dShrink[i], dShrink[i+1] 同时抬高
        for (int i = 0; i + 1 < n; ++i)
        {
          double bestD = std::numeric_limits<double>::max();
          for (int j = 0; j + 1 < (int)oppoInner.size(); ++j)
          {
            double d2 = segSegDist2D(innerSelf[i], innerSelf[i + 1],
                                     oppoInner[j], oppoInner[j + 1]);
            if (d2 < bestD)
              bestD = d2;
          }
          if (bestD < gap - 1e-6)
          {
            double need = gap - bestD;
            // 平摊到段两端，但都至少升到 need * 0.5（让两端协同收缩）
            double half = need * 0.5;
            if (dShrink[i] < half)
              dShrink[i] = half;
            if (dShrink[i + 1] < half)
              dShrink[i + 1] = half;
          }
        }
        // (c) oppo 顶点反查 self 内边界 → self 局部最近段两端共同抬高
        for (int j = 0; j < (int)oppoInner.size(); ++j)
        {
          ProjResult pr = ProjectOntoPolyline(oppoInner[j], innerSelf);
          if (pr.dist >= gap - 1e-6)
            continue;
          // pr.s 是弧长；找到 self 内边界上对应段索引 i*（顶点 i*,i*+1）
          double cum = 0.0;
          int ki = 0;
          for (; ki + 1 < n; ++ki)
          {
            double dx = innerSelf[ki + 1].x - innerSelf[ki].x;
            double dy = innerSelf[ki + 1].y - innerSelf[ki].y;
            double L = std::sqrt(dx * dx + dy * dy);
            if (cum + L >= pr.s - 1e-9)
              break;
            cum += L;
          }
          double need = gap - pr.dist;
          if (ki < n && dShrink[ki] < need)
            dShrink[ki] = need;
          if (ki + 1 < n && dShrink[ki + 1] < need)
            dShrink[ki + 1] = need;
        }

        // 3) 1D 平滑 dShrink（避免邻居断崖式跳变）
        if (n >= 3)
        {
          std::vector<double> tmp(n);
          tmp[0] = dShrink[0];
          tmp[n - 1] = dShrink[n - 1];
          for (int i = 1; i < n - 1; ++i)
          {
            tmp[i] = 0.25 * dShrink[i - 1] + 0.5 * dShrink[i] +
                     0.25 * dShrink[i + 1];
            // 平滑后下界仍是邻居中的最大需求（不允许减小）
            double need = dShrink[i];
            if (i > 0 && dShrink[i - 1] > need)
              need = std::max(need, 0.5 * dShrink[i - 1]);
            if (i + 1 < n && dShrink[i + 1] > need)
              need = std::max(need, 0.5 * dShrink[i + 1]);
            if (tmp[i] < need * 0.7)
              tmp[i] = need * 0.7;
          }
          dShrink = std::move(tmp);
        }

        for (int i = 0; i < n; ++i)
        {
          if (dShrink[i] > 1e-6)
            anyDelta = true;
        }
        dShrinkMap[lane] = std::move(dShrink);
      }
      if (!anyDelta)
        break;

      // 4) 应用 dShrink：把 bpts 沿 self 法向同步收缩；shrinkMap 累计
      for (auto &kv : dShrinkMap)
      {
        LaneSegment *lane = kv.first;
        const std::vector<double> &dS = kv.second;
        auto bit = bpts.find(lane);
        if (bit == bpts.end())
          continue;
        auto it = table.find(lane);
        if (it == table.end())
          continue;
        LaneData &ld = it->second;
        BoundaryPts &b = bit->second;
        std::vector<double> &S = shrinkMap[lane];
        int n = (int)b.left.size();
        for (int i = 0; i < n && i < (int)dS.size(); ++i)
        {
          if (dS[i] <= 1e-6)
            continue;
          // self 法向：从 ld.pts[i] 指向 b.left[i]（外法向）
          double ux, uy;
          double L = unitOutNormal(ld.pts[i], b.left[i], ux, uy);
          if (L < 1e-9)
            continue;
          // 不能让 halfW 变成负数，留 0.05 m 余量
          double curHalf = L; // 当前 left 离中心线距离 ≈ halfW
          double s = dS[i];
          if (s > curHalf - 0.05)
            s = curHalf - 0.05;
          if (s <= 1e-6)
            continue;
          // left 与 right 关于中心线对称，左右同时朝中心线收缩 s
          b.left[i].x -= ux * s;
          b.left[i].y -= uy * s;
          b.right[i].x += ux * s;
          b.right[i].y += uy * s;
          S[i] += s;
        }
      }
    }

    // ---- 第六遍：写出几何；替换旧边界 ----
    auto clearOldBoundary = [&](BoundSegment *bs)
    {
      if (!bs)
        return;
      if (m_boundlayer)
        m_boundlayer->DeleteMapFeature(bs);
      delete bs;
    };
    int processed = 0;
    for (auto *lane : lanes)
    {
      // 局部模式：只重写 targetSet 内的 lane；其它保持不变
      if (!globalMode && targetSet.find(lane) == targetSet.end())
        continue;
      auto bit = bpts.find(lane);
      if (bit == bpts.end())
        continue;
      const BoundaryPts &b = bit->second;
      if ((int)b.left.size() < 2)
      {
        if (diag)
          ++diag->skipped;
        continue;
      }
      Geometry *leftGeo = CreateGeoLine(Geometry::GT_POLYLINE);
      Geometry *rightGeo = CreateGeoLine(Geometry::GT_POLYLINE);
      for (const auto &p : b.left)
        leftGeo->AppendVertex(p);
      for (const auto &p : b.right)
        rightGeo->AppendVertex(p);
      clearOldBoundary(lane->GetLeftBoundary());
      clearOldBoundary(lane->GetRightBoundary());
      auto *bl = new BoundSegment();
      bl->SetGeometry(leftGeo);
      m_boundlayer->AddMapFeature(bl);
      auto *br = new BoundSegment();
      br->SetGeometry(rightGeo);
      m_boundlayer->AddMapFeature(br);
      lane->SetLeftBoundary(bl);
      lane->SetRightBoundary(br);
      lane->SetChanged(true);
      ++processed;
    }
    if (diag)
      diag->processed = processed;

    if (diag)
      diag->detail = os.str();
    return processed;
  }

} // namespace geditor
