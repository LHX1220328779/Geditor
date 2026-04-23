
#include "core/road_area_layer.h"
#include "core/camera.h"
#include "core/factory_drawable.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_rectangle.h"
#include "core/road_area.h"
#include "map/vdb_manage.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"

#include <time.h>

namespace geditor {

AreaLayer::AreaLayer() : Layer(LT_ROADAREA) {}

AreaLayer::~AreaLayer() {
  for (std::vector<RoadArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    delete (*it);
  }
  m_LaneSegment.clear();
}

MapFeature *AreaLayer::GetFeature(int idx) {
  return GetFeatureVec(idx, m_LaneSegment);
}
static bool Judge(Point3d aa, Point3d bb, Point3d cc, Point3d dd) {
  //判断两个形成的矩形不相交
  if (Mathd::Max(aa.x, bb.x) < Mathd::Min(cc.x, dd.x)) return false;
  if (Mathd::Max(aa.y, bb.y) < Mathd::Min(cc.y, dd.y)) return false;
  if (Mathd::Max(cc.x, dd.x) < Mathd::Min(aa.x, bb.x)) return false;
  if (Mathd::Max(cc.y, dd.y) < Mathd::Min(aa.y, bb.y)) return false;

  double rt0 = (aa.x - cc.x) * (aa.y - bb.y) - (aa.y - cc.y) * (aa.x - bb.x);
  double rt1 = (aa.x - bb.x) * (aa.y - dd.y) - (aa.y - bb.y) * (aa.x - dd.x);
  if (rt0 * rt1 < 0) {
    return false;
  }

  double ret1 = (cc.x - aa.x) * (cc.y - dd.y) - (cc.y - aa.y) * (cc.x - dd.x);
  double ret2 = (cc.x - dd.x) * (cc.y - bb.y) - (cc.y - dd.y) * (cc.x - bb.x);
  if (ret1 * ret2 < 0) {
    return false;
  }

  return true;
}

static bool LineIntersect(const Point3d &P1, const Point3d &P2,
                          const Point3d &P3, const Point3d &P4, Point3d *Pt) {
  float numera = (P4.x - P3.x) * (P1.y - P3.y) - (P4.y - P3.y) * (P1.x - P3.x);
  float numerb = (P2.x - P1.x) * (P1.y - P3.y) - (P2.y - P1.y) * (P1.x - P3.x);

  float denom = (P4.y - P3.y) * (P2.x - P1.x) - (P4.x - P3.x) * (P2.y - P1.y);

  if (denom == 0.0f) {
    return false;
  }

  float ua = numera / denom;
  float ub = numerb / denom;

  if (ua >= 0.0f && ua <= 1.0f && ub >= 0.0f && ub <= 1.0f) {
    Pt->x = P1.x + ua * (P2.x - P1.x);
    Pt->y = P1.y + ua * (P2.y - P1.y);

    return true;
  }

  return false;
}

static bool CliperPolygon(GeoPolyline *pClipLine, GeoPolygon *pPolygon,
                          GeoPolygon *polygon) {
  //交叉点数量
  int counter = 0;

  //交点线的索引
  int idx_line[2];

  //交点polygon的索引
  int idx_poly[2];

  //交点
  Point3d crossPnt[2];

  int iCount = pClipLine->GetVertexCount();
  for (int i = 0; i < iCount - 1; i++) {
    Point3d ptStart = pClipLine->GetVertex(i);
    Point3d ptEnd = pClipLine->GetVertex(i + 1);

    int iSize = pPolygon->GetVertexCount();
    for (int j = 0; j < iSize; j++) {
      Point3d pt0 = pPolygon->GetVertex(j);
      Point3d pt1 = pPolygon->GetVertex((j + 1) % iSize);

      if (Judge(ptStart, ptEnd, pt0, pt1)) {
        Point3d outPt;

        if (LineIntersect(ptStart, ptEnd, pt0, pt1, &outPt)) {
          idx_line[counter] = i;
          idx_poly[counter] = j;
          crossPnt[counter] = outPt;

          if (++counter >= 2) {
            int iVertexCount = pPolygon->GetVertexCount();
            for (int x = 0; x < 2; x++) {
              polygon[x].AppendVertex(crossPnt[0]);
              for (int j = idx_line[0] + 1; j <= idx_line[1]; j++) {
                Point3d pt = pClipLine->GetVertex(j);
                polygon[x].AppendVertex(pt);
              }
              polygon[x].AppendVertex(crossPnt[1]);
            }

            //--------------------------------------------------------
            polygon[1].ReverseVertex();
            for (int x = 0; x < 2; x++) {
              int start_index, end_index;
              if (x == 0) {
                start_index = idx_poly[1] + 1;
                end_index = (idx_poly[0] + 1) % iVertexCount;
              } else {
                start_index = idx_poly[0] + 1;
                end_index = (idx_poly[1] + 1) % iVertexCount;
              }

              for (int j = start_index; j % iVertexCount != end_index; j++) {
                Point3d pt = pPolygon->GetVertex(j % iVertexCount);
                polygon[x].AppendVertex(pt);
              }
            }

            return true;
          }  // end  if ++counter >= 2
        }    // end  if LineIntersect
      }      // end  if Judge
    }        // end for
  }          // end for

  return false;
}

bool AreaLayer::CliperLayer(GeoPolyline *pClipLine) {
  for (int i = 0; i < m_LaneSegment.size(); i++) {
    RoadArea *feature = m_LaneSegment[i];

    GeoPolygon *pPolygon = (GeoPolygon *)feature->GetGeometry();

    GeoPolygon polygon[2];

    if (CliperPolygon(pClipLine, pPolygon, polygon)) {
      AreaProperty *pProperty = feature->GetProperty();
      pPolygon->Clear();

      int iCount = polygon[0].GetVertexCount();
      for (int j = 0; j < iCount; j++) {
        Point3d pt = polygon[0].GetVertex(j);
        pPolygon->AppendVertex(pt);
      }

      RoadArea *pAreaRoad = new RoadArea();

      GeoPolygon *pGeo = new GeoPolygon();
      iCount = polygon[1].GetVertexCount();
      for (int j = 0; j < iCount; j++) {
        Point3d pt = polygon[1].GetVertex(j);
        pGeo->AppendVertex(pt);
      }

      pAreaRoad->SetProperty(pProperty);
      pAreaRoad->SetGeometry(pGeo);

      AddMapFeature(pAreaRoad);

      //==================================
      return true;
    }
  }

  return false;
}

//添加地图要素
void AreaLayer::AddMapFeature(MapFeature *feature) {
  if (feature != NULL) {
    feature->SetMapLayer(this);

    if (feature->GetUniqueID() == 0) {
      int uniqueId = GenerateFeatureID();
      feature->SetUniqueID(uniqueId);
    }

    m_LaneSegment.push_back((RoadArea *)feature);
  }
}

bool AreaLayer::DeleteMapFeature(MapFeature *feature) {
  for (std::vector<RoadArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    if (feature == (*it)) {
      m_LaneSegment.erase(it);
      return true;
    }
  }
  return false;
}

bool AreaLayer::GetBoundary(V3d &vMin, V3d &vMax) {
  if (m_LaneSegment.size() > 0) {
    for (std::vector<RoadArea *>::iterator it = m_LaneSegment.begin();
         it != m_LaneSegment.end(); it++) {
      Geometry *pGeo = (*it)->GetGeometry();
      pGeo->CalculateBoundBox();

      const BoundBox3d &box = pGeo->GetBound();

      if (box.v_min_[0] < vMin[0]) vMin[0] = box.v_min_[0];
      if (box.v_min_[1] < vMin[1]) vMin[1] = box.v_min_[1];
      if (box.v_min_[2] < vMin[2]) vMin[2] = box.v_min_[2];
      if (box.v_max_[0] > vMax[0]) vMax[0] = box.v_max_[0];
      if (box.v_max_[1] > vMax[1]) vMax[1] = box.v_max_[1];
      if (box.v_max_[2] > vMax[2]) vMax[2] = box.v_max_[2];
    }
    return true;
  }

  return false;
}

void AreaLayer::GetAllMapFeature(std::vector<MapFeature *> &objects) {
  for (int i = 0; i < m_LaneSegment.size(); i++) {
    MapFeature *pMapFeature = m_LaneSegment[i];

    objects.push_back(pMapFeature);
  }
}

bool AreaLayer::PickupObject(const Point3d &mousePoint, double tolerance,
                             PickupResult &reslut) {
  bool bFind = false;

  for (std::vector<RoadArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    Point3d outpnt;
    double fal;

    Geometry *pGeomLine = (*it)->GetGeometry();
    if (pGeomLine->GetGeometryType() == Geometry::GT_POLYGON) {
      GeoPolygon *polyline = dynamic_cast<GeoPolygon *>(pGeomLine);
      int index = polyline->IsPointInEdge(mousePoint, outpnt, tolerance, fal);
      if (index > 0) {
        if (fal < reslut.dDistance) {
          reslut.pFeatureObject = (*it);
          reslut.ptNearPoint = outpnt;
          reslut.nSegmentIdx = index;
          reslut.dDistance = fal;

          bFind = true;
        }
      }
    }
  }

  if (bFind) {
    int nIndex =
        OnPoint(reslut.pFeatureObject->GetGeometry(), mousePoint, tolerance);
    reslut.nKeyPoint = nIndex;
  }

  return bFind;
}

//几何对象转换成渲染对象
void AreaLayer::Cull(double minX, double minY, double maxX, double maxY) {
  for (std::map<Geometry *, PositionTransformNode *>::iterator iter =
           m_RenderLeaf.begin();
       iter != m_RenderLeaf.end();) {
    bool bFind = false;

    for (std::vector<RoadArea *>::iterator it = m_LaneSegment.begin();
         it != m_LaneSegment.end(); it++) {
      Geometry *pPolyline = (*it)->GetGeometry();
      if (pPolyline == iter->first) {
        bFind = true;
        break;
      }
    }

    if (!bFind) {
      std::map<Geometry *, PositionTransformNode *>::iterator it_delete =
          iter++;
      m_RenderLeaf.erase(it_delete);
    } else {
      iter++;
    }
  }

  for (std::vector<RoadArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    RoadArea *pLaneSegment = (*it);

    AreaProperty *pProperty = (*it)->GetProperty();
    Geometry *pPolyline = (*it)->GetGeometry();
    std::map<Geometry *, PositionTransformNode *>::iterator it_transformNode =
        m_RenderLeaf.find(pPolyline);

    PositionTransformNode *pTransformNode = NULL;

    if (m_RenderLeaf.end() == it_transformNode) {
      pTransformNode = new PositionTransformNode();
      m_RenderLeaf.insert(std::pair<Geometry *, PositionTransformNode *>(
          pPolyline, pTransformNode));
    } else {
      if (pLaneSegment->IsChanged()) {
        pTransformNode = it_transformNode->second;
        pLaneSegment->SetChanged(false);
      }
    }

    if (pTransformNode != NULL) {
      pTransformNode->RemoveAllChild();

      pPolyline->CalculateBoundBox();
      BoundBox3d bound = pPolyline->GetBound();
      V3d vCnt = bound.GetCenter();
      pTransformNode->SetPosition(vCnt);

      PolygonSytle *pStyle = pLaneSegment->GetStyle();
      int highlightPoint = pLaneSegment->GetHighlightPoint();

      // Color clr = m_colorAreaMap.GetDisplayColor(pProperty->areaType);
      Color clr(200, 200, 200, 255);
      clr.a =
          (pLaneSegment->selected() || pLaneSegment->highlighted()) ? 175 : 100;
      V4f col(clr.r / 255.0f, clr.g / 255.0f, clr.b / 255.0f, clr.a / 255.0);
      Drawable *pArrow =
          FactoryDrawable::CreateAreaDrawable(pPolyline, vCnt, col);
      if (pArrow != NULL) {
        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(1);

        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pArrow);
        pRenderLeaf->SetRenderTechnique(pTechnique);

        pTransformNode->AddChild(pRenderLeaf);
      }

      Drawable *pLine = FactoryDrawable::CreateLineDrawable(
          pPolyline, vCnt, pStyle->GetLineColor());
      if (pLine != NULL) {
        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(4);

        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pLine);
        pRenderLeaf->SetRenderTechnique(pTechnique);

        pTransformNode->AddChild(pRenderLeaf);
      }

      V3f vclr = pStyle->GetKeyVertexColor();
      Drawable *pNode = FactoryDrawable::CreateNodeDrawable(
          pPolyline, vCnt, 0.04f, V4f(vclr[0], vclr[1], vclr[2], 1.0),
          highlightPoint);
      if (pNode != NULL) {
        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(1);

        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pNode);
        pRenderLeaf->SetRenderTechnique(pTechnique);

        pTransformNode->AddChild(pRenderLeaf);
      }
    }
  }
}

void AreaLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera) {
  for (std::map<Geometry *, PositionTransformNode *>::iterator it =
           m_RenderLeaf.begin();
       it != m_RenderLeaf.end(); it++) {
    PositionTransformNode *pTransformNode = it->second;
    if (pTransformNode != NULL) {
      const V3d &position = pTransformNode->GetPosition();

      for (int i = 0; i < pTransformNode->GetNumChildren(); i++) {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);

        //生成平移矩阵
        V3d origin = pCamera->GetPostion();
        Viewport viewport = pCamera->GetViewport();

        V3d vecT = position - origin;
        Matrix4x4f mat = Matrix4x4f::MakeTrans(vecT[0], vecT[1], vecT[2]);

        Matrix4x4f mvMatrix = mat * svMatrix;
        Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

        pRenderLeaf->SetViewport(viewport);
        pRenderLeaf->SetModelViewMatrix(&mvMatrix);
        pRenderLeaf->SetProjectionMatrix(&prjMatrix);
      }
    }
  }
}

void AreaLayer::Draw(RenderInfo &rendinfo) {
  if (!m_bVisible) return;
  for (std::map<Geometry *, PositionTransformNode *>::iterator it =
           m_RenderLeaf.begin();
       it != m_RenderLeaf.end(); it++) {
    PositionTransformNode *pTransformNode = it->second;
    if (pTransformNode != NULL) {
      for (int i = 0; i < pTransformNode->GetNumChildren(); i++) {
        RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);
        if (pRenderLeaf != NULL) {
          pRenderLeaf->Render(rendinfo, NULL);
        }
      }
    }
  }
}

int AreaLayer::OnPoint(Geometry *geometry, const Point3d &Q, double tolerance) {
  int index = -1;
  double minFlag = 100;

  int nSize = geometry->GetVertexCount();
  for (int i = 0; i < nSize; i++) {
    Point3d point = geometry->GetVertex(i);

    double maxVal =
        Mathd::Max(Mathd::Abs(point.x - Q.x), Mathd::Abs(point.y - Q.y));
    if (maxVal < minFlag) {
      minFlag = maxVal;
      index = i;
    }
  }

  if (minFlag < tolerance) {
    return index;
  } else {
    return -1;
  }
}

void AreaLayer::Save(VDBManage *vdb) {
  vdb->ClearAreaProperty();

  for (std::vector<RoadArea *>::iterator iter = m_LaneSegment.begin();
       iter != m_LaneSegment.end(); iter++) {
    RoadArea *pLaneSegment = (*iter);

    char *pMem = NULL;
    int length = PackGeometry(pLaneSegment->GetGeometry(), pMem);

    vdb->SaveAreaProperty(pLaneSegment->GetUniqueID(),
                          pLaneSegment->GetProperty(), pMem, length);
  }
}

void AreaLayer::ClearLayer() {
  for (std::vector<RoadArea *>::iterator iter = m_LaneSegment.begin();
       iter != m_LaneSegment.end(); iter++) {
    delete (*iter);
  }
  m_LaneSegment.clear();
}

void AreaLayer::Read(VDBManage *vdb) {
  ClearLayer();

  std::vector<Area *> areaArray;
  vdb->ReadAreaProperty(areaArray);
  for (int i = 0; i < areaArray.size(); i++) {
    RoadArea *pLaneSegment = new RoadArea();

    Geometry *pPolyline = areaArray[i]->polyline;

    pLaneSegment->SetGeometry(areaArray[i]->polyline);
    pLaneSegment->SetProperty(&areaArray[i]->pProperty);
    pLaneSegment->SetUniqueID(areaArray[i]->uniqueId);

    this->AddMapFeature(pLaneSegment);

    delete areaArray[i];
  }
}

struct GEO_HDR {
  unsigned int magic;
  unsigned short version;
  unsigned short reserve;
  unsigned int crc32;
};

struct GEO_INFO {
  int geotype;  //图形类别
  int count;    //点数量
  double minLat;
  double maxLat;
  double maxLon;
  double minLon;
  float minAlt;
  float maxAlt;
};

int AreaLayer::PackGeometry(Geometry *pPolyline, char *&pMem) {
  //---------------------------------------------
  GEO_HDR hdr;

  hdr.magic = 0x00306567;
  hdr.version = 1;
  hdr.reserve = 0;
  hdr.crc32 = 0;

  //------------------------------------

  GEO_INFO info;

  std::vector<Point3d> items;
  Geometry::GeometryType type = pPolyline->GetGeometryType();
  if (type == Geometry::GT_BEZIER_CURVE) {
    GeoBezierCurve3 *pCurve = (GeoBezierCurve3 *)pPolyline;
    pCurve->Hermite(items);
  } else if (type == Geometry::GT_BSPLINE_CURVE) {
    GeoBSplineCurve3 *pCurve = (GeoBSplineCurve3 *)pPolyline;
    pCurve->Hermite(items);
  } else if (type == Geometry::GT_CIRCULAR_ARC) {
    GeoCircularArc *pCurve = (GeoCircularArc *)pPolyline;
    pCurve->Hermite(items);
  } else if (type == Geometry::GT_HERMITE_CURVE) {
    LOG(ERROR) << "Hermite not supported yet";
    // GeoHermiteCurve3 *pCurve = (GeoHermiteCurve3 *) pPolyline;
    // pCurve->Hermite(items);
  } else if (type == Geometry::GT_POLYLINE || type == Geometry::GT_POLYGON) {
    for (int i = 0; i < pPolyline->GetVertexCount(); i++) {
      Point3d pt = pPolyline->GetVertex(i);
      items.push_back(pt);
    }
  } else if (type == Geometry::GT_RECTANGLE) {
    GeoRectangle *pCurve = (GeoRectangle *)pPolyline;
    pCurve->Hermite(items);
  }
  //--------------------------------------------------

  std::vector<GPSPoint> pointlist;
  //-----------------------------------
  int nCount = items.size();
  pointlist.resize(nCount);

  ProjectionUTM projectionUTM;
  for (int i = 0; i < nCount; i++) {
    int nzone = ProjectionUTM::zone;
    projectionUTM.CartesianToLatLon(items[i].x, items[i].y, nzone, false,
                                    pointlist[i].latlon);
    pointlist[i].altitude = items[i].z;
  }
  //----------------------------------------------------------

  std::vector<GPSPoint> rawPoints;
  //-----------------------------------------
  int rawCount = pPolyline->GetVertexCount();
  rawPoints.resize(rawCount);

  for (int i = 0; i < rawCount; i++) {
    Point3d pt = pPolyline->GetVertex(i);
    int nzone = ProjectionUTM::zone;
    projectionUTM.CartesianToLatLon(pt.x, pt.y, nzone, false,
                                    rawPoints[i].latlon);
    rawPoints[i].altitude = pt.z;
    rawPoints[i].id = pt.GetId();
  }
  //临时这样写
  hdr.reserve = rawCount;
  //----------------------------------

  info.geotype = type;
  info.count = nCount;
  info.minLat = 0;
  info.maxLat = 0;
  info.maxLon = 0;
  info.minLon = 0;
  info.minAlt = 0;
  info.maxAlt = 0;

  //----------------------------------
  int length = sizeof(GEO_HDR) + sizeof(GEO_INFO) +
               sizeof(GPSPoint) * (nCount + rawCount);
  char *pMemery = new char[length];
  int nWriteByte = 0;

  memcpy(pMemery + nWriteByte, &hdr, sizeof(GEO_HDR));
  nWriteByte += sizeof(GEO_HDR);

  memcpy(pMemery + nWriteByte, &info, sizeof(GEO_INFO));
  nWriteByte += sizeof(GEO_INFO);

  memcpy(pMemery + nWriteByte, &pointlist[0], sizeof(GPSPoint) * nCount);
  nWriteByte += sizeof(GPSPoint) * nCount;

  memcpy(pMemery + nWriteByte, &rawPoints[0], sizeof(GPSPoint) * rawCount);
  nWriteByte += sizeof(GPSPoint) * rawCount;

  pMem = pMemery;

  return nWriteByte;
}

}  // namespace geditor