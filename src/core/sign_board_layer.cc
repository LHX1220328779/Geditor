
#include "core/sign_board_layer.h"
#include "core/area_drawable.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_rectangle.h"
#include "core/lane_segment_layer.h"
#include "core/sign_drawable.h"

#include "renderGL/mc_render_pass.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"

#include "map/projection_utm.h"
#include "map/vdb_manage.h"

#include <assert.h>
#include <time.h>

namespace geditor {

PointLayer::PointLayer() : Layer(LT_SIGN) {}

PointLayer::~PointLayer() {}

void PointLayer::AddMapFeature(MapFeature *feature) {
  if (feature != NULL) {
    feature->SetMapLayer(this);

    if (feature->GetUniqueID() == 0) {
      int uniqueId = GenerateFeatureID();
      feature->SetUniqueID(uniqueId);
    }

    m_LaneSegment.push_back((SignBoard *)feature);
  }
}

bool PointLayer::DeleteMapFeature(MapFeature *feature) {
  for (std::vector<SignBoard *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    if (feature == (*it)) {
      m_LaneSegment.erase(it);
      return true;
    }
  }
  return false;
}

bool PointLayer::PickupObject(const Point3d &mousePoint, double tolerance,
                              PickupResult &reslut) {
  bool bFind = false;

  for (std::vector<SignBoard *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    Point3d outpnt;
    double fal;

    Geometry *pGeomLine = (*it)->GetGeometry();
    if (pGeomLine->GetGeometryType() == Geometry::GT_POLYGON) {
      GeoPolygon *polyline = dynamic_cast<GeoPolygon *>(pGeomLine);
      int index =
          polyline->IsPointInEdge(mousePoint, outpnt, tolerance * 4, fal);
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
    int nIndex = OnPoint(reslut.pFeatureObject->GetGeometry(), mousePoint,
                         tolerance * 4);
    reslut.nKeyPoint = nIndex;
  }

  return bFind;
}
MapFeature *PointLayer::GetFeature(int idx) {
  return GetFeatureVec(idx, m_LaneSegment);
}
//几何对象转换成渲染对象
void PointLayer::Cull(double minX, double minY, double maxX, double maxY) {
  for (std::map<Geometry *, PositionTransformNode *>::iterator iter =
           m_RenderLeaf.begin();
       iter != m_RenderLeaf.end();) {
    bool bFind = false;

    for (std::vector<SignBoard *>::iterator it = m_LaneSegment.begin();
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

  for (std::vector<SignBoard *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    SignBoard *pLaneSegment = (*it);

    SignBoardProperty *pProperty = (*it)->GetProperty();
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
      int areaType = pLaneSegment->GetProperty()->areaType;
      Drawable *pNode =
          ConvertNodeDrawable(pPolyline, highlightPoint, vCnt, areaType);
      if (pNode != NULL) {
        if (pLaneSegment->selected() || pLaneSegment->highlighted())
          pNode->SetColorType(1, 1);
        else
          pNode->SetColorType(0, 0);
        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pNode);
        pTransformNode->AddChild(pRenderLeaf);
      }
    }
  }
}

void PointLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera) {
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
        V3d vecT = position - origin;
        Matrix4x4f mat = Matrix4x4f::MakeTrans(vecT[0], vecT[1], vecT[2]);

        Matrix4x4f mvMatrix = mat * svMatrix;
        Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(2);

        //更新变换矩阵
        pRenderLeaf->SetRenderTechnique(pTechnique);
        pRenderLeaf->SetModelViewMatrix(&mvMatrix);
        pRenderLeaf->SetProjectionMatrix(&prjMatrix);
      }
    }
  }
}

void PointLayer::Draw(RenderInfo &rendinfo) {
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

int PointLayer::OnPoint(Geometry *geometry, const Point3d &Q,
                        double tolerance) {
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

V2f texArray[16][2] = {
    {V2f(0.750f, 0.250f), V2f(0.875f, 0.500f)},
    {V2f(0.000f, 0.250f), V2f(0.125f, 0.500f)},
    {V2f(0.125f, 0.250f), V2f(0.250f, 0.500f)},
    {V2f(0.250f, 0.250f), V2f(0.375f, 0.500f)},
    {V2f(0.625f, 0.250f), V2f(0.750f, 0.500f)},
    {V2f(0.375f, 0.250f), V2f(0.500f, 0.500f)},
    {V2f(0.500f, 0.250f), V2f(0.625f, 0.500f)},
    {V2f(0.125f, 0.000f), V2f(0.250f, 0.250f)},
    {V2f(0.625f, 0.000f), V2f(0.750f, 0.250f)},
    {V2f(0.375f, 0.000f), V2f(0.500f, 0.250f)},
    {V2f(0.750f, 0.000f), V2f(0.875f, 0.250f)},
    {V2f(0.875f, 0.000f), V2f(1.000f, 0.250f)},
    {V2f(0.000f, 0.000f), V2f(0.125f, 0.250f)},
    {V2f(0.500f, 0.000f), V2f(0.625f, 0.250f)},
    {V2f(0.250f, 0.000f), V2f(0.375f, 0.250f)},
};
V2f TexCoor(int type, int be) {
  static std::vector<V2f> arr;
  static int row = 4, col = 8;
  static float ri = 1.0 / row, ci = 1.0 / col;
  if (arr.empty()) {
    arr.resize(row * col);
    for (int i = 0; i < row; ++i)
      for (int j = 0; j < col; ++j) {
        arr[j + i * col] = V2f(j * ci, i * ri);
      }
  }
  if (type < 0 || type >= row * col) return arr.front() + V2f(be, be * 0.5);
  return arr[type] + V2f(be * ci, be * ri);
}

//生成渲染对象
Drawable *PointLayer::ConvertNodeDrawable(Geometry *pPolyline, int select,
                                          const V3d &vCnt, int areaType) {
  int nCount = pPolyline->GetVertexCount();

  if (nCount > 0) {
    //构造顶点格式
    VertexFormat *vertexformat = new VertexFormat();
    vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_TEXCOORD,
                                  VertexFormat::MCAT_FLOAT2, 0);

    //构建顶点数据bufffer
    VertexBuffer *vertexBuffer = new VertexBuffer(4, vertexformat->GetStride());
    VertexBufferAccessor vba(vertexformat, vertexBuffer);

    Point3d pnt = pPolyline->GetVertex(0);
    V3f origin(pnt.x - vCnt[0], pnt.y - vCnt[1], pnt.z - vCnt[2]);
    float fSize = 0.50f;
    // V2f texMinPt = texArray[areaType - 21][0];
    // V2f texMaxPt = texArray[areaType - 21][1];
    V2f texMinPt = TexCoor(areaType, 0);
    V2f texMaxPt = TexCoor(areaType, 1);
    int nIndex = 0;
    vba.Position<V3f>(nIndex) =
        V3f(origin[0] - fSize, origin[1] - fSize, origin[2]);
    vba.TCoord<V2f>(0, nIndex) = V2f(texMinPt[0], texMaxPt[1]);
    nIndex++;

    vba.Position<V3f>(nIndex) =
        V3f(origin[0] + fSize, origin[1] - fSize, origin[2]);
    vba.TCoord<V2f>(0, nIndex) = texMaxPt;
    nIndex++;

    vba.Position<V3f>(nIndex) =
        V3f(origin[0] + fSize, origin[1] + fSize, origin[2]);
    vba.TCoord<V2f>(0, nIndex) = V2f(texMaxPt[0], texMinPt[1]);
    nIndex++;

    vba.Position<V3f>(nIndex) =
        V3f(origin[0] - fSize, origin[1] + fSize, origin[2]);
    vba.TCoord<V2f>(0, nIndex) = texMinPt;
    nIndex++;

    //构造索引buffer
    IndexBuffer *indexBuffer = new IndexBuffer(6, sizeof(int));
    int *indices = (int *)indexBuffer->GetData();

    int i = 0;
    indices[i++] = 0;
    indices[i++] = 1;
    indices[i++] = 2;
    indices[i++] = 0;
    indices[i++] = 2;
    indices[i++] = 3;

    SignDrawable *pNode = new SignDrawable();
    pNode->Update(vertexformat, vertexBuffer, indexBuffer);

    return pNode;
  }
  return NULL;
}

void PointLayer::Save(VDBManage *vdb) {
  vdb->ClearSignboardProperty();

  for (std::vector<SignBoard *>::iterator iter = m_LaneSegment.begin();
       iter != m_LaneSegment.end(); iter++) {
    SignBoard *pLaneSegment = (*iter);

    char *pMem = NULL;
    int length = PackGeometry(pLaneSegment->GetGeometry(), pMem);

    vdb->SaveSignboardProperty(pLaneSegment->GetUniqueID(),
                               pLaneSegment->GetProperty(), pMem, length);
  }
}

void PointLayer::ClearLayer() {
  for (std::vector<SignBoard *>::iterator iter = m_LaneSegment.begin();
       iter != m_LaneSegment.end(); iter++) {
    delete (*iter);
  }
  m_LaneSegment.clear();
}

void PointLayer::Read(VDBManage *vdb, SegmentLayer *pLayer) {
  ClearLayer();

  std::vector<Sign *> areaArray;
  vdb->ReadSignboardProperty(areaArray);
  for (int i = 0; i < areaArray.size(); i++) {
    SignBoard *pLaneSegment = new SignBoard();

    Geometry *pPolyline = areaArray[i]->polyline;

    pLaneSegment->SetGeometry(areaArray[i]->polyline);
    pLaneSegment->SetProperty(&areaArray[i]->pProperty);
    pLaneSegment->SetUniqueID(areaArray[i]->uniqueId);

    AddMapFeature(pLaneSegment);

    delete areaArray[i];
  }

  for (int i = 0; i < m_LaneSegment.size(); i++) {
    int uniqueID = m_LaneSegment[i]->GetUniqueID();

    std::vector<int> attachObjects;
    vdb->ReadAttachObject2(uniqueID, attachObjects);

    for (int j = 0; j < attachObjects.size(); j++) {
      LaneSegment *pLane = (LaneSegment *)pLayer->GetFeature(attachObjects[j]);
      if (pLane != NULL) {
        m_LaneSegment[i]->AddRelationSegment(pLane);
      }
    }
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

int PointLayer::PackGeometry(Geometry *pPolyline, char *&pMem) {
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