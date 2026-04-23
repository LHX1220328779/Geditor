
#include "core/topology_layer.h"

#include <assert.h>
#include <time.h>

#include "core/area_drawable.h"
#include "core/edge_drawable.h"
#include "core/factory_drawable.h"
#include "core/geo_arc_line.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_rectangle.h"
#include "core/lane_segment_layer.h"
#include "core/pointset_drawable.h"
#include "map/projection_utm.h"
#include "map/vdb_manage.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"

namespace geditor {

TopologyLayer::TopologyLayer() : Layer(LT_TOPO), transform_node_(NULL) {
  path_update_ = false;
}

TopologyLayer::~TopologyLayer() {
  for (std::map<int, PointElement *>::iterator it = path_points_.begin();
       it != path_points_.end(); it++) {
    delete it->second;
  }
  path_points_.clear();
}

void TopologyLayer::AddMapFeature(MapFeature *feature) {
  if (feature != NULL) {
    feature->SetMapLayer(this);

    if (feature->GetUniqueID() == 0) {
      int uniqueId = GenerateFeatureID();
      feature->SetUniqueID(uniqueId);
    }

    int uniqueID = feature->GetUniqueID();
    path_points_.insert(
        std::pair<int, PointElement *>(uniqueID, (PointElement *)feature));

    path_update_ = true;
  }
}

bool TopologyLayer::DeleteMapFeature(MapFeature *feature) {
  //首先删除关联的拓扑
  for (std::vector<std::pair<int, int>>::iterator it = path_topology_.begin();
       it != path_topology_.end();) {
    if (it->first == feature->GetUniqueID() ||
        it->second == feature->GetUniqueID()) {
      it = path_topology_.erase(it);
    } else {
      ++it;
    }
  }

  //再删除点
  for (std::map<int, PointElement *>::iterator it = path_points_.begin();
       it != path_points_.end(); it++) {
    if (feature == it->second) {
      path_points_.erase(it);

      path_update_ = true;

      return true;
    }
  }
  return false;
}

bool TopologyLayer::PickupObject(const Point3d &mousePoint, double tolerance,
                                 PickupResult &reslut) {
  bool bFind = false;

  for (std::map<int, PointElement *>::iterator it = path_points_.begin();
       it != path_points_.end(); it++) {
    Point3d outpnt;
    double fal;

    Geometry *pGeomLine = it->second->GetGeometry();
    if (pGeomLine->GetGeometryType() == Geometry::GT_POLYGON) {
      GeoPolygon *polyline = dynamic_cast<GeoPolygon *>(pGeomLine);
      int index = polyline->IsPointInEdge(mousePoint, outpnt, tolerance, fal);
      if (index > 0) {
        if (fal < reslut.dDistance) {
          reslut.pFeatureObject = it->second;
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
void TopologyLayer::Cull(double minX, double minY, double maxX, double maxY) {
  bool bChanged = false;
  for (std::map<int, PointElement *>::iterator iter = path_points_.begin();
       iter != path_points_.end(); iter++) {
    if (iter->second->IsChanged()) {
      bChanged = true;
      break;
    }
  }

  if (bChanged || path_update_) {
    for (std::map<int, PointElement *>::iterator iter = path_points_.begin();
         iter != path_points_.end(); iter++) {
      iter->second->SetChanged(false);
    }
    path_update_ = false;

    PositionTransformNode *pTransformNode = NULL;
    if (transform_node_) {
      transform_node_->RemoveAllChild();
      pTransformNode = transform_node_;
    } else {
      pTransformNode = new PositionTransformNode();
      transform_node_ = pTransformNode;
    }

    //-------------------------------------------
    //计算bound box
    BoundBox3d boundBox;
    for (std::map<int, PointElement *>::iterator iter = path_points_.begin();
         iter != path_points_.end(); iter++) {
      Geometry *pMapFeature = iter->second->GetGeometry();
      pMapFeature->CalculateBoundBox();
      BoundBox3d bound = pMapFeature->GetBound();

      boundBox.ExpandBy(bound);
    }

    boundBox.v_min_[2] = -1000;
    boundBox.v_max_[2] = 1000;

    //----------------------------------------------

    V3d vCnt = boundBox.GetCenter();
    pTransformNode->SetPosition(vCnt);

    //生成拓扑点渲染对象
    Drawable *pPointNode = ConvertNodeDrawable(NULL, vCnt);
    if (pPointNode != NULL) {
      RenderLeaf *pRenderLeaf = new RenderLeaf();
      pRenderLeaf->SetDrawable(pPointNode);

      pTransformNode->AddChild(pRenderLeaf);
    }

    //生成拓扑线条渲染对象
    //--------------------------------------------
    Drawable *pLineNode = ConvertLineDrawable(NULL, 0, vCnt);
    if (pLineNode != NULL) {
      RenderLeaf *pRenderLeaf = new RenderLeaf();
      pRenderLeaf->SetDrawable(pLineNode);

      pTransformNode->AddChild(pRenderLeaf);
    }
  }
}

void TopologyLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera) {
  PositionTransformNode *pTransformNode = transform_node_;
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
          TechniqueManager::GetInstance()->GetTechnique(1);

      //更新变换矩阵
      pRenderLeaf->SetRenderTechnique(pTechnique);
      pRenderLeaf->SetModelViewMatrix(&mvMatrix);
      pRenderLeaf->SetProjectionMatrix(&prjMatrix);
      pRenderLeaf->SetViewport(pCamera->GetViewport());
    }
  }
}

void TopologyLayer::Draw(RenderInfo &rendinfo) {
  if (!m_bVisible) return;
  PositionTransformNode *pTransformNode = transform_node_;
  if (pTransformNode != NULL) {
    for (int i = 0; i < pTransformNode->GetNumChildren(); i++) {
      RenderLeaf *pRenderLeaf = (RenderLeaf *)pTransformNode->GetChild(i);
      if (pRenderLeaf != NULL) {
        pRenderLeaf->Render(rendinfo, NULL);
      }
    }
  }
}

int TopologyLayer::OnPoint(Geometry *geometry, const Point3d &Q,
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

//生成渲染对象
Drawable *TopologyLayer::ConvertLineDrawable(Geometry *pPolyline, int select,
                                             const V3d &vCnt) {
  std::vector<Point3d> vertexArr;
  for (std::vector<std::pair<int, int>>::iterator iter = path_topology_.begin();
       iter != path_topology_.end(); iter++) {
    //设置原点
    std::pair<int, int> &topology = *iter;

    std::map<int, PointElement *>::iterator it1_find =
        path_points_.find(topology.first);
    if (it1_find == path_points_.end()) {
      continue;
    }

    std::map<int, PointElement *>::iterator it2_find =
        path_points_.find(topology.second);
    if (it2_find == path_points_.end()) {
      continue;
    }

    Point3d point1 = it1_find->second->GetGeometry()->GetStartVertex();
    Point3d point2 = it2_find->second->GetGeometry()->GetStartVertex();

    vertexArr.push_back(point1);
    vertexArr.push_back(point2);
  }

  int nCount = vertexArr.size();
  if (nCount > 1) {
    //------------------------------------------------

    V4f vClr(0.0, 0.0, 1.0, 1.0);

    //构造顶点格式
    VertexFormat *vertexformat = new VertexFormat();
    vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                  VertexFormat::MCAT_FLOAT4, 0);

    //构建顶点数据bufffer
    int vbSize = vertexArr.size();
    VertexBuffer *vertexBuffer =
        new VertexBuffer(vbSize, vertexformat->GetStride());
    VertexBufferAccessor vba(vertexformat, vertexBuffer);

    //构造索引buffer
    int ibSize = vertexArr.size();
    IndexBuffer *indexBuffer = new IndexBuffer(ibSize, sizeof(int));
    int *indices = (int *)indexBuffer->GetData();

    int vbIdx = 0, ibIdx = 0;

    for (int i = 0; i < vertexArr.size(); i += 2) {
      indices[ibIdx++] = i;
      indices[ibIdx++] = i + 1;

      vba.Position<V3f>(vbIdx) =
          V3f(vertexArr[i].x - vCnt[0], vertexArr[i].y - vCnt[1],
              vertexArr[i].z - vCnt[2]);
      vba.Color<V4f>(0, vbIdx) = vClr;
      vbIdx++;

      vba.Position<V3f>(vbIdx) =
          V3f(vertexArr[i + 1].x - vCnt[0], vertexArr[i + 1].y - vCnt[1],
              vertexArr[i + 1].z - vCnt[2]);
      vba.Color<V4f>(0, vbIdx) = vClr;
      vbIdx++;
    }

    EgdeDrawable *pNode = new EgdeDrawable();
    pNode->Update(vertexformat, vertexBuffer, indexBuffer);

    return pNode;
  }
  return NULL;
}

//生成渲染对象
Drawable *TopologyLayer::ConvertNodeDrawable(Geometry *pPolyline,
                                             const V3d &vCnt) {
  int nCount = path_points_.size();
  if (nCount > 0) {
    const float fSize = 0.1250f;
    V4f vClr(0.0, 0.0, 1.0, 1.0);

    //构造顶点格式
    VertexFormat *vertexformat = new VertexFormat();
    vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                  VertexFormat::MCAT_FLOAT4, 0);

    //构建顶点数据bufffer
    int vbSize = nCount * 4;
    VertexBuffer *vertexBuffer =
        new VertexBuffer(vbSize, vertexformat->GetStride());
    VertexBufferAccessor vba(vertexformat, vertexBuffer);

    //构造索引buffer
    int ibSize = nCount * 6;
    IndexBuffer *indexBuffer = new IndexBuffer(ibSize, sizeof(int));
    int *indices = (int *)indexBuffer->GetData();

    int vbIdx = 0, ibIdx = 0;

    for (std::map<int, PointElement *>::iterator iter = path_points_.begin();
         iter != path_points_.end(); iter++) {
      // V4f vClr = iter->second->GetColor();

      //设置原点
      Point3d pnt = iter->second->GetGeometry()->GetStartVertex();
      V3f origin(pnt.x - vCnt[0], pnt.y - vCnt[1], pnt.z - vCnt[2]);

      indices[ibIdx++] = vbIdx;
      indices[ibIdx++] = vbIdx + 1;
      indices[ibIdx++] = vbIdx + 2;
      indices[ibIdx++] = vbIdx;
      indices[ibIdx++] = vbIdx + 2;
      indices[ibIdx++] = vbIdx + 3;

      vba.Position<V3f>(vbIdx) =
          V3f(origin[0] - fSize, origin[1] - fSize, origin[2]);
      vba.Color<V4f>(0, vbIdx) = vClr;
      vbIdx++;

      vba.Position<V3f>(vbIdx) =
          V3f(origin[0] + fSize, origin[1] - fSize, origin[2]);
      vba.Color<V4f>(0, vbIdx) = vClr;
      vbIdx++;

      vba.Position<V3f>(vbIdx) =
          V3f(origin[0] + fSize, origin[1] + fSize, origin[2]);
      vba.Color<V4f>(0, vbIdx) = vClr;
      vbIdx++;

      vba.Position<V3f>(vbIdx) =
          V3f(origin[0] - fSize, origin[1] + fSize, origin[2]);
      vba.Color<V4f>(0, vbIdx) = vClr;
      vbIdx++;
    }

    PointSetDrawable *pNode = new PointSetDrawable();
    pNode->Update(vertexformat, vertexBuffer, indexBuffer);

    return pNode;
  }
  return NULL;
}

void TopologyLayer::DeleteTopology(int indx1, int indx2) {
  for (std::vector<std::pair<int, int>>::iterator it = path_topology_.begin();
       it != path_topology_.end(); it++) {
    if ((indx1 == it->first && indx2 == it->second) ||
        (indx2 == it->first && indx1 == it->second)) {
      path_topology_.erase(it);
      path_update_ = true;
      break;
    }
  }
}

void TopologyLayer::AddTopology(int indx1, int indx2) {
  if (indx1 != indx2) {
    if (indx1 < indx2) {
      path_topology_.push_back(std::pair<int, int>(indx1, indx2));
    } else if (indx1 > indx2) {
      path_topology_.push_back(std::pair<int, int>(indx2, indx1));
    }
    path_update_ = true;
  }
}

void TopologyLayer::Save(VDBManage *vdb) {
  vdb->BeginTransaction();

  vdb->ClearTopologyProperty();

  for (std::map<int, PointElement *>::iterator iter = path_points_.begin();
       iter != path_points_.end(); iter++) {
    char *pMem = NULL;
    int length = PackGeometry(iter->second->GetGeometry(), pMem);

    vdb->SaveTopologyProperty(iter->second->GetUniqueID(), pMem, length);
  }
  vdb->EndTransaction();

  //强制删除无效拓扑
  for (std::vector<std::pair<int, int>>::iterator it = path_topology_.begin();
       it != path_topology_.end();) {
    if (path_points_.find(it->first) == path_points_.end()) {
      it = path_topology_.erase(it);

    } else if (path_points_.find(it->second) == path_points_.end()) {
      it = path_topology_.erase(it);
    } else {
      ++it;
    }
  }

  vdb->BeginTransaction();

  //保存拓扑
  vdb->ClearTopologyEdge();

  for (std::vector<std::pair<int, int>>::iterator iter = path_topology_.begin();
       iter != path_topology_.end(); iter++) {
    vdb->SaveTopologyEdge(iter->first, iter->second);
  }
  vdb->EndTransaction();
}

void TopologyLayer::ClearLayer() {
  for (std::map<int, PointElement *>::iterator iter = path_points_.begin();
       iter != path_points_.end(); iter++) {
    delete iter->second;
  }
  path_points_.clear();
  path_update_ = true;
}

void TopologyLayer::Read(VDBManage *vdb) {
  ClearLayer();

  std::vector<Sign *> areaArray;
  vdb->ReadTopologyProperty(areaArray);

  for (int i = 0; i < areaArray.size(); i++) {
    PointElement *pLaneSegment = new PointElement();

    Geometry *pPolyline = areaArray[i]->polyline;

    pLaneSegment->SetGeometry(areaArray[i]->polyline);
    pLaneSegment->SetUniqueID(areaArray[i]->uniqueId);

    AddMapFeature(pLaneSegment);

    delete areaArray[i];
  }

  std::vector<std::pair<int, int>> edgeArray;
  vdb->ReadTopologyEdge(edgeArray);
  for (int i = 0; i < edgeArray.size(); i++) {
    path_topology_.push_back(edgeArray[i]);
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

int TopologyLayer::PackGeometry(Geometry *pPolyline, char *&pMem) {
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
