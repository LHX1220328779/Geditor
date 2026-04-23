
#include "core/boundary_layer.h"

#include <time.h>

#include <algorithm>

#include "core/factory_drawable.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon_hole.h"
#include "core/geo_polyline.h"
#include "core/geo_rectangle.h"
#include "map/projection_utm.h"
#include "map/vdb_manage.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_hdw_vertex_buffer.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"
#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"

namespace geditor {

BoundaryLayer::BoundaryLayer() : Layer(LT_BOUNDARY) {}

BoundaryLayer::~BoundaryLayer() {}

//添加地图要素
void BoundaryLayer::AddMapFeature(MapFeature *feature) {
  feature->SetMapLayer(this);

  if (feature->GetUniqueID() == 0) {
    int uniqueId = GenerateFeatureID();
    feature->SetUniqueID(uniqueId);
  }

  lane_segment_.push_back((BoundSegment *)feature);
}

MapFeature *BoundaryLayer::GetFeature(int idx) {
  return GetFeatureVec(idx, lane_segment_);
}

GeoPolygon *CombinationPolygon(const std::vector<Geometry *> &outlines) {
  if (outlines.size() > 0) {
    GeoPolygon *pPolygon = new GeoPolygon();

    int iCount = outlines[0]->GetVertexCount();
    for (int i = 0; i < iCount; i++) {
      Point3d point = outlines[0]->GetVertex(i);
      pPolygon->AppendVertex(point);
    }

    //----------------------------
    std::vector<Geometry *> lineStrings(outlines.begin() + 1, outlines.end());

    while (!lineStrings.empty()) {
      bool bFilp;
      double minDist = 10000;
      std::vector<Geometry *>::iterator it_find;

      //查找下一条线段
      Point3d ptEnd = pPolygon->GetEndVertex();
      for (std::vector<Geometry *>::iterator it = lineStrings.begin();
           it != lineStrings.end(); it++) {
        Point3d ptS = (*it)->GetStartVertex();
        Point3d ptE = (*it)->GetEndVertex();

        double distS = Point3d::Distance(ptEnd, ptS);
        double distE = Point3d::Distance(ptEnd, ptE);

        if (distE > distS) {
          if (distS < minDist) {
            bFilp = false;
            minDist = distS;
            it_find = it;
          }
        } else {
          if (distE < minDist) {
            bFilp = true;
            minDist = distE;
            it_find = it;
          }
        }
      }

      //拼接下一条线段
      Geometry *pGeo = (*it_find);
      lineStrings.erase(it_find);

      if (minDist < 0.3) {
        if (bFilp) {
          pGeo->ReverseVertex();
        }

        int iCount = pGeo->GetVertexCount();
        for (int i = 1; i < iCount; i++) {
          Point3d point = pGeo->GetVertex(i);
          pPolygon->AppendVertex(point);
        }
      } else {
        return NULL;
      }
    }

    return pPolygon;
  }

  return NULL;
}

void BoundaryLayer::GetBoundaryArea(std::vector<Geometry *> &boundaryArea) {
  std::vector<Geometry *> outlines;
  std::map<int, std::vector<Geometry *>> interlines;

  for (std::vector<BoundSegment *>::iterator it = lane_segment_.begin();
       it != lane_segment_.end(); it++) {
    BoundaryProperty *pProperty = (*it)->GetProperty();
    Geometry *pGeometry = (*it)->GetGeometry();

    //---------------------------------------------------------------

    unsigned int bound_type = pProperty->boundType & 0x0000FF;
    unsigned int outline_type = (pProperty->boundType & 0x00FF00) >> 8;
    unsigned int bound_seq = (pProperty->boundType & 0xFF0000) >> 16;

    if (outline_type) {
      outlines.push_back(pGeometry);
    } else {
      auto iter = interlines.find(bound_seq);
      if (iter != interlines.end()) {
        iter->second.push_back(pGeometry);
      } else {
        std::vector<Geometry *> vecArr(1, pGeometry);
        interlines.insert(
            std::pair<int, std::vector<Geometry *>>(bound_seq, vecArr));
      }
    }
  }

  //生成外轮廓
  GeoPolygon *pPolygon = CombinationPolygon(outlines);
  if (pPolygon != NULL) {
    if (interlines.size() > 0) {
      GeoPolygonHole *pGeoPolyonHole = new GeoPolygonHole;

      //外轮廓
      if (pPolygon != NULL) {
        int iCount = pPolygon->GetVertexCount();
        for (int i = 0; i < iCount; i++) {
          Point3d pt = pPolygon->GetVertex(i);
          pGeoPolyonHole->AppendVertex(pt);
        }
      }

      //内轮廓
      for (int i = 0; i < interlines.size(); i++) {
        GeoPolygon *pPolygon = CombinationPolygon(interlines[i]);
        if (pPolygon != NULL) {
          pGeoPolyonHole->AppendHole(pPolygon);
        }
      }
      boundaryArea.push_back(pGeoPolyonHole);
    } else {
      boundaryArea.push_back(pPolygon);
    }
  }
}

//删除地图要素
bool BoundaryLayer::DeleteMapFeature(MapFeature *polyline) {
  for (std::vector<BoundSegment *>::iterator it = lane_segment_.begin();
       it != lane_segment_.end(); it++) {
    if (polyline == (*it)) {
      lane_segment_.erase(it);
      return true;
    }
  }
  return false;
}

bool BoundaryLayer::GetBoundary(V3d &vMin, V3d &vMax) {
  if (lane_segment_.size() > 0) {
    for (std::vector<BoundSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++) {
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

//拾取对象
bool BoundaryLayer::PickupObject(const Point3d &pnt3d, double tolerance,
                                 PickupResult &result) {
  bool bFind = false;

  for (std::vector<BoundSegment *>::iterator it = lane_segment_.begin();
       it != lane_segment_.end(); it++) {
    Point3d outpnt;
    double fal;

    Geometry *pGeomLine = (*it)->GetGeometry();
    if (pGeomLine->GetGeometryType() == Geometry::GT_POLYLINE ||
        pGeomLine->GetGeometryType() == Geometry::GT_BEZIER_CURVE ||
        pGeomLine->GetGeometryType() == Geometry::GT_ARC_LINE ||
        pGeomLine->GetGeometryType() == Geometry::GT_BSPLINE_CURVE) {
      int index = pGeomLine->IsPointInEdge(pnt3d, outpnt, tolerance, fal);
      if (index > 0) {
        if (fal < result.dDistance) {
          result.pFeatureObject = (*it);

          result.ptNearPoint = outpnt;
          result.nSegmentIdx = index;
          result.dDistance = fal;

          bFind = true;
        }
      }
    }
    // if (pGeomLine->GetGeometryType() == Geometry::GT_POLYLINE) {
    //   GeoPolyline *polyline = dynamic_cast<GeoPolyline *>(pGeomLine);
    //   int index = polyline->IsPointInEdge(pnt3d, outpnt, tolerance, fal);
    //   if (index > 0) {
    //     if (fal < result.dDistance) {
    //       result.pFeatureObject = (*it);

    //       result.ptNearPoint = outpnt;
    //       result.nSegmentIdx = index;
    //       result.dDistance = fal;

    //       bFind = true;
    //     }
    //   }
    // } else if (pGeomLine->GetGeometryType() == Geometry::GT_HERMITE_CURVE) {
    //   LOG(ERROR) << "Hermite curve not supported yet";
    //   // Point3d outpnt;
    //   // GeoHermiteCurve3 *polyline = dynamic_cast<GeoHermiteCurve3
    //   // *>(pGeomLine); int index = polyline->IsPointInEdge(pnt3d, outpnt,
    //   // tolerance, fal); if (index > 0) {
    //   //     if (fal < reslut.dDistance) {
    //   //         reslut.pFeatureObject = (*it);

    //   //         reslut.ptNearPoint = outpnt;
    //   //         reslut.nSegmentIdx = index;
    //   //         reslut.dDistance = fal;

    //   //         bFind = true;
    //   //     }
    //   // }
    // } else if (pGeomLine->GetGeometryType() == Geometry::GT_BEZIER_CURVE) {
    //   Point3d outpnt;
    //   GeoBezierCurve3 *polyline = dynamic_cast<GeoBezierCurve3 *>(pGeomLine);
    //   int index = polyline->IsPointInEdge(pnt3d, outpnt, tolerance, fal);
    //   if (index > 0) {
    //     if (fal < result.dDistance) {
    //       result.pFeatureObject = (*it);
    //       result.ptNearPoint = outpnt;
    //       result.nSegmentIdx = index;
    //       result.dDistance = fal;

    //       bFind = true;
    //     }
    //   }
    // } else if (pGeomLine->GetGeometryType() == Geometry::GT_CIRCULAR_ARC) {
    //   Point3d outpnt;
    //   GeoCircularArc *polyline = dynamic_cast<GeoCircularArc *>(pGeomLine);
    //   int index = polyline->IsPointInEdge(pnt3d, outpnt, tolerance, fal);
    //   if (index > 0) {
    //     if (fal < result.dDistance) {
    //       result.pFeatureObject = (*it);

    //       result.ptNearPoint = outpnt;
    //       result.nSegmentIdx = index;
    //       result.dDistance = fal;

    //       bFind = true;
    //     }
    //   }
    // } else if (pGeomLine->GetGeometryType() == Geometry::GT_BSPLINE_CURVE) {
    //   Point3d outpnt;
    //   GeoBSplineCurve3 *polyline = dynamic_cast<GeoBSplineCurve3
    //   *>(pGeomLine); int index = polyline->IsPointInEdge(pnt3d, outpnt,
    //   tolerance, fal); if (index > 0) {
    //     if (fal < result.dDistance) {
    //       result.pFeatureObject = (*it);

    //       result.ptNearPoint = outpnt;
    //       result.nSegmentIdx = index;
    //       result.dDistance = fal;

    //       bFind = true;
    //     }
    //   }
    // }
  }

  if (bFind) {
    int nIndex =
        OnPoint(result.pFeatureObject->GetGeometry(), pnt3d, tolerance);
    result.nKeyPoint = nIndex;
  }

  return bFind;
}

int BoundaryLayer::OnPoint(Geometry *geometry, const Point3d &Q,
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

void BoundaryLayer::MergeBoundary(std::vector<BoundSegment *> arraySegment) {
  int iCount = arraySegment.size();
  for (int i = 0; i < iCount - 1; i++) {
    for (int j = 1 + i; j < iCount; j++) {
      Geometry *pLine1 = arraySegment[i]->GetGeometry();
      Geometry *pLine2 = arraySegment[j]->GetGeometry();

      Geometry *pMergeLine = MergeGeometry(pLine1, pLine2);
      if (pMergeLine != NULL) {
        DeleteMapFeature(arraySegment[j]);
        return;
      }
    }
  }
}

Geometry *BoundaryLayer::MergeGeometry(Geometry *pLine1, Geometry *pLine2) {
  if (pLine1 != nullptr && pLine2 != nullptr) {
    Point3d p1SV = pLine1->GetStartVertex();
    Point3d p1EV = pLine1->GetEndVertex();

    Point3d p2SV = pLine2->GetStartVertex();
    Point3d p2EV = pLine2->GetEndVertex();

    double dDist;
    dDist = Point3d::Distance(p1SV, p2SV);
    if (dDist < 0.10) {
      pLine1->ReverseVertex();

      int iCount = pLine2->GetVertexCount();
      for (int i = 0; i < iCount; i++) {
        const Point3d &point = pLine2->GetVertex(i);
        pLine1->AppendVertex(point);
      }
      return pLine1;
    }

    dDist = Point3d::Distance(p1EV, p2EV);
    if (dDist < 0.10) {
      int iCount = pLine2->GetVertexCount();
      for (int i = iCount - 1; i >= 0; i--) {
        const Point3d &point = pLine2->GetVertex(i);
        pLine1->AppendVertex(point);
      }
      return pLine1;
    }

    dDist = Point3d::Distance(p1SV, p2EV);
    if (dDist < 0.10) {
      pLine1->ReverseVertex();

      int iCount = pLine2->GetVertexCount();
      for (int i = iCount - 1; i >= 0; i--) {
        const Point3d &point = pLine2->GetVertex(i);
        pLine1->AppendVertex(point);
      }
      return pLine1;
    }

    dDist = Point3d::Distance(p1EV, p2SV);
    if (dDist < 0.10) {
      int iCount = pLine2->GetVertexCount();
      for (int i = 0; i < iCount; i++) {
        const Point3d &point = pLine2->GetVertex(i);
        pLine1->AppendVertex(point);
      }

      return pLine1;
    }
  }
  return NULL;
}

void BoundaryLayer::BreakPolyline(MapFeature *pObject, int index,
                                  const Point3d &nearPnt3d) {
  if (pObject != NULL) {
    Geometry *pline = pObject->GetGeometry();

    Geometry *pNewLine = new GeoPolyline();
    pNewLine->AppendVertex(nearPnt3d);
    for (int i = index; i < pline->GetVertexCount(); i++) {
      const Point3d &pnt = pline->GetVertex(i);
      pNewLine->AppendVertex(pnt);
    }

    pline->Resize(index);
    pline->AppendVertex(nearPnt3d);

    int uniqueId = GenerateFeatureID();
    BoundSegment *pLaneSegment = new BoundSegment();
    pLaneSegment->SetUniqueID(uniqueId);
    pLaneSegment->SetGeometry(pNewLine);

    AddMapFeature(pLaneSegment);
  }
}

void BoundaryLayer::AddMapObject(std::vector<Lane *> segmentArray) {
  for (int i = 0; i < segmentArray.size(); i++) {
    BoundSegment *pLaneSegment = new BoundSegment();

    Geometry *pPolyline = segmentArray[i]->polyline;

    pLaneSegment->SetGeometry(segmentArray[i]->polyline);
    pLaneSegment->SetUniqueID(segmentArray[i]->uniqueId);

    BoundaryProperty boundProperty;
    boundProperty.length = segmentArray[i]->pProperty.length;
    boundProperty.boundType = segmentArray[i]->pProperty.rightBorderType;
    boundProperty.mineSegmentIndex = segmentArray[i]->pProperty.mineSegmentIndex;
    std::strncpy(boundProperty.mineSegmentCode,
                 segmentArray[i]->pProperty.mineSegmentCode,
                 sizeof(boundProperty.mineSegmentCode) - 1);
    pLaneSegment->SetProperty(&boundProperty);

    AddMapFeature(pLaneSegment);

    delete segmentArray[i];
  }
}

void BoundaryLayer::ClearLayer() {
  for (std::vector<BoundSegment *>::iterator iter = lane_segment_.begin();
       iter != lane_segment_.end(); iter++) {
    delete (*iter);
  }
  lane_segment_.clear();
}

void BoundaryLayer::Read(VDBManage *vdb) {
  if (vdb != NULL) {
    ClearLayer();

    std::vector<Lane *> segmentArray;
    if (vdb->ReadBoundaryProperty(segmentArray)) {
      AddMapObject(segmentArray);
    }
  }
}

void BoundaryLayer::Save(VDBManage *vdb) {
  if (vdb != NULL) {
    vdb->ClearBoundaryProperty();

    for (std::vector<BoundSegment *>::iterator iter = lane_segment_.begin();
         iter != lane_segment_.end(); iter++) {
      BoundSegment *pLaneSegment = (*iter);

      char *pMem = NULL;
      int length = PackGeometry(pLaneSegment->GetGeometry(), pMem);
      if (length > 0) {
        vdb->SaveBoundaryProperty(pLaneSegment->GetUniqueID(),
                                  pLaneSegment->GetProperty(), pMem, length);
      }
    }
  }
}

V4f GetLineColor(BoundSegment *pFeature) {
  return pFeature->GetStyle()->GetLineColor();
  BoundaryProperty *pProperty = pFeature->GetProperty();

  unsigned int outinter_type = (pProperty->boundType & 0x00FF00) >> 8;
  unsigned int outline_type = (pProperty->boundType & 0x00000ff);

  float alpha = outinter_type ? 1.0 : 0.35;

  if (outline_type == 1) {
    return V4f(alpha, 0.0, 0.0, 1.0);
  } else if (outline_type == 2) {
    return V4f(alpha, alpha, 0.0, 1.0);
  } else if (outline_type == 3) {
    return V4f(0.0, alpha, 0.0, 1.0);
  } else {
    return V4f(alpha, alpha, alpha, 1.0);
  }
}

//几何对象转换成渲染对象
void BoundaryLayer::Cull(double minX, double minY, double maxX, double maxY) {
  for (std::map<Geometry *, PositionTransformNode *>::iterator iter =
           render_leaf_.begin();
       iter != render_leaf_.end();) {
    bool bFind = false;

    for (std::vector<BoundSegment *>::iterator it = lane_segment_.begin();
         it != lane_segment_.end(); it++) {
      Geometry *pPolyline = (*it)->GetGeometry();
      if (pPolyline == iter->first) {
        bFind = true;
        break;
      }
    }

    if (!bFind) {
      std::map<Geometry *, PositionTransformNode *>::iterator it_delete =
          iter++;
      render_leaf_.erase(it_delete);
    } else {
      iter++;
    }
  }

  for (std::vector<BoundSegment *>::iterator it = lane_segment_.begin();
       it != lane_segment_.end(); it++) {
    BoundSegment *pLaneSegment = (*it);

    Geometry *pPolyline = (*it)->GetGeometry();
    std::map<Geometry *, PositionTransformNode *>::iterator it_transformNode =
        render_leaf_.find(pPolyline);

    PositionTransformNode *pTransformNode = NULL;

    if (render_leaf_.end() == it_transformNode) {
      pTransformNode = new PositionTransformNode();
      render_leaf_.insert(std::pair<Geometry *, PositionTransformNode *>(
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

      PolyLineSytle *pStyle = pLaneSegment->GetStyle();
      int highlightPoint = pLaneSegment->GetHighlightPoint();

      V3f vclr = pStyle->GetKeyVertexColor();

      Drawable *pNode = FactoryDrawable::CreateNodeDrawable(
          pPolyline, vCnt, 0.1f, V4f(vclr[0], vclr[1], vclr[2], 1.0),
          highlightPoint);
      if (pNode != NULL) {
        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(1);

        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pNode);
        pRenderLeaf->SetRenderTechnique(pTechnique);

        pTransformNode->AddChild(pRenderLeaf);
      }

      V4f vlineclr = GetLineColor(*it);

      Drawable *pLine =
          FactoryDrawable::CreateLineDrawable(pPolyline, vCnt, vlineclr);
      if (pLine != NULL) {
        RenderTechnique *pTechnique =
            TechniqueManager::GetInstance()->GetTechnique(4);

        RenderLeaf *pRenderLeaf = new RenderLeaf();
        pRenderLeaf->SetDrawable(pLine);
        pRenderLeaf->SetRenderTechnique(pTechnique);

        pTransformNode->AddChild(pRenderLeaf);
      }
    }
  }
}

void BoundaryLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera) {
  for (std::map<Geometry *, PositionTransformNode *>::iterator it =
           render_leaf_.begin();
       it != render_leaf_.end(); it++) {
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

void BoundaryLayer::Draw(RenderInfo &rendinfo) {
  if (!m_bVisible) return;
  for (std::map<Geometry *, PositionTransformNode *>::iterator it =
           render_leaf_.begin();
       it != render_leaf_.end(); it++) {
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

int BoundaryLayer::PackGeometry(Geometry *pPolyline, char *&pMem) {
  int rawCount = pPolyline->GetVertexCount();
  if (rawCount < 2) {
    return 0;
  }

  //---------------------------------------------
  GEO_HDR hdr;

  hdr.magic = 0x00306567;
  hdr.version = 1;
  hdr.reserve = 0;
  hdr.crc32 = 0;

  //------------------------------------

  GEO_INFO info;

  std::vector<Point3d> items;
  pPolyline->Hermite(items);
  Geometry::GeometryType type = pPolyline->GetGeometryType();
  // if (type == Geometry::GT_BEZIER_CURVE) {
  //   GeoBezierCurve3 *pCurve = (GeoBezierCurve3 *)pPolyline;
  //   pCurve->Hermite(items);
  // } else if (type == Geometry::GT_BSPLINE_CURVE) {
  //   GeoBSplineCurve3 *pCurve = (GeoBSplineCurve3 *)pPolyline;
  //   pCurve->Hermite(items);
  // } else if (type == Geometry::GT_CIRCULAR_ARC) {
  //   GeoCircularArc *pCurve = (GeoCircularArc *)pPolyline;
  //   pCurve->Hermite(items);
  // } else if (type == Geometry::GT_HERMITE_CURVE) {
  //   LOG(INFO) << "Hermite curve not supported yet";
  //   // GeoHermiteCurve3 *pCurve = (GeoHermiteCurve3 *) pPolyline;
  //   // pCurve->Hermite(items);
  // } else if (type == Geometry::GT_POLYLINE || type == Geometry::GT_POLYGON) {
  //   for (int i = 0; i < pPolyline->GetVertexCount(); i++) {
  //     Point3d pt = pPolyline->GetVertex(i);
  //     items.push_back(pt);
  //   }
  // } else if (type == Geometry::GT_RECTANGLE) {
  //   GeoRectangle *pCurve = (GeoRectangle *)pPolyline;
  //   pCurve->Hermite(items);
  // }

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