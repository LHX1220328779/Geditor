
#include "core/factory_drawable.h"

#include "core/area_drawable.h"
#include "core/car_body_check.h"
#include "core/compute_curve.h"
#include "core/earcut.h"
#include "core/geo_arc_line.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_polygon_hole.h"
#include "core/job_area.h"
#include "core/line_drawable.h"
#include "core/wline_drawable.h"

namespace geditor {

Drawable *FactoryDrawable::CreateLineDrawable(Geometry *pPolyline,
                                              const V3d &vCnt, const V4f &clr) {
  if (pPolyline->GetGeometryType() == Geometry::GT_POLYGON_HOLE) {
    GeoPolygonHole *pPolygonHole = (GeoPolygonHole *)pPolyline;

    LineDrawable *pLine = new LineDrawable();

    int nCount = pPolyline->GetVertexCount();
    for (int i = 0; i < nCount; i++) {
      Point3d pnt = pPolyline->GetVertex(i);
      pLine->add(pnt.x - vCnt[0], pnt.y - vCnt[1], 0.0,
                 V3f(clr[0], clr[1], clr[2]));
    }

    Point3d pnt = pPolyline->GetVertex(0);
    pLine->add(pnt.x - vCnt[0], pnt.y - vCnt[1], 0.0,
               V3f(clr[0], clr[1], clr[2]));

    pLine->addline(0, nCount + 1);

    int iSize = pPolygonHole->GetHoleCount();
    for (int x = 0; x < iSize; x++) {
      int start = pLine->size();

      GeoPolygon *pPolygon = pPolygonHole->GetHole(x);

      int nCount = pPolygon->GetVertexCount();
      for (int i = 0; i < nCount; i++) {
        Point3d pnt = pPolygon->GetVertex(i);
        pLine->add(pnt.x - vCnt[0], pnt.y - vCnt[1], 0.0,
                   V3f(clr[0], clr[1], clr[2]));
      }

      Point3d pnt = pPolygon->GetVertex(0);
      pLine->add(pnt.x - vCnt[0], pnt.y - vCnt[1], 0.0,
                 V3f(clr[0], clr[1], clr[2]));

      pLine->addline(start, nCount + 1);
    }
    return pLine;
  }

  std::vector<Point3d> items;
  pPolyline->Hermite(items);
  if (pPolyline->GetGeometryType() == Geometry::GT_POLYGON)
    items.push_back(items[0]);
  // if (pPolyline->GetGeometryType() == Geometry::GT_POLYLINE) {
  //   int nCount = pPolyline->GetVertexCount();
  //   for (int i = 0; i < nCount; i++) {
  //     Point3d pnt = pPolyline->GetVertex(i);
  //     items.push_back(pnt);
  //   }
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_HERMITE_CURVE) {
  //   LOG(ERROR) << "Hermite curve not implemented yet";
  //   // ((GeoHermiteCurve3 *) pPolyline)->Hermite(items);
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_BEZIER_CURVE) {
  //   ((GeoBezierCurve3 *)pPolyline)->Hermite(items);
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_BSPLINE_CURVE) {
  //   ((GeoBSplineCurve3 *)pPolyline)->Hermite(items);
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_CIRCULAR_ARC) {
  //   ((GeoCircularArc *)pPolyline)->Hermite(items);
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_POLYGON) {
  //   int nCount = pPolyline->GetVertexCount();
  //   for (int i = 0; i < nCount; i++) {
  //     Point3d pnt = pPolyline->GetVertex(i);
  //     items.push_back(pnt);
  //   }
  //   Point3d pnt = pPolyline->GetVertex(0);
  //   items.push_back(pnt);
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_RECTANGLE) {
  //   int nCount = pPolyline->GetVertexCount();
  //   if (nCount > 1) {
  //     for (int i = 0; i < 2; i++) {
  //       Point3d pnt = pPolyline->GetVertex(i);
  //       items.push_back(pnt);
  //     }

  //     Point3d pnt = pPolyline->GetVertex(0);
  //     items.push_back(pnt);
  //   }
  // } else if (pPolyline->GetGeometryType() == Geometry::GT_ARC_LINE) {
  //   ((GeoArcLine *)pPolyline)->Hermite(items);
  // }

  if (items.size() > 1) {
    LineDrawable *pLine = new LineDrawable();
    if (pPolyline->GetGeometryType() == Geometry::GT_POLYLINE ||
        pPolyline->GetGeometryType() == Geometry::GT_BSPLINE_CURVE ||
        pPolyline->GetGeometryType() == Geometry::GT_ARC_LINE)
      for (size_t i = 0; i < items.size(); i++) {
        float r = 0.2 + 0.8 * (items.size() - i) / items.size();
        pLine->add(items[i].x - vCnt[0], items[i].y - vCnt[1], 0.0,
                   V3f(clr[0] * r, clr[1] * r, clr[2] * r));
      }
    else
      for (size_t i = 0; i < items.size(); i++) {
        pLine->add(items[i].x - vCnt[0], items[i].y - vCnt[1], 0.0,
                   V3f(clr[0], clr[1], clr[2]));
      }

    return pLine;
  }

  return NULL;
}

Drawable *FactoryDrawable::CreateArrowDrawable(Geometry *pPolyline,
                                               const V3d &vCnt, const V4f &clr,
                                               float size, float r) {
  std::vector<Point3d> items;
  pPolyline->Hermite(items);
  int nCount = items.size();
  if (nCount > 1) {
    std::vector<V3f> positon;
    std::vector<int> indices_array;

    Point3d start = items[0];
    for (int i = 1; i < nCount; i++) {
      if (Point3d::Distance(items[i], start) < 3 && (i != nCount - 1)) continue;
      if (positon.size() > 0 && i == nCount - 1 &&
          pPolyline->GetGeometryType() != Geometry::GT_POLYLINE)
        continue;
      Point3d end = items[i];
      start = items[i - 1];
      V3f dir(end.x - start.x, end.y - start.y, end.z - start.z);
      {
        V3d midPiont;

        midPiont[0] = end.x * r + start.x * (1 - r) - vCnt[0];
        midPiont[1] = end.y * r + start.y * (1 - r) - vCnt[1];
        midPiont[2] = end.z * r + start.z * (1 - r) - vCnt[2];

        V3f orgin(midPiont[0], midPiont[1], midPiont[2]);
        V3f vDir = Normalize(dir);

        V3f start = orgin + vDir * (size * (1 - r) * 2);
        V3f end = orgin - vDir * (size * (r * 2));

        V3f vec = dir.cross(V3f(0.0f, 0.0f, 1.0f));
        vec.normalize();

        V3f left = end + vec * (size * 0.5f);
        V3f right = end - vec * (size * 0.5f);

        int startIdx = positon.size();
        positon.push_back(start);
        positon.push_back(left);
        positon.push_back(right);

        indices_array.push_back(startIdx);
        indices_array.push_back(startIdx + 1);
        indices_array.push_back(startIdx + 2);
      }
      start = end;
    }

    //---------------------------------

    //���ɶ�������
    VertexFormat *vertexformat = new VertexFormat();
    vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                  VertexFormat::MCAT_FLOAT4, 0);

    VertexBuffer *vertexBuffer =
        new VertexBuffer(positon.size(), vertexformat->GetStride());
    VertexBufferAccessor vba(vertexformat, vertexBuffer);

    //������������bufffer
    for (size_t i = 0; i < positon.size(); i++) {
      vba.Position<V3f>(i) = positon[i];
      vba.Color<V4f>(0, i) = clr;
    }

    //������������
    IndexBuffer *indexBuffer =
        new IndexBuffer(indices_array.size(), sizeof(int));
    int *indices = (int *)indexBuffer->GetData();
    for (size_t i = 0; i < indices_array.size(); i++) {
      indices[i] = indices_array[i];
    }

    AreaDrawable *pLine = new AreaDrawable();

    pLine->m_pVertexBuffer = vertexBuffer;
    pLine->m_pVertexformat = vertexformat;
    pLine->m_pIndexBuffer = indexBuffer;

    return pLine;
  }

  return NULL;
}

Drawable *FactoryDrawable::CreateNodeDrawable(Geometry *pPolyline,
                                              const V3d &vCnt, float size,
                                              const V4f &clr, int hlid) {
  std::vector<Point3d> items;

  int nCount = pPolyline->GetVertexCount();
  for (int i = 0; i < nCount; i++) {
    Point3d pnt = pPolyline->GetVertex(i);
    items.push_back(pnt);
  }

  if (pPolyline->GetGeometryType() == Geometry::GT_POLYGON_HOLE) {
    GeoPolygonHole *pPolygon = (GeoPolygonHole *)pPolyline;

    int iSize = pPolygon->GetHoleCount();
    for (int x = 0; x < iSize; x++) {
      GeoPolygon *pHole = pPolygon->GetHole(x);

      int nCount = pHole->GetVertexCount();
      for (int i = 0; i < nCount; i++) {
        Point3d pnt = pHole->GetVertex(i);
        items.push_back(pnt);
      }
    }
  }

  if (items.size() > 0) {
    std::vector<V3f> position;
    std::vector<int> indices_array;

    int nCount = items.size();
    for (int i = 0; i < nCount; i++) {
      Point3d pnt = items[i];

      V3f origin;
      origin[0] = pnt.x - vCnt[0];
      origin[1] = pnt.y - vCnt[1];
      origin[2] = pnt.z - vCnt[2];

      int startIdx = position.size();

      position.push_back(V3f(origin[0] - size, origin[1] - size, origin[2]));
      position.push_back(V3f(origin[0] + size, origin[1] - size, origin[2]));
      position.push_back(V3f(origin[0] + size, origin[1] + size, origin[2]));
      position.push_back(V3f(origin[0] - size, origin[1] + size, origin[2]));

      indices_array.push_back(startIdx);
      indices_array.push_back(startIdx + 1);
      indices_array.push_back(startIdx + 3);

      indices_array.push_back(startIdx + 3);
      indices_array.push_back(startIdx + 1);
      indices_array.push_back(startIdx + 2);
    }

    //---------------------------------------------------------------

    //���ɶ�������
    VertexFormat *vertexformat = new VertexFormat();
    vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                  VertexFormat::MCAT_FLOAT4, 0);

    VertexBuffer *vertexBuffer =
        new VertexBuffer(position.size(), vertexformat->GetStride());
    VertexBufferAccessor vba(vertexformat, vertexBuffer);

    //������������bufffer
    for (size_t i = 0; i < position.size(); i++) {
      vba.Position<V3f>(i) = position[i];
      if (i / 4 == hlid)
        vba.Color<V4f>(0, i) = V4f(1, 0, 0, clr[3]);
      else
        vba.Color<V4f>(0, i) = clr;
    }

    //������������
    IndexBuffer *indexBuffer =
        new IndexBuffer(indices_array.size(), sizeof(int));
    int *indices = (int *)indexBuffer->GetData();
    for (size_t i = 0; i < indices_array.size(); i++) {
      indices[i] = indices_array[i];
    }

    AreaDrawable *pLine = new AreaDrawable();

    pLine->m_pVertexBuffer = vertexBuffer;
    pLine->m_pVertexformat = vertexformat;
    pLine->m_pIndexBuffer = indexBuffer;

    return pLine;
  }
  return NULL;
}

void *stdAlloc(void *userData, unsigned int size) {
  int *allocated = (int *)userData;
  // TESS_NOTUSED(userData);
  *allocated += (int)size;
  return malloc(size);
}

void stdFree(void *userData, void *ptr) {
  // TESS_NOTUSED(userData);
  free(ptr);
}

Drawable *FactoryDrawable::CreateAreaDrawable(Geometry *pPolyline,
                                              const V3d &vCnt, const V4f &clr) {
  int nCount = pPolyline->GetVertexCount();

  if (pPolyline->GetGeometryType() == Geometry::GT_RECTANGLE) {
    if (nCount > 1) {
      Point3d sPnt = pPolyline->GetVertex(0);
      Point3d ePnt = pPolyline->GetVertex(1);

      V3d vDir(ePnt.x - sPnt.x, ePnt.y - sPnt.y, ePnt.z - sPnt.z);
      vDir.normalize();

      V3d sDir = V3d(sPnt.x, sPnt.y, sPnt.z) - vDir * 3.0;
      V3d eDir = sDir + vDir * 6.0;

      V3d vUp(0, 0, 1);
      V3d vLeft = vDir.cross(vUp);

      V3d vPoints[4];
      vPoints[0] = sDir + vLeft * 1.5;
      vPoints[1] = eDir + vLeft * 1.5;
      vPoints[2] = eDir - vLeft * 1.5;
      vPoints[3] = sDir - vLeft * 1.5;

      VertexFormat *vertexformat = new VertexFormat();
      vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                    VertexFormat::MCAT_FLOAT3, 0);
      vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                    VertexFormat::MCAT_FLOAT4, 0);

      VertexBuffer *vertexBuffer =
          new VertexBuffer(4, vertexformat->GetStride());
      VertexBufferAccessor vba(vertexformat, vertexBuffer);

      for (int i = 0; i < 4; i++) {
        vba.Position<V3f>(i) =
            V3f(vPoints[i][0] - vCnt[0], vPoints[i][1] - vCnt[1],
                vPoints[i][2] - vCnt[2]);
        vba.Color<V4f>(0, i) = clr;
      }

      IndexBuffer *indexBuffer = new IndexBuffer(6, sizeof(int));
      int *indices = (int *)indexBuffer->GetData();
      indices[0] = 0;
      indices[1] = 1;
      indices[2] = 2;
      indices[3] = 0;
      indices[4] = 2;
      indices[5] = 3;

      AreaDrawable *pLine = new AreaDrawable();

      pLine->m_pVertexBuffer = vertexBuffer;
      pLine->m_pVertexformat = vertexformat;
      pLine->m_pIndexBuffer = indexBuffer;

      return pLine;
    }

  } else if (pPolyline->GetGeometryType() == Geometry::GT_POLYGON) {
    if (nCount > 2) {
      std::vector<V3f> vertices_3d;
      using Point = std::array<double, 2>;
      std::vector<std::vector<V2f>> vertices_2d;

      vertices_3d.resize(nCount);
      vertices_2d.resize(2);
      vertices_2d[0].resize(nCount);

      for (int i = 0; i < nCount; i++) {
        Point3d pnt = pPolyline->GetVertex(i);
        vertices_3d[i] = V3f(pnt.x - vCnt[0], pnt.y - vCnt[1], pnt.z - vCnt[2]);
        vertices_2d[0][i] = V2f(pnt.x - vCnt[0], pnt.y - vCnt[1]);
      }

      AreaDrawable *pLine = nullptr;
      std::vector<int> indices = mapbox::earcut<int>(vertices_2d);

      if (indices.size() > 0) {
        int n_vertex = vertices_3d.size();

        VertexFormat *vertexformat = new VertexFormat();
        vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                      VertexFormat::MCAT_FLOAT3, 0);
        vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                      VertexFormat::MCAT_FLOAT4, 0);

        VertexBuffer *vertexBuffer =
            new VertexBuffer(n_vertex, vertexformat->GetStride());
        VertexBufferAccessor vba(vertexformat, vertexBuffer);

        // 索引buffer
        int inPos = 0;
        IndexBuffer *indexBuffer = new IndexBuffer(indices.size(), sizeof(int));
        int *ibuffer_data = (int *)indexBuffer->GetData();
        for (int i = 0; i < indices.size(); i++) {
          ibuffer_data[i] = indices[i];
        }

        // 顶点buffer
        for (int i = 0; i < n_vertex; i++) {
          vba.Position<V3f>(i) = vertices_3d[i];
          vba.Color<V4f>(0, i) = clr;
        }

        pLine = new AreaDrawable();

        pLine->m_pVertexBuffer = vertexBuffer;
        pLine->m_pVertexformat = vertexformat;
        pLine->m_pIndexBuffer = indexBuffer;
      }
      return pLine;
    }
  } else if (pPolyline->GetGeometryType() == Geometry::GT_POLYGON_HOLE) {
    GeoPolygonHole *pPolygonHole = (GeoPolygonHole *)pPolyline;
    if (nCount > 2) {
      AreaDrawable *pLine = nullptr;
      /*
      TESSalloc ma;

      int allocated = 0;

      memset(&ma, 0, sizeof(ma));
      ma.memalloc = stdAlloc;
      ma.memfree = stdFree;
      ma.userData = (void *) &allocated;
      ma.extraVertices = 256;

      TESStesselator *tess = tessNewTess(&ma);
      tessSetOption(tess, TESS_REVERSE_CONTOURS, 1);



      //------------------------------------
      {
          //������
          int iSize = pPolygonHole->GetVertexCount();

          std::vector<V3f> vertexs;
          vertexs.resize(iSize);
          for (int i = 0; i < iSize; i++) {
              Point3d pnt = pPolygonHole->GetVertex(i);
              vertexs[i] = V3f(pnt.x - vCnt[0], pnt.y - vCnt[1], pnt.z -
      vCnt[2]);
          }

          tessAddContour(tess, 3, &vertexs[0], sizeof(V3f), iSize);
      }
      {
          //������
          int nHole = pPolygonHole->GetHoleCount();
          for (int x = 0; x < nHole; x++) {
              GeoPolygon *pPolygon = pPolygonHole->GetHole(x);

              int iSize = pPolygon->GetVertexCount();

              std::vector<V3f> vertexs;
              vertexs.resize(iSize);
              for (int i = 0; i < iSize; i++) {
                  Point3d pnt = pPolygon->GetVertex(i);
                  vertexs[i] = V3f(pnt.x - vCnt[0], pnt.y - vCnt[1], pnt.z -
      vCnt[2]);
              }

              tessAddContour(tess, 3, &vertexs[0], sizeof(V3f), iSize);
          }
      }
      //-------------------------------------------

      if (!tessTesselate(tess, TESS_WINDING_ODD, TESS_POLYGONS, 3, 3, 0)) {
          tess = 0;
      }


      //����������
      const float *verts = tessGetVertices(tess);

      //��������
      int nverts = tessGetVertexCount(tess);

      //��ȡ������׵�ַ
      const int *elems = tessGetElements(tess);

      //���������
      int nelems = tessGetElementCount(tess);

      AreaDrawable *pLine = NULL;
      if (nelems > 0) {
          VertexFormat *vertexformat = new VertexFormat();
          vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
      VertexFormat::MCAT_FLOAT3, 0);
          vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
      VertexFormat::MCAT_FLOAT4, 0);

          VertexBuffer *vertexBuffer = new VertexBuffer(nverts,
      vertexformat->GetStride()); VertexBufferAccessor vba(vertexformat,
      vertexBuffer);


          //��������buffer
          int inPos = 0;
          std::vector<int> indices_array;

          for (int i = 0; i < nelems; i++) {
              //ȡ��������������

              for (int j = 0; j < 3; j++) {
                  if (elems[i * 3 + j] != TESS_UNDEF) {
                      indices_array.push_back(elems[i * 3 + j]);
                  } else {
                      bool b = true;
                      break;
                  }
              }
          }

          IndexBuffer *indexBuffer = new IndexBuffer(indices_array.size(),
      sizeof(int)); int *indices = (int *) indexBuffer->GetData();

          for (int i = 0; i < indices_array.size(); i++) {
              indices[i] = indices_array[i];
          }


          //������������bufffer
          V3f *points = (V3f *) verts;
          for (int i = 0; i < nverts; i++) {
              vba.Position<V3f>(i) = points[i];
              vba.Color<V4f>(0, i) = V4f(clr[0], clr[1], clr[2], 0.5);
          }

          pLine = new AreaDrawable();

          pLine->m_pVertexBuffer = vertexBuffer;
          pLine->m_pVertexformat = vertexformat;
          pLine->m_pIndexBuffer = indexBuffer;

      }

      if (tess) {
          tessDeleteTess(tess);
      }
       */

      return pLine;
    }
  }
  return NULL;
}

Drawable *FactoryDrawable::CreateWidthLineDrawable(Geometry *pPolyline,
                                                   const V3d &vCnt,
                                                   double dWidth,
                                                   const V4f &clr) {
  int nCount = pPolyline->GetVertexCount();
  if (nCount > 1) {
    std::vector<Point3d> items;
    pPolyline->Hermite(items);
    // if (pPolyline->GetGeometryType() == Geometry::GT_POLYLINE) {
    //   for (int i = 0; i < nCount; i++) {
    //     Point3d pnt = pPolyline->GetVertex(i);
    //     items.push_back(pnt);
    //   }
    // } else if (pPolyline->GetGeometryType() == Geometry::GT_HERMITE_CURVE) {
    //   LOG(ERROR) << "Hermite curve not supported in this version.";
    //   // ((GeoHermiteCurve3 *) pPolyline)->Hermite(items);
    // } else if (pPolyline->GetGeometryType() == Geometry::GT_BEZIER_CURVE) {
    //   ((GeoBezierCurve3 *)pPolyline)->Hermite(items);
    // } else if (pPolyline->GetGeometryType() == Geometry::GT_BSPLINE_CURVE) {
    //   ((GeoBSplineCurve3 *)pPolyline)->Hermite(items);
    // } else if (pPolyline->GetGeometryType() == Geometry::GT_CIRCULAR_ARC) {
    //   ((GeoCircularArc *)pPolyline)->Hermite(items);
    // } else if (pPolyline->GetGeometryType() == Geometry::GT_ARC_LINE) {
    //   ((GeoArcLine *)pPolyline)->Hermite(items);
    // }

    //=================================================
    std::vector<sPoint> PathPoints;

    ComputeCurve computerCurve;
    computerCurve.FittingCurveBy2Vector(items, PathPoints);

    //-------------------------------------------
    std::vector<V3f> positon;
    std::vector<V3f> normal;
    std::vector<V4f> colors_array;
    std::vector<int> indices_array;

    const double deltAngle = 15.0f * 0.01745329251994329576923690768489;

    size_t iCount = items.size();
    for (size_t i = 0; i < iCount; i++) {
      V3f vUp(0, 0, 1);
      float v_width = dWidth * 0.5;

      V3f orginPt(items[i].x - vCnt[0], items[i].y - vCnt[1],
                  items[i].z - vCnt[2]);

      V4f orginClr(clr[0], clr[1], clr[2], clr[3]);
      if (PathPoints[i].curvature > 1 / 1.6) {
        orginClr = V4f(1.0f, 1.0f, 1.0F, 1.0F);
      }

      //------------------------------------

      if (i == 0) {
        V3f vRight;

        V3f vDir;
        vDir[0] = items[i + 1].x - items[i].x;
        vDir[1] = items[i + 1].y - items[i].y;
        vDir[2] = 0;

        V3f vCross = vDir.cross(vUp);
        vRight = Normalize(vCross);

        vRight[2] = 1;
        positon.push_back(orginPt);
        positon.push_back(orginPt);
        colors_array.push_back(orginClr);
        colors_array.push_back(orginClr);
        normal.push_back(vRight);
        normal.push_back(-vRight);

      } else if (i == items.size() - 1) {
        V3f vRight;

        V3f vDir;
        vDir[0] = items[i].x - items[i - 1].x;
        vDir[1] = items[i].y - items[i - 1].y;
        vDir[2] = 0;

        V3f vCross = vDir.cross(vUp);

        vRight = Normalize(vCross);

        int sz = positon.size();
        vRight[2] = 1;

        positon.push_back(orginPt);
        positon.push_back(orginPt);
        colors_array.push_back(orginClr);
        colors_array.push_back(orginClr);
        normal.push_back(vRight);
        normal.push_back(-vRight);

        int idx01 = sz - 2;
        int idx02 = sz - 1;

        int idx11 = sz;
        int idx12 = sz + 1;

        indices_array.push_back(idx02);
        indices_array.push_back(idx01);
        indices_array.push_back(idx11);

        indices_array.push_back(idx02);
        indices_array.push_back(idx11);
        indices_array.push_back(idx12);
      } else {
        V3f vDir1;
        vDir1[0] = items[i + 1].x - items[i].x;
        vDir1[1] = items[i + 1].y - items[i].y;
        vDir1[2] = 0;

        V3f vDir2;
        vDir2[0] = items[i].x - items[i - 1].x;
        vDir2[1] = items[i].y - items[i - 1].y;
        vDir2[2] = 0;

        V3f temp1 = vDir1.cross(vUp);
        temp1.normalize();

        V3f temp2 = vDir2.cross(vUp);
        temp2.normalize();

        V3f jonin_normal = (temp1 + temp2);
        jonin_normal = Normalize(jonin_normal);
        V3f v_temp_norma = Normalize(vDir2);
        float dot_val = v_temp_norma.dot(jonin_normal);

        double angle = acos(abs(dot_val));
        if (angle < deltAngle) {
          {
            int sz = positon.size();
            temp2[2] = 1;
            positon.push_back(orginPt);
            positon.push_back(orginPt);
            colors_array.push_back(orginClr);
            colors_array.push_back(orginClr);
            normal.push_back(temp2);
            normal.push_back(-temp2);

            indices_array.push_back(sz - 1);
            indices_array.push_back(sz - 2);
            indices_array.push_back(sz);

            indices_array.push_back(sz);
            indices_array.push_back(sz - 1);
            indices_array.push_back(sz + 1);
          }
          {
            int sz = positon.size();
            temp1[2] = 1;
            positon.push_back(orginPt);
            positon.push_back(orginPt);
            colors_array.push_back(orginClr);
            colors_array.push_back(orginClr);
            normal.push_back(temp1);
            normal.push_back(-temp1);

            jonin_normal[2] = 0;
            double dDot = jonin_normal.dot(vDir2);
            if (dDot > 0.0) {
              indices_array.push_back(sz - 2);
              indices_array.push_back(sz);
              indices_array.push_back(sz + 1);

            } else {
              indices_array.push_back(sz + 1);
              indices_array.push_back(sz - 1);
              indices_array.push_back(sz - 2);
            }
          }

        } else {
          v_width = 1.0 / sin(angle);

          int sz = positon.size();

          jonin_normal = jonin_normal * v_width;
          jonin_normal[2] = 1;
          positon.push_back(orginPt);
          positon.push_back(orginPt);
          colors_array.push_back(orginClr);
          colors_array.push_back(orginClr);
          normal.push_back(jonin_normal);
          normal.push_back(-jonin_normal);

          int idx01 = sz - 2;
          int idx02 = sz - 1;

          int idx11 = sz;
          int idx12 = sz + 1;

          indices_array.push_back(idx02);
          indices_array.push_back(idx01);
          indices_array.push_back(idx11);

          indices_array.push_back(idx02);
          indices_array.push_back(idx11);
          indices_array.push_back(idx12);
        }
      }
    }

    VertexFormat *vertexformat = new VertexFormat();
    vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_NORMAL,
                                  VertexFormat::MCAT_FLOAT3, 0);
    vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                  VertexFormat::MCAT_FLOAT4, 0);

    VertexBuffer *vertexBuffer =
        new VertexBuffer(positon.size(), vertexformat->GetStride());
    VertexBufferAccessor vba(vertexformat, vertexBuffer);

    for (size_t i = 0; i < positon.size(); i++) {
      vba.Position<V3f>(i) = positon[i];
      vba.Normal<V3f>(i) = normal[i];
      vba.Color<V4f>(0, i) = colors_array[i];
    }

    IndexBuffer *indexBuffer =
        new IndexBuffer(indices_array.size(), sizeof(int));
    int *indices = (int *)indexBuffer->GetData();
    for (size_t i = 0; i < indices_array.size(); i++) {
      indices[i] = indices_array[i];
    }

    WLineDrawable *pLine = new WLineDrawable();

    pLine->m_pVertexBuffer = vertexBuffer;
    pLine->m_pVertexformat = vertexformat;
    pLine->m_pIndexBuffer = indexBuffer;
    pLine->m_lineParam[0] = dWidth * 0.5;
    return pLine;
  }

  return NULL;
}

void bulidMesh(const std::vector<Point3d> &leftBound,
               const std::vector<Point3d> &rightBound,
               const std::vector<V4f> &leftClr,
               const std::vector<V4f> &rightClr, std::vector<V3f> &vertexs,
               std::vector<V4f> &vColors, std::vector<int> &indices) {
  size_t nverts = leftBound.size();

  vertexs.resize(nverts * 2);
  vColors.resize(nverts * 2);
  for (size_t i = 0; i < nverts; i++) {
    vertexs[i * 2 + 0] = V3f(rightBound[i].x, rightBound[i].y, rightBound[i].z);
    vertexs[i * 2 + 1] = V3f(leftBound[i].x, leftBound[i].y, leftBound[i].z);

    vColors[i * 2 + 0] = rightClr[i];
    vColors[i * 2 + 1] = leftClr[i];
  }

  for (size_t i = 0; i < nverts - 1; i++) {
    int start_pos = i * 2;

    indices.push_back(start_pos);
    indices.push_back(start_pos + 2);
    indices.push_back(start_pos + 1);

    indices.push_back(start_pos + 1);
    indices.push_back(start_pos + 2);
    indices.push_back(start_pos + 3);
  }
}

bool PointInPolygon(Point3d p, GeoPolygon *pGeoPolygon, int nCount) {
  int nCross = 0;
  for (int i = 0; i < nCount; i++) {
    Point3d p1 = pGeoPolygon->GetVertex(i);
    Point3d p2 = pGeoPolygon->GetVertex((i + 1) % nCount);
    // ��� y=p.y �� p1p2 �Ľ���
    if (p1.y == p2.y) continue;
    if (p.y <= Mathd::Min(p1.y, p2.y)) continue;
    if (p.y >= Mathd::Max(p1.y, p2.y)) continue;
    double x =
        (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) +
        p1.x;
    if (x >= p.x) {
      nCross++;
    }
  }
  return (nCross % 2 == 1);
}

bool VertexInJobArea(std::vector<JobArea *> &jobArea, V3f vertex,
                     const V3d &vCnt) {
  int nCount = jobArea.size();
  if (nCount == 0) {
    return true;
  }

  for (int i = 0; i < nCount; i++) {
    MapFeature *pMapFeature = jobArea[i];

    GeoPolygon *pGeoPolygon = (GeoPolygon *)pMapFeature->GetGeometry();
    int nCount = pGeoPolygon->GetVertexCount();

    Point3d p3d(vertex[0] + vCnt[0], vertex[1] + vCnt[1], vertex[2] + vCnt[2]);
    if (PointInPolygon(p3d, pGeoPolygon, nCount)) {
      return true;
    }
  }
  return false;
}

//������Ⱦ����
Drawable *FactoryDrawable::CreateCarBodyLineDrawable(
    Geometry *pPolyline, const V3d &vCnt, double dWidth, const V4f &clr,
    std::vector<JobArea *> &jobArea) {
  int nCount = pPolyline->GetVertexCount();
  if (nCount > 1) {
    std::vector<Point3d> items;

    if (pPolyline->GetGeometryType() == Geometry::GT_POLYLINE) {
      for (int i = 0; i < nCount; i++) {
        Point3d pnt = pPolyline->GetVertex(i);
        items.push_back(pnt);
      }
    } else if (pPolyline->GetGeometryType() == Geometry::GT_HERMITE_CURVE) {
      // ((GeoHermiteCurve3 *) pPolyline)->Hermite(items);
      LOG(ERROR) << "Hermite curve not supported in this version.";
    } else if (pPolyline->GetGeometryType() == Geometry::GT_BEZIER_CURVE) {
      ((GeoBezierCurve3 *)pPolyline)->Hermite(items);
    } else if (pPolyline->GetGeometryType() == Geometry::GT_BSPLINE_CURVE) {
      ((GeoBSplineCurve3 *)pPolyline)->Hermite(items);
    } else if (pPolyline->GetGeometryType() == Geometry::GT_CIRCULAR_ARC) {
      ((GeoCircularArc *)pPolyline)->Hermite(items);
    }

    //=================================================
    std::vector<sPoint> PathPoints;

    ComputeCurve computerCurve;
    computerCurve.FittingCurveBy2Vector(items, PathPoints);

    //-------------------------------------------

    std::vector<V3d> outArray[2];

    CarBodyCheck bodyCheck;
    bodyCheck.OffsetRoadPath(PathPoints, outArray[0], outArray[1]);

    std::vector<Point3d> leftBound[2];
    std::vector<Point3d> rightBound[2];
    std::vector<V4f> leftClr[2];
    std::vector<V4f> rightClr[2];

    for (int i = 0; i < 2; i++) {
      bool bRight = (i % 2 == 0);

      std::vector<V3d> &offsetArray = outArray[i];
      for (size_t x = 0; x < offsetArray.size() - 1; x++) {
        Point3d carPos;
        carPos.x = offsetArray[x][0] - vCnt[0];
        carPos.y = offsetArray[x][1] - vCnt[1];
        carPos.z = offsetArray[x][2] - vCnt[2];

        double det_x = PathPoints[x + 1].x - PathPoints[x].x;
        double det_y = PathPoints[x + 1].y - PathPoints[x].y;
        if ((det_x > 0.00003 || det_x < -0.00003) &&
            (det_y < -0.00003 || det_y > 0.00003)) {
          std::vector<Point3d> carPoint =
              bodyCheck.GetAABBPoints(carPos, Point3d(det_x, det_y, 0.0));

          V4f orginClr = clr;
          if (PathPoints[x].curvature > 1 / 1.6) {
            orginClr = V4f(1.0f, 1.0f, 1.0f, 1.0f);
          }

          if (bRight) {
            rightBound[0].push_back(carPoint[1]);
            rightBound[1].push_back(carPoint[2]);

            rightClr[0].push_back(orginClr);
            rightClr[1].push_back(orginClr);
          } else {
            leftBound[0].push_back(carPoint[0]);
            leftBound[1].push_back(carPoint[3]);
            leftClr[0].push_back(orginClr);
            leftClr[1].push_back(orginClr);
          }
        }
      }
    }

    // ----------------------------------------

    AreaDrawable *pLine = NULL;
    if (leftBound[0].size() > 0 && rightBound[1].size() > 0) {
      std::vector<V3f> vertexs[2];
      std::vector<V4f> vColors[2];
      std::vector<int> indices[2];

      bulidMesh(leftBound[0], rightBound[0], rightClr[0], rightClr[0],
                vertexs[0], vColors[0], indices[0]);
      bulidMesh(leftBound[1], rightBound[1], leftClr[1], rightClr[1],
                vertexs[1], vColors[1], indices[1]);

      if (indices[0].size() < 3 || indices[1].size() < 3) {
        return pLine;
      }

      VertexFormat *vertexformat = new VertexFormat();
      vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION,
                                    VertexFormat::MCAT_FLOAT3, 0);
      vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR,
                                    VertexFormat::MCAT_FLOAT4, 0);

      //
      int vertex_count = vertexs[0].size() + vertexs[1].size();
      VertexBuffer *vertexBuffer =
          new VertexBuffer(vertex_count, vertexformat->GetStride());

      int index_count = indices[0].size() + indices[1].size();
      IndexBuffer *indexBuffer = new IndexBuffer(index_count, sizeof(int));

      VertexBufferAccessor vba(vertexformat, vertexBuffer);
      int *indices_array = (int *)indexBuffer->GetData();

      //����ƫ��
      size_t offset = vertexs[0].size();
      for (size_t i = 0; i < indices[1].size(); i++) {
        indices[1][i] += offset;
      }

      int pos_start = 0, idx_start = 0;
      for (int x = 0; x < 2; x++) {
        for (size_t i = 0; i < vertexs[x].size(); i++) {
          vba.Position<V3f>(pos_start) = vertexs[x][i];
          V3f vertex = vertexs[x][i];

          if (VertexInJobArea(jobArea, vertex, vCnt)) {
            vba.Color<V4f>(0, pos_start) = vColors[x][i];
          } else {
            V4f vCorlor;
            vCorlor[0] = 0.2;
            vCorlor[1] = 0.2;
            vCorlor[2] = 0.2;
            vCorlor[3] = 0.5;
            vba.Color<V4f>(0, pos_start) = vCorlor;
          }

          pos_start++;
        }

        for (size_t i = 0; i < indices[x].size(); i++) {
          indices_array[idx_start] = indices[x][i];
          idx_start++;
        }
      }

      pLine = new AreaDrawable();

      pLine->m_pVertexBuffer = vertexBuffer;
      pLine->m_pVertexformat = vertexformat;
      pLine->m_pIndexBuffer = indexBuffer;
    }
    return pLine;
  }

  return NULL;
}

}  // namespace geditor
