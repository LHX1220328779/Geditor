
#include "core/job_layer.h"

#include "core/boundary_layer.h"
#include "core/factory_drawable.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_polygon_hole.h"
#include "core/geo_rectangle.h"
#include "core/job_area.h"
#include "coverage/clipper/clipper.hpp"
#include "map/vdb_manage.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_technique_manager.h"

// #include "coverage\clipper\clipper.hpp"

#include <assert.h>
#include <time.h>

#include "platform/system.h"

namespace geditor {
JobLayer::JobLayer() : Layer(LT_FUNAREA), m_dPointSize(0.1) {}

JobLayer::~JobLayer() {
  for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    delete (*it);
  }
}

MapFeature *JobLayer::GetFeature(int idx) {
  return GetFeatureVec(idx, m_LaneSegment);
}
//
void JobLayer::AddMapObject(std::vector<JobArea *> segmentArray) {
  for (int i = 0; i < segmentArray.size(); i++) {
    AddMapFeature(segmentArray[i]);
  }
}

void JobLayer::SetPointSize(double x) {
  m_dPointSize = x;

  for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    (*it)->SetChanged(true);
  }
}

//添加地图要素
void JobLayer::AddMapFeature(MapFeature *feature) {
  if (feature != NULL) {
    JobArea *jobArea = (JobArea *)feature;

    jobArea->SetMapLayer(this);
    if (jobArea->GetUniqueID() == 0) {
      int uniqueId = GenerateFeatureID();

      JobProperty *pProperty = jobArea->GetProperty();
      pProperty->jobId = uniqueId;

      jobArea->SetUniqueID(uniqueId);
    }

    m_LaneSegment.push_back(jobArea);
  }
}

void JobLayer::GetAllMapFeature(std::vector<MapFeature *> &objects) {
  for (int i = 0; i < m_LaneSegment.size(); i++) {
    MapFeature *pMapFeature = m_LaneSegment[i];

    objects.push_back(pMapFeature);
  }
}

bool JobLayer::DeleteMapFeature(MapFeature *feature) {
  for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    if (feature == (*it)) {
      m_LaneSegment.erase(it);
      return true;
    }
  }
  return false;
}

bool JobLayer::GetBoundary(V3d &vMin, V3d &vMax) {
  if (m_LaneSegment.size() > 0) {
    for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
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

void JobLayer::MergeObject(const std::vector<JobArea *> &jobArea) {
  int iCount = jobArea.size();
  if (iCount > 0) {
    int nUniqueID = jobArea[0]->GetUniqueID();

    for (int i = 0; i < iCount; i++) {
      JobProperty *pProperty = jobArea[i]->GetProperty();
      pProperty->jobId = nUniqueID;
    }
  }
  /*
  int  iCount = jobArea.size();
  if (iCount > 1)
  {
  GeoPolygonHole* polygon = new GeoPolygonHole;


  Geometry* outerPolygon = jobArea[0]->GetGeometry();

  int iSize = outerPolygon->GetVertexCount();
  for (int i = 0; i < iSize; i++)
  {
  Point3d pt = outerPolygon->GetVertex(i);
  polygon->AppendVertex(pt);
  }

  for (int i = 1; i < iCount; i++)
  {
  Geometry* interPolygon = jobArea[i]->GetGeometry();
  polygon->AppendHole((GeoPolygon*)interPolygon);

  this->DeleteMapFeature(jobArea[i]);
  }

  jobArea[0]->SetGeometry(polygon);
  }
  */
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

static bool Orientation(GeoPolygon *pPolygon) {
  int size = (int)pPolygon->GetVertexCount();
  if (size < 3) {
    return 0;
  }

  double a = 0;
  for (int i = 0, j = size - 1; i < size; ++i) {
    Point3d const &U0 = pPolygon->GetVertex(i);
    Point3d const &U1 = pPolygon->GetVertex(j);

    a += ((double)U1.x + U0.x) * ((double)U1.y - U0.y);
    j = i;
  }
  return (-a * 0.5) >= 0;
}

static bool CliperSinglePolygon(GeoPolyline *pClipLine, GeoPolygon *pPolygon,
                                GeoPolygon **polygon) {
  //交叉点数量
  int counter = 0;

  //交点线的索引
  int idx_line[2];

  //交点polygon的索引
  int idx_poly[2];

  //交点
  Point3d crossPnt[2];

  //多边形按顺时针排序
  if (!Orientation(pPolygon)) {
    pPolygon->ReverseVertex();
  }

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
              polygon[x] = new GeoPolygon();
              polygon[x]->AppendVertex(crossPnt[0]);
              for (int j = idx_line[0] + 1; j <= idx_line[1]; j++) {
                Point3d pt = pClipLine->GetVertex(j);
                polygon[x]->AppendVertex(pt);
              }
              polygon[x]->AppendVertex(crossPnt[1]);
            }

            //--------------------------------------------------------
            polygon[1]->ReverseVertex();
            for (int x = 0; x < 2; x++) {
              int start_index, end_index;
              if (x == 0) {
                start_index = (idx_poly[1] + 1) % iVertexCount;
                end_index = (idx_poly[0] + 1) % iVertexCount;
              } else {
                start_index = (idx_poly[0] + 1) % iVertexCount;
                end_index = (idx_poly[1] + 1) % iVertexCount;
              }

              if (start_index != end_index) {
                for (int j = start_index; j % iVertexCount != end_index; j++) {
                  Point3d pt = pPolygon->GetVertex(j % iVertexCount);
                  polygon[x]->AppendVertex(pt);
                }
              } else {
                if (Orientation(polygon[x])) {
                  polygon[x]->ReverseVertex();
                }

                int j = start_index;
                do {
                  Point3d pt = pPolygon->GetVertex(j % iVertexCount);
                  polygon[x]->AppendVertex(pt);
                  j++;
                } while (j % iVertexCount != end_index);

                break;
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

static int findEdge(GeoPolygon *pPolygon, const Point3d &p) {
  int size = (int)pPolygon->GetVertexCount();
  for (int i = 0, j = size - 1; i < size; j = i++) {
    Point3d const &U0 = pPolygon->GetVertex(i);
    Point3d const &U1 = pPolygon->GetVertex(j);

    double rhs, lhs;
    if (p.y < U1.y) {
      if (U0.y <= p.y) {
        lhs = (p.y - U0.y) * (U1.x - U0.x);
        rhs = (p.x - U0.x) * (U1.y - U0.y);

        if (abs(lhs - rhs) < 0.000001) {
          return j;
        }
      }
    } else if (p.y < U0.y)  // U1 on or below ray, U0 above ray
    {
      lhs = (p.y - U0.y) * (U1.x - U0.x);
      rhs = (p.x - U0.x) * (U1.y - U0.y);

      if (abs(rhs - lhs) < 0.000001) {
        return j;
      }
    }
  }

  return -1;
}

void writeHoleLog(const char *filename, GeoPolygonHole *pPolygon) {
  FILE *pfile = fopen(filename, "wb");
  if (pfile) {
    fprintf(pfile, "polygon((\r\n");
    int vCnt = pPolygon->GetVertexCount();
    for (int i = 0; i < vCnt; i++) {
      Point3d pt = pPolygon->GetVertex(i);
      if (i == vCnt - 1) {
        fprintf(pfile, "%.4f  %0.4f\r\n", pt.x, pt.y);
      } else {
        fprintf(pfile, "%.4f  %0.4f,\r\n", pt.x, pt.y);
      }
    }
    fprintf(pfile, "))\r\n");

    int iCount = pPolygon->GetHoleCount();
    for (int i = 0; i < iCount; i++) {
      GeoPolygon *pHole = pPolygon->GetHole(i);

      fprintf(pfile, "polygon((\r\n");
      int vCnt = pHole->GetVertexCount();
      for (int i = 0; i < vCnt; i++) {
        Point3d pt = pHole->GetVertex(i);
        if (i == vCnt - 1) {
          fprintf(pfile, "%.4f  %0.4f\r\n", pt.x, pt.y);
        } else {
          fprintf(pfile, "%.4f  %0.4f,\r\n", pt.x, pt.y);
        }
      }
      fprintf(pfile, "))\r\n");
    }

    fclose(pfile);
  }
}

void writeLog(const char *filename, Geometry *pPolygon) {
  FILE *pfile = fopen(filename, "wb");
  if (pfile) {
    fprintf(pfile, "polygon((\r\n");
    int vCnt = pPolygon->GetVertexCount();
    for (int i = 0; i < vCnt; i++) {
      Point3d pt = pPolygon->GetVertex(i);
      if (i == vCnt - 1) {
        fprintf(pfile, "%.4f  %0.4f\r\n", pt.x, pt.y);
      } else {
        fprintf(pfile, "%.4f  %0.4f,\r\n", pt.x, pt.y);
      }
    }
    fprintf(pfile, "))\r\n");

    fclose(pfile);
  }
}

static void DeleteEdge(GeoPolygon *pPolygon, GeoPolygon *pHole, int edge,
                       int idxPt) {
  writeLog(".\\01_polygon.txt", pPolygon);
  writeLog(".\\01_hole.txt", pHole);

  GeoPolygon newPolygon;

  for (int i = 0; i <= edge; i++) {
    Point3d pt = pPolygon->GetVertex(i);
    newPolygon.AppendVertex(pt);
  }

  int nNum = pHole->GetVertexCount();
  int insertPos = edge + 1;

  Point3d pt = pHole->GetVertex(idxPt--);
  newPolygon.InsertVertex(insertPos++, pt);

  do {
    //空洞的下一个点
    idxPt = (idxPt + nNum) % nNum;
    Point3d pt = pHole->GetVertex(idxPt--);

    int edgeIndex = findEdge(pPolygon, pt);
    if (edgeIndex >= 0) {
      newPolygon.InsertVertex(insertPos++, pt);

      int vCnt = pPolygon->GetVertexCount();
      for (int i = edgeIndex + 1; i < vCnt; i++) {
        Point3d pt = pPolygon->GetVertex(i);
        newPolygon.AppendVertex(pt);
      }
      break;
    } else {
      newPolygon.InsertVertex(insertPos++, pt);
    }

  } while (true);

  int iCnt = newPolygon.GetVertexCount();

  pPolygon->Clear();
  for (int i = 0; i < iCnt; i++) {
    Point3d pt = newPolygon.GetVertex(i);
    pPolygon->AppendVertex(pt);
  }
}

GeoPolygon *SimplePolygon(GeoPolygon *pPolygon,
                          std::vector<GeoPolygon *> arrHole) {
  if (!Orientation(pPolygon)) {
    pPolygon->ReverseVertex();
  }
  for (int x = 0; x < arrHole.size(); x++) {
    if (!Orientation(arrHole[x])) {
      arrHole[x]->ReverseVertex();
    }
  }

  //尝试移除孔洞
  std::vector<GeoPolygon *> leftHole;
  for (int x = 0; x < arrHole.size(); x++) {
    writeLog(".\\02_polygon.txt", pPolygon);
    writeLog(".\\02_hole.txt", arrHole[x]);

    /*
    pPolygon->Clear();
    pPolygon->AppendVertex(Point3d(443890.0785, 4436679.7641, 0.0));
    pPolygon->AppendVertex(Point3d(443880.4969, 4436683.9144, 0.0));
    pPolygon->AppendVertex(Point3d(443876.7327, 4436685.6378, 0.0));
    pPolygon->AppendVertex(Point3d(443880.0746, 4436687.5703, 0.0));
    pPolygon->AppendVertex(Point3d(443886.2811, 4436686.7109, 0.0));
    pPolygon->AppendVertex(Point3d(443887.1405, 4436682.9870, 0.0));
    pPolygon->AppendVertex(Point3d(443886.3390, 4436681.3839, 0.0));
    pPolygon->AppendVertex(Point3d(443880.4969, 4436683.9144, 0.0));
    pPolygon->AppendVertex(Point3d(443870.7093, 4436688.3955, 0.0));
    pPolygon->AppendVertex(Point3d(443867.8611, 4436685.0823, 0.0));
    pPolygon->AppendVertex(Point3d(443870.0092, 4436682.9342, 0.0));
    pPolygon->AppendVertex(Point3d(443864.5442, 4436677.3422, 0.0));
    pPolygon->AppendVertex(Point3d(443859.7875, 4436676.3533, 0.0));
    pPolygon->AppendVertex(Point3d(443858.9270, 4436682.7380, 0.0));
    pPolygon->AppendVertex(Point3d(443859.1146, 4436683.5413, 0.0));
    pPolygon->AppendVertex(Point3d(443850.3818, 4436686.3450, 0.0));
    pPolygon->AppendVertex(Point3d(443850.4329, 4436684.6080, 0.0));
    pPolygon->AppendVertex(Point3d(443850.4329, 4436684.6080, 0.0));
    pPolygon->AppendVertex(Point3d(443850.2153, 4436672.3162, 0.0));
    pPolygon->AppendVertex(Point3d(443871.5357, 4436663.5596, 0.0));
    pPolygon->AppendVertex(Point3d(443887.3628, 4436664.7561, 0.0));
    pPolygon->AppendVertex(Point3d(443891.0069, 4436672.0442, 0.0));

    arrHole[x]->Clear();
    arrHole[x]->AppendVertex(Point3d(443886.3390, 4436681.3839, 0.0));
    arrHole[x]->AppendVertex(Point3d(443880.4969, 4436683.9144, 0.0));
    arrHole[x]->AppendVertex(Point3d(443876.7327, 4436685.6378, 0.0));
    arrHole[x]->AppendVertex(Point3d(443875.8031, 4436685.1003, 0.0));
    arrHole[x]->AppendVertex(Point3d(443875.7777, 4436685.0877, 0.0));
    arrHole[x]->AppendVertex(Point3d(443875.3958, 4436675.1571, 0.0));
    arrHole[x]->AppendVertex(Point3d(443882.5572, 4436673.8203, 0.0));
    */

    bool bDelFlag = true;

    GeoPolygon *pHole = arrHole[x];
    int iVxCnt = pHole->GetVertexCount();
    for (int t = 0; t < iVxCnt; t++) {
      Point3d pt = pHole->GetVertex(t);

      int edgeIndex = findEdge(pPolygon, pt);
      if (edgeIndex >= 0) {
        DeleteEdge(pPolygon, pHole, edgeIndex, t);

        bDelFlag = false;
        break;
      }
    }

    if (bDelFlag) {
      leftHole.push_back(pHole);
    } else {
      bool b = true;
    }
  }

  //组合输出
  if (leftHole.size() > 0) {
    GeoPolygonHole *pSplitPolygon = new GeoPolygonHole();

    //生成外轮廓
    int iCount = pPolygon->GetVertexCount();
    for (int m = 0; m < iCount; m++) {
      Point3d pt = pPolygon->GetVertex(m);
      pSplitPolygon->AppendVertex(pt);
    }

    //添加孔洞
    for (int m = 0; m < leftHole.size(); m++) {
      pSplitPolygon->AppendHole(leftHole[m]);
    }

    return pSplitPolygon;
  } else {
    return pPolygon;
  }
}

bool isPolygonInPolygon(GeoPolygon *host, GeoPolygon *subject) {
  ClipperLib::Path host_path;

  int iVxCnt = host->GetVertexCount();
  for (int x = 0; x < iVxCnt; x++) {
    Point3d pt = host->GetVertex(x);
    host_path.push_back(ClipperLib::IntPoint(pt.x * 1000, pt.y * 1000));
  }

  ClipperLib::Path subject_path;

  int iCnt = subject->GetVertexCount();
  for (int x = 0; x < iCnt; x++) {
    Point3d pt = subject->GetVertex(x);
    subject_path.push_back(ClipperLib::IntPoint(pt.x * 1000, pt.y * 1000));
  }

  ClipperLib::Paths out_polys;

  ClipperLib::Clipper c;
  c.AddPath(host_path, ClipperLib::ptSubject, true);
  c.AddPath(subject_path, ClipperLib::ptClip, true);
  c.StrictlySimple(true);
  c.Execute(ClipperLib::ctIntersection, out_polys, ClipperLib::pftEvenOdd);

  for (int i = 0; i < out_polys.size(); i++) {
    double dArea = ClipperLib::Area(out_polys[i]);
    if (abs(dArea) > 10000) {
      return true;
    }
  }

  return false;
}

static bool CliperPolygonHole(GeoPolyline *pClipLine, GeoPolygonHole *pPolygon,
                              GeoPolygon **arrPolygon) {
  writeLog(".\\03_ClipLine.txt", pClipLine);
  writeHoleLog(".\\03_holePolygon.txt", pPolygon);

  //切分外扩轮
  GeoPolygon geoPolygon;

  int vnt = pPolygon->GetVertexCount();
  for (int x = 0; x < vnt; x++) {
    Point3d pt = pPolygon->GetVertex(x);
    geoPolygon.AppendVertex(pt);
  }

  GeoPolygon *outPolygon[2];
  if (!CliperSinglePolygon(pClipLine, &geoPolygon, outPolygon)) {
    return false;
  }

  writeLog(".\\04_outPolygon0.txt", outPolygon[0]);
  writeLog(".\\04_outPolygon1.txt", outPolygon[1]);

  //切分空洞
  std::vector<GeoPolygon *> holeArr;

  int iCount = pPolygon->GetHoleCount();
  for (int i = 0; i < iCount; i++) {
    GeoPolygon *pHole = pPolygon->GetHole(i);

    GeoPolygon *holeSplit[2];
    if (CliperSinglePolygon(pClipLine, pHole, holeSplit)) {
      holeArr.push_back(holeSplit[0]);
      holeArr.push_back(holeSplit[1]);
    } else {
      holeArr.push_back(pHole);
    }
  }

  for (int i = 0; i < holeArr.size(); i++) {
    char tem[128];
    sprintf(tem, ".\\04_holeArr%d.txt", i);

    writeLog(tem, holeArr[i]);
  }

  //拼裝組合
  for (int i = 0; i < 2; i++) {
    std::vector<GeoPolygon *> combine;

    for (int x = 0; x < holeArr.size(); x++) {
      if (isPolygonInPolygon(outPolygon[i], holeArr[x])) {
        combine.push_back(holeArr[x]);
      }
    }

    writeLog(".\\05_polygon.txt", outPolygon[i]);

    for (int t = 0; t < combine.size(); t++) {
      char tem[128];
      sprintf(tem, ".\\05_combine%d.txt", t);

      writeLog(tem, combine[t]);
    }

    if (combine.size() > 0) {
      arrPolygon[i] = SimplePolygon(outPolygon[i], combine);
    } else {
      arrPolygon[i] = outPolygon[i];
    }
    writeLog(".\\06_polygon.txt", arrPolygon[i]);
  }

  return true;
}

bool JobLayer::CliperLayer(GeoPolyline *pClipLine) {
  // GeoPolygon* pPolygon1 = new GeoPolygon();
  // pPolygon1->AppendVertex(Point3d(443844.5330, 4436658.4432, 0.0));
  // pPolygon1->AppendVertex(Point3d(443860.9876, 4436672.5867, 0.0));
  // pPolygon1->AppendVertex(Point3d(443892.7141, 4436678.5426, 0.0));
  // pPolygon1->AppendVertex(Point3d(443894.2036, 4436686.3963, 0.0));
  // pPolygon1->AppendVertex(Point3d(443859.6440, 4436693.3295, 0.0));
  // pPolygon1->AppendVertex(Point3d(443859.5950, 4436693.3632, 0.0));
  // pPolygon1->AppendVertex(Point3d(443829.9058, 4436660.2884, 0.0));
  //
  // GeoPolygon* pPolygon2 = new GeoPolygon();
  // pPolygon2->AppendVertex(Point3d(443893.2339, 4436681.2829, 0.0));
  // pPolygon2->AppendVertex(Point3d(443892.7141, 4436678.5426, 0.0));
  // pPolygon2->AppendVertex(Point3d(443877.6944, 4436675.7230, 0.0));
  // pPolygon2->AppendVertex(Point3d(443879.0709, 4436666.0462, 0.0));
  // pPolygon2->AppendVertex(Point3d(443890.4683, 4436668.9631, 0.0));
  // pPolygon2->AppendVertex(Point3d(443894.8525, 4436675.4106, 0.0));
  // pPolygon2->AppendVertex(Point3d(443894.4227, 4436680.2247, 0.0));

  // isPolygonInPolygon(pPolygon1, pPolygon2);

  // GeoPolygon* pPolygon = new GeoPolygon();
  //
  // std::vector<GeoPolygon*> arrHole;
  // arrHole.push_back(new GeoPolygon);
  //
  // SimplePolygon(pPolygon, arrHole);
  //
  // JobArea* pAreaRoad = new JobArea();
  // pAreaRoad->SetGeometry(pPolygon);
  //
  // m_LaneSegment.clear();
  // AddMapFeature(pAreaRoad);
  //
  //
  // return false;

  for (int i = 0; i < m_LaneSegment.size(); i++) {
    JobArea *feature = m_LaneSegment[i];

    Geometry *pGeometry = (GeoPolygon *)feature->GetGeometry();
    if (pGeometry->GetGeometryType() == Geometry::GT_POLYGON) {
      GeoPolygon *pPolygon = (GeoPolygon *)pGeometry;

      GeoPolygon *polygon[2];
      if (CliperSinglePolygon(pClipLine, pPolygon, polygon)) {
        JobProperty *pProperty = feature->GetProperty();
        feature->SetGeometry(polygon[0]);

        //------------

        JobArea *pAreaRoad = new JobArea();

        pAreaRoad->SetProperty(pProperty);
        pAreaRoad->SetGeometry(polygon[1]);

        AddMapFeature(pAreaRoad);

        return true;
      }
    } else if (pGeometry->GetGeometryType() == Geometry::GT_POLYGON_HOLE) {
      GeoPolygonHole *pPolygon = (GeoPolygonHole *)pGeometry;

      GeoPolygon *polygon[2];
      if (CliperPolygonHole(pClipLine, pPolygon, polygon)) {
        JobProperty *pProperty = feature->GetProperty();
        feature->SetGeometry(polygon[0]);

        //------------

        JobArea *pAreaRoad = new JobArea();

        pAreaRoad->SetProperty(pProperty);
        pAreaRoad->SetGeometry(polygon[1]);

        AddMapFeature(pAreaRoad);

        return true;
      }
    }
  }
  return false;
}

int JobLayer::GetAllJobArea(std::vector<JobArea *> &jobArea) {
  int iCount = m_LaneSegment.size();

  for (int i = 0; i < iCount; i++) {
    jobArea.push_back(m_LaneSegment[i]);
  }

  return iCount;
}

bool JobLayer::PickupObject(const Point3d &mousePoint, double tolerance,
                            PickupResult &reslut) {
  bool bFind = false;

  for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
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
    } else if (pGeomLine->GetGeometryType() == Geometry::GT_POLYGON_HOLE) {
      GeoPolygonHole *polyline = dynamic_cast<GeoPolygonHole *>(pGeomLine);
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
    } else if (pGeomLine->GetGeometryType() == Geometry::GT_RECTANGLE) {
      GeoRectangle *polyline = dynamic_cast<GeoRectangle *>(pGeomLine);
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
    } else if (pGeomLine->GetGeometryType() == Geometry::GT_POLYLINE) {
      GeoPolyline *polyline = dynamic_cast<GeoPolyline *>(pGeomLine);
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
void JobLayer::Cull(double minX, double minY, double maxX, double maxY) {
  for (std::map<Geometry *, PositionTransformNode *>::iterator iter =
           m_RenderLeaf.begin();
       iter != m_RenderLeaf.end();) {
    bool bFind = false;

    for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
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

  for (std::vector<JobArea *>::iterator it = m_LaneSegment.begin();
       it != m_LaneSegment.end(); it++) {
    JobArea *pLaneSegment = (*it);

    JobProperty *pProperty = (*it)->GetProperty();
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

      Color clr = m_colorAreaMap.GetDisplayColor(pProperty->areaType);
      clr.a =
          pLaneSegment->selected() || pLaneSegment->highlighted() ? 175 : 100;
      Drawable *pArrow = FactoryDrawable::CreateAreaDrawable(
          pPolyline, vCnt,
          V4f(clr.r / 255.0f, clr.g / 255.0f, clr.b / 255.0f, clr.a / 255.0f));
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

      if (pPolyline->GetGeometryType() == Geometry::GT_POLYLINE) {
        V3f clr = pStyle->GetArrowColor();
        Drawable *pArrow = FactoryDrawable::CreateArrowDrawable(
            pPolyline, vCnt, pStyle->GetLineColor(), 0.3, 1);
        if (pArrow != NULL) {
          RenderTechnique *pTechnique =
              TechniqueManager::GetInstance()->GetTechnique(1);

          RenderLeaf *pRenderLeaf = new RenderLeaf();
          pRenderLeaf->SetDrawable(pArrow);
          pRenderLeaf->SetRenderTechnique(pTechnique);

          pTransformNode->AddChild(pRenderLeaf);
        }
      }

      V3f vclr = pStyle->GetKeyVertexColor();
      Drawable *pNode = FactoryDrawable::CreateNodeDrawable(
          pPolyline, vCnt, m_dPointSize, V4f(vclr[0], vclr[1], vclr[2], 1.0),
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

void JobLayer::Update(const Matrix4x4f &svMatrix, Camera *pCamera) {
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

        //更新变换矩阵
        pRenderLeaf->SetViewport(viewport);
        pRenderLeaf->SetModelViewMatrix(&mvMatrix);
        pRenderLeaf->SetProjectionMatrix(&prjMatrix);
      }
    }
  }
}

void JobLayer::Draw(RenderInfo &rendinfo) {
  if (!m_bVisible) return;
  for (std::map<Geometry *, PositionTransformNode *>::iterator it =
           m_RenderLeaf.begin();
       it != m_RenderLeaf.end(); it++) {
    if (it->first->GetGeometryType() == Geometry::GT_POLYGON) {
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
  for (std::map<Geometry *, PositionTransformNode *>::iterator it =
           m_RenderLeaf.begin();
       it != m_RenderLeaf.end(); it++) {
    if (it->first->GetGeometryType() != Geometry::GT_POLYGON) {
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
}

int JobLayer::OnPoint(Geometry *geometry, const Point3d &Q, double tolerance) {
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

  if (geometry->GetGeometryType() == Geometry::GT_POLYGON_HOLE) {
    int start = nSize;

    GeoPolygonHole *pPolygonHole = (GeoPolygonHole *)geometry;
    int nHoleCount = pPolygonHole->GetHoleCount();

    for (int x = 0; x < nHoleCount; x++) {
      GeoPolygon *pPolygon = pPolygonHole->GetHole(x);

      int nSize = pPolygon->GetVertexCount();
      for (int i = 0; i < nSize; i++) {
        Point3d point = pPolygon->GetVertex(i);

        double maxVal =
            Mathd::Max(Mathd::Abs(point.x - Q.x), Mathd::Abs(point.y - Q.y));
        if (maxVal < minFlag) {
          minFlag = maxVal;
          index = start + i;
        }
      }

      start += nSize;
    }
  }

  if (minFlag < tolerance) {
    return index;
  } else {
    return -1;
  }
}

void JobLayer::Save(VDBManage *vdb) {
  vdb->ClearJobProperty();

  for (std::vector<JobArea *>::iterator iter = m_LaneSegment.begin();
       iter != m_LaneSegment.end(); iter++) {
    JobArea *pLaneSegment = (*iter);

    char *pMem = NULL;
    int length = PackGeometry(pLaneSegment->GetGeometry(), pMem);

    vdb->SaveJobProperty(pLaneSegment->GetUniqueID(),
                         pLaneSegment->GetProperty(), pMem, length);

    //----------------------------

    std::vector<MapFeature *> attachOjbects;
    pLaneSegment->GetAttachObject(attachOjbects);

    vdb->SaveJobAttachObject(pLaneSegment->GetUniqueID(), attachOjbects);
  }
}

void JobLayer::ClearLayer() {
  for (std::vector<JobArea *>::iterator iter = m_LaneSegment.begin();
       iter != m_LaneSegment.end(); iter++) {
    delete (*iter);
  }
  m_LaneSegment.clear();
}

void JobLayer::Read(VDBManage *vdb, BoundaryLayer *boundary) {
  ClearLayer();

  std::vector<Job *> areaArray;
  vdb->ReadJobProperty(areaArray);
  for (int i = 0; i < areaArray.size(); i++) {
    JobArea *pLaneSegment = new JobArea();

    Geometry *pPolyline = areaArray[i]->polyline;

    pLaneSegment->SetGeometry(areaArray[i]->polyline);
    pLaneSegment->SetProperty(&areaArray[i]->pProperty);
    pLaneSegment->SetUniqueID(areaArray[i]->uniqueId);

    std::vector<int> attachObjects;
    vdb->ReadJobAttachObject(areaArray[i]->uniqueId, attachObjects);
    for (int x = 0; x < attachObjects.size(); x++) {
      auto *pBound = boundary->GetFeature(attachObjects[i]);
      if (pBound) {
        pLaneSegment->AddAttachObject(pBound);
      }
    }

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

int JobLayer::PackGeometry(Geometry *pPolyline, char *&pMem) {
  //---------------------------------------------
  GEO_HDR hdr;

  hdr.magic = 0x00306567;
  hdr.version = 1;
  hdr.reserve = 0;
  hdr.crc32 = 0;

  //------------------------------------

  std::vector<Point3d> sample_items;
  std::vector<Point3d> control_items;

  std::vector<std::pair<int, int>> vxPair;

  Geometry::GeometryType type = pPolyline->GetGeometryType();
  if (type == Geometry::GT_BEZIER_CURVE) {
    GeoBezierCurve3 *pCurve = (GeoBezierCurve3 *)pPolyline;
    pCurve->Hermite(sample_items);

    for (int i = 0; i < pCurve->GetVertexCount(); i++) {
      Point3d pt = pCurve->GetVertex(i);
      control_items.push_back(pt);
    }
  } else if (type == Geometry::GT_BSPLINE_CURVE) {
    GeoBSplineCurve3 *pCurve = (GeoBSplineCurve3 *)pPolyline;
    pCurve->Hermite(sample_items);

    for (int i = 0; i < pCurve->GetVertexCount(); i++) {
      Point3d pt = pCurve->GetVertex(i);
      control_items.push_back(pt);
    }
  } else if (type == Geometry::GT_CIRCULAR_ARC) {
    GeoCircularArc *pCurve = (GeoCircularArc *)pPolyline;
    pCurve->Hermite(sample_items);

    for (int i = 0; i < pCurve->GetVertexCount(); i++) {
      Point3d pt = pCurve->GetVertex(i);
      control_items.push_back(pt);
    }
  } else if (type == Geometry::GT_HERMITE_CURVE) {
    LOG(ERROR) << "Hermite curve not supported yet";
    // GeoHermiteCurve3 *pCurve = (GeoHermiteCurve3 *) pPolyline;
    // pCurve->Hermite(sample_items);

    // for (int i = 0; i < pCurve->GetVertexCount(); i++) {
    //     Point3d pt = pCurve->GetVertex(i);
    //     control_items.push_back(pt);
    // }
  } else if (type == Geometry::GT_RECTANGLE) {
    GeoRectangle *pCurve = (GeoRectangle *)pPolyline;
    pCurve->Hermite(sample_items);

    for (int i = 0; i < pCurve->GetVertexCount(); i++) {
      Point3d pt = pCurve->GetVertex(i);
      control_items.push_back(pt);
    }
  } else if (type == Geometry::GT_POLYLINE || type == Geometry::GT_POLYGON) {
    for (int i = 0; i < pPolyline->GetVertexCount(); i++) {
      Point3d pt = pPolyline->GetVertex(i);
      sample_items.push_back(pt);
    }

    control_items = sample_items;
  } else if (type == Geometry::GT_POLYGON_HOLE) {
    GeoPolygonHole *pPolygonHole = (GeoPolygonHole *)pPolyline;
    int startPosition = 0;

    //外轮廓
    for (int i = 0; i < pPolygonHole->GetVertexCount(); i++) {
      Point3d pt = pPolygonHole->GetVertex(i);
      sample_items.push_back(pt);
    }
    vxPair.push_back(std::pair<int, int>(startPosition, sample_items.size()));
    startPosition = sample_items.size();

    //内轮廓
    int iHoleCount = pPolygonHole->GetHoleCount();
    for (int x = 0; x < iHoleCount; x++) {
      GeoPolygon *pPolygon = pPolygonHole->GetHole(x);
      for (int i = 0; i < pPolygon->GetVertexCount(); i++) {
        Point3d pt = pPolygon->GetVertex(i);
        sample_items.push_back(pt);
      }

      vxPair.push_back(std::pair<int, int>(startPosition, sample_items.size()));
      startPosition = sample_items.size();
    }

    control_items = sample_items;
  }

  //--------------------------------------------------
  ProjectionUTM projectionUTM;

  std::vector<GPSPoint> pointlist;

  int nCount = sample_items.size();
  pointlist.resize(nCount);
  for (int i = 0; i < nCount; i++) {
    int nzone = ProjectionUTM::zone;
    projectionUTM.CartesianToLatLon(sample_items[i].x, sample_items[i].y, nzone,
                                    false, pointlist[i].latlon);
    pointlist[i].altitude = sample_items[i].z;
    // LOG(INFO) << i << " " << sample_items[i].z;
  }
  //----------------------------------------------------------

  std::vector<GPSPoint> rawPoints;

  int rawCount = control_items.size();
  rawPoints.resize(rawCount);
  for (int i = 0; i < rawCount; i++) {
    Point3d pt = control_items[i];

    int nzone = ProjectionUTM::zone;
    projectionUTM.CartesianToLatLon(pt.x, pt.y, nzone, false,
                                    rawPoints[i].latlon);
    rawPoints[i].altitude = pt.z;
    rawPoints[i].id = pt.GetId();
  }
  //临时这样写
  hdr.reserve = rawCount;
  //----------------------------------

  GEO_INFO info;

  info.geotype = type;
  info.count = nCount;
  info.minLat = 0;
  info.maxLat = 0;
  info.maxLon = 0;
  info.minLon = 0;
  info.minAlt = 0;
  info.maxAlt = 0;

  //----------------------------------
  int pointlen = sizeof(GPSPoint) * nCount;
  int rawlen = sizeof(GPSPoint) * rawCount;
  int pairlen = sizeof(std::pair<int, int>) * vxPair.size();

  int length = sizeof(GEO_HDR) + sizeof(GEO_INFO) + pointlen + rawlen;
  length += pairlen;

  char *pMemery = new char[length];
  int nWriteByte = 0;

  int icousn = sizeof(std::pair<int, int>);

  memcpy(pMemery + nWriteByte, &hdr, sizeof(GEO_HDR));
  nWriteByte += sizeof(GEO_HDR);

  memcpy(pMemery + nWriteByte, &info, sizeof(GEO_INFO));
  nWriteByte += sizeof(GEO_INFO);

  if (pointlen > 0) {
    memcpy(pMemery + nWriteByte, &pointlist[0], pointlen);
    nWriteByte += pointlen;
  }

  if (rawlen > 0) {
    memcpy(pMemery + nWriteByte, &rawPoints[0], rawlen);
    nWriteByte += rawlen;
  }

  if (pairlen > 0) {
    memcpy(pMemery + nWriteByte, &vxPair[0], pairlen);
    nWriteByte += pairlen;
  }

  pMem = pMemery;

  return nWriteByte;
}
}  // namespace geditor
