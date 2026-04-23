
#include "core/geo_bspline_curve3.h"

#include <algorithm>

#include "algorithm/mc_math.h"
#include "core/bline.h"
#include "core/compute_curve.h"

namespace geditor {

GeoBSplineCurve3::GeoBSplineCurve3()
    : Geometry(GT_BSPLINE_CURVE), m_bNeedSample(false) {}

GeoBSplineCurve3::~GeoBSplineCurve3() {}

void GeoBSplineCurve3::OnChange() { m_bNeedSample = true; }

void GeoBSplineCurve3::ControlPoint(const std::vector<Point3d> &mousePoint,
                                    std::vector<Point3d> &controlPoint) {
  size_t nCtrlPtCount = mousePoint.size();  // ʵ�ʿ��Ƶ���
  if (nCtrlPtCount < 2) {
    return;
  }

  int nPt = nCtrlPtCount + 3;  // ���ۿ��Ƶ���
  int maxPtCount = nCtrlPtCount + 5;

  double q0x = mousePoint[0].x;
  double q0y = mousePoint[0].y;
  double qnx = mousePoint.back().x;
  double qny = mousePoint.back().y;

  //-----------------------------------------

  double *sishu = new double[maxPtCount * maxPtCount];
  memset(sishu, 0, maxPtCount * maxPtCount * sizeof(double));

  double *tempRow = new double[maxPtCount];

  //	��ʼ�������
  sishu[0 * maxPtCount + 0] = -0.5;
  sishu[0 * maxPtCount + 1] = 0.0;
  sishu[0 * maxPtCount + 2] = 3.0 / 2;
  sishu[1 * maxPtCount + 0] = 0.0;
  sishu[1 * maxPtCount + 1] = 4.0 / 6;
  sishu[1 * maxPtCount + 2] = 4.0 / 6;

  for (int i = 2; i <= nPt - 1 - 2; i++) {
    sishu[i * maxPtCount + (i - 1)] = 1.0 / 6;
    sishu[i * maxPtCount + (i)] = 4.0 / 6;
    sishu[i * maxPtCount + (i + 1)] = 1.0 / 6;
  }

  sishu[(nPt - 2) * maxPtCount + (nPt - 2 - 1)] = 2.0 / 9;
  sishu[(nPt - 2) * maxPtCount + (nPt - 2)] = 4.0 / 6;
  sishu[(nPt - 2) * maxPtCount + (nPt - 2 + 1)] = 0;
  sishu[(nPt - 1) * maxPtCount + (nPt - 2 - 1)] = -0.5;
  sishu[(nPt - 1) * maxPtCount + (nPt - 2)] = 0.0;
  sishu[(nPt - 1) * maxPtCount + (nPt - 2 + 1)] = 3.0 / 2;

  //-------------------------------------------

  // ��ʼ���ұߵľ���
  sishu[0 * maxPtCount + (nPt)] = q0x;
  sishu[0 * maxPtCount + (nPt + 1)] = q0y;
  sishu[1 * maxPtCount + (nPt)] = q0x + 1.0 / 3 * q0x;
  sishu[1 * maxPtCount + (nPt + 1)] = q0y + 1.0 / 3 * q0y;

  for (int i = 2; i <= nPt - 1 - 2; i++) {
    sishu[i * maxPtCount + (nPt)] = mousePoint[i - 2].x;
    sishu[i * maxPtCount + (nPt + 1)] = mousePoint[i - 2].y;
  }

  sishu[(nPt - 2) * maxPtCount + (nPt)] = qnx - 1.0 / 9 * qnx;
  sishu[(nPt - 2) * maxPtCount + (nPt + 1)] = qny - 1.0 / 9 * qny;
  sishu[(nPt - 1) * maxPtCount + (nPt)] = qnx;
  sishu[(nPt - 1) * maxPtCount + (nPt + 1)] = qny;

  for (int i = 0; i < nPt; i++) {
    int k = i;
    double max = fabs(sishu[i * maxPtCount + k]);
    for (int j = i; j < nPt; j++) {
      if (max < fabs(sishu[j * maxPtCount + i])) {
        k = j;
      }
    }
    if (k != i) {
      double *iRow = sishu + (i * maxPtCount);
      double *kRow = sishu + (k * maxPtCount);

      memcpy(tempRow, iRow, maxPtCount * sizeof(double));
      memcpy(iRow, kRow, maxPtCount * sizeof(double));
      memcpy(tempRow, iRow, maxPtCount * sizeof(double));
    }

    double temp = sishu[i * maxPtCount + i];
    for (k = i; k <= nPt + 1; k++) {
      sishu[i * maxPtCount + k] = sishu[i * maxPtCount + k] / temp;
    }

    for (int nn = 0; nn < nPt; nn++) {
      if (nn != i) {
        double tt = -1.0 * sishu[nn * maxPtCount + i];
        for (int mm = 0; mm <= nPt + 1; mm++) {
          sishu[nn * maxPtCount + mm] += sishu[i * maxPtCount + mm] * tt;
        }
      }
    }
  }

  controlPoint.clear();
  for (int i = 0; i < nPt; i++) {
    Point3d pt;
    pt.x = sishu[i * maxPtCount + (nPt)];
    pt.y = sishu[i * maxPtCount + (nPt + 1)];

    controlPoint.push_back(pt);
  }

  delete[] tempRow;
  delete[] sishu;
}

int FindNearstPoint(const std::vector<Point3d> &mousePoint,
                    const Point3d &point) {
  int cur_idx = -1;

  double minDist = 1000000;
  for (int i = 0; i < mousePoint.size(); i++) {
    const Point3d &pnt = mousePoint[i];

    double detX = point.x - pnt.x;
    double detY = point.y - pnt.y;
    double dist = sqrt(detX * detX + detY * detY);

    if (dist < minDist) {
      cur_idx = i;
      minDist = dist;
    }
  }

  if (cur_idx < 0 || minDist > 0.3) {
    return -1;
  } else {
    return cur_idx;
  }
}

bool GeoBSplineCurve3::CheckCurvature(const std::vector<Point3d> &mousePoint,
                                      int ctrlBegin, int ctrlEnd) {
  std::vector<Point3d> ctrlPoint;
  ControlPoint(mousePoint, ctrlPoint);

  std::vector<Point3d> samplePoint;
  for (int i = ctrlBegin + 1; i <= ctrlEnd; i++) {
    SamplePoint(samplePoint, ctrlPoint[i], ctrlPoint[i + 1], ctrlPoint[i + 2],
                ctrlPoint[i + 3]);
  }

  std::vector<sPoint> PathPoints;

  ComputeCurve computerCurve;
  computerCurve.FittingCurveBy2Vector(samplePoint, PathPoints);

  bool bOptimizeFlag = false;
  for (int x = 0; x < PathPoints.size(); x++) {
    if (PathPoints[x].curvature > 1 / 1.6) {
      bOptimizeFlag = true;
      break;
    }
  }

  return bOptimizeFlag;
}

static bool FindAdjoinContrlPoint1(const std::vector<Point3d> &samplePoint,
                                   const std::vector<Point3d> &contrlPoint,
                                   const Point3d &point, int &idx1, int &idx2) {
  int cur_idx = FindNearstPoint(samplePoint, point);
  if (cur_idx >= 0) {
    int iCount = samplePoint.size();

    for (int i = cur_idx; i < iCount; i++) {
      int cur_idx = FindNearstPoint(contrlPoint, samplePoint[i]);
      if (cur_idx > 0) {
        idx1 = cur_idx - 1;
        idx2 = cur_idx;

        return true;
      }
    }
  }
  return false;
}

static bool FindAdjoinContrlPoint2(const std::vector<Point3d> &samplePoint,
                                   const std::vector<Point3d> &contrlPoint,
                                   const Point3d &point, int &idx1, int &idx2) {
  int cur_idx = FindNearstPoint(contrlPoint, point);
  if (cur_idx > 0) {
    idx1 = cur_idx - 1;
    idx2 = cur_idx + 1;

    return true;
  }

  return false;
}

int GeoBSplineCurve3::OptimizeCurvature(const Point3d &point) {
  int nOptimizeCounter = 0;

  std::vector<int> errorContrl;

  std::vector<Point3d> ctrlPoint;
  ControlPoint(m_pointSet, ctrlPoint);

  int nPt = ctrlPoint.size();
  for (int i = 1; i < nPt - 3; i++) {
    std::vector<Point3d> samplePoint;
    SamplePoint(samplePoint, ctrlPoint[i], ctrlPoint[i + 1], ctrlPoint[i + 2],
                ctrlPoint[i + 3]);

    //�ж������Ƿ�����
    std::vector<sPoint> PathPoints;

    ComputeCurve computerCurve;
    computerCurve.FittingCurveBy2Vector(samplePoint, PathPoints);

    for (int x = 0; x < PathPoints.size(); x++) {
      if (PathPoints[x].curvature > 1 / 1.6) {
        errorContrl.push_back(i - 1);
        break;
      }
    }
  }

  std::vector<Point3d> mousePoint;

  for (int m = 0; m < errorContrl.size(); m++) {
    int idx1_mouse = errorContrl[m];
    int idx2_mouse = errorContrl[m] + 1;

    mousePoint = m_pointSet;

    int idx1_point = FindNearstPoint(m_samplePoint, mousePoint[idx1_mouse]);
    int idx2_point = FindNearstPoint(m_samplePoint, mousePoint[idx2_mouse]);

    int iSampleCount = m_samplePoint.size();

    int nCounter = 1;
    bool bFlagUpdate = false;
    do {
      if (CheckCurvature(mousePoint, idx1_mouse, idx2_mouse)) {
        mousePoint[idx1_mouse] =
            m_samplePoint[idx1_point - nCounter < 0 ? 0
                                                    : idx1_point - nCounter];
        mousePoint[idx2_mouse] =
            m_samplePoint[idx2_point + nCounter < iSampleCount
                              ? idx2_point + nCounter
                              : iSampleCount - 1];
      } else {
        bFlagUpdate = true;
        break;
      }

    } while (nCounter++ < 5);

    if (bFlagUpdate) {
      m_pointSet[idx1_mouse] = mousePoint[idx1_mouse];
      m_pointSet[idx2_mouse] = mousePoint[idx2_mouse];

      nOptimizeCounter++;
    }
  }
  m_samplePoint.clear();
  std::vector<Point3d>(m_samplePoint).swap(m_samplePoint);
  ControlPoint(m_pointSet, m_controlPoint);
  B3Line(m_controlPoint, m_samplePoint);

  return nOptimizeCounter;
}

void GeoBSplineCurve3::Hermite(std::vector<Point3d> &items) {
  if (m_bNeedSample) {
    m_bNeedSample = false;

    m_samplePoint.clear();
    std::vector<Point3d>(m_samplePoint).swap(m_samplePoint);

    ControlPoint(m_pointSet, m_controlPoint);

    B3Line(m_controlPoint, m_samplePoint, 1.0);
  }
  items = m_samplePoint;
  direct_.clear();
  if (items.size() > 1)
    direct_.push_back(Point3d::Normalize(items[0], items[1]));
  for (int i = 1; i < items.size(); ++i) {
    if (items[i].z > 0)
      direct_.push_back(Point3d::Normalize(items[i - 1], items[i]));
  }
}

int GeoBSplineCurve3::CopyGeometry(double offset, Geometry *line) {
  int iCount = m_pointSet.size();
  if (iCount < 2 || !line) return -1;
  if (direct_.size() != iCount) {
    return Geometry::CopyGeometry(offset, line);
  }

  for (int i = 0; i < iCount; i++) {
    double dis = Point3d::Distance(m_pointSet[i + 1], m_pointSet[i]);
    Point3d v = direct_[i];
    Point3d dxy(-v.y * offset, v.x * offset, 0);
    Point3d p = m_pointSet[i];
    p += dxy;
    line->AppendVertex(p);
  }
  return 0;
}

int GeoBSplineCurve3::IsPointInEdge(const Point3d &point, Point3d &outPnt,
                                    double tolerance, double &fal) {
  const int nPointCount = 20;

  int nSize = m_pointSet.size();
  if (nSize > 1) {
    double minLength = 555555;
    int findIndex = 0;
    Point3d nearPnt;

    std::vector<Point3d> items;

    int nPt = m_controlPoint.size();
    for (int i = 1; i < nPt - 3; i++) {
      std::vector<Point3d> items;
      SamplePoint(items, m_controlPoint[i], m_controlPoint[i + 1],
                  m_controlPoint[i + 2], m_controlPoint[i + 3], 3);

      if (items.size() > 1) {
        Point3d start = items[0];

        for (int j = 1; j < items.size(); j++) {
          Point3d end = items[j];
          Point3d pt3d;
          double length = OnSegment(start, end, point, pt3d);
          if (minLength > length) {
            nearPnt = pt3d;
            findIndex = i;
            minLength = length;
          }

          start = end;
        }
      }
    }

    if (tolerance > minLength) {
      outPnt = nearPnt;
      fal = tolerance;
      return findIndex;
    }
  } else if (nSize > 0) {
    Point3d start = m_pointSet[0];

    double length = sqrt((start.x - point.x) * (start.x - point.x) +
                         (start.y - point.y) * (start.y - point.y) +
                         (start.z - point.z) * (start.z - point.z));
    if (length < tolerance) {
      fal = length;
      outPnt = start;
      return 1;
    }
  }
  return false;
}

void GeoBSplineCurve3::MoveGeometry(double x, double y, double z) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    for (int i = 0; i < nSize; i++) {
      Point3d end = m_pointSet[i];

      m_pointSet[i].x = end.x + x;
      m_pointSet[i].y = end.y + y;
      m_pointSet[i].z = end.z + z;
    }
  }
  m_bDirty = true;
}

double GeoBSplineCurve3::Length() const {
  double dSum = 0.0;

  int nSize = m_samplePoint.size();
  if (nSize > 1) {
    Point3d start = m_samplePoint[0];

    for (int i = 0; i < nSize; i++) {
      Point3d end = m_samplePoint[i];

      dSum += sqrt((start.x - end.x) * (start.x - end.x) +
                   (start.y - end.y) * (start.y - end.y) +
                   (start.z - end.z) * (start.z - end.z));

      start = end;
    }
  }
  return dSum;
}

double GeoBSplineCurve3::GetNeartPoint(const Point3d &point) const {
  Point3d retPoint;

  int iCount = m_pointSet.size();
  if (iCount > 1) {
    double distance = 2000.0;
    Point3d segStart;
    Point3d segEnd;

    Point3d start = m_pointSet[0];
    for (int i = 1; i < iCount; i++) {
      Point3d end = m_pointSet[i];

      Point3d outPt;
      double dist = OnSegment(start, end, point, outPt);
      if (dist < distance) {
        segStart = start;
        segEnd = end;

        distance = dist;
        retPoint = outPt;
      }

      start = end;
    }

    Point3d vec1(segEnd.x - segStart.x, segEnd.y - segStart.y,
                 segEnd.z - segStart.z);
    Point3d vec2(point.x - retPoint.x, point.y - retPoint.y,
                 point.z - retPoint.z);

    double dCross = vec1.x * vec2.y - vec1.y * vec2.x;

    return dCross;
  }

  return 0.0;
}

int GeoBSplineCurve3::OnPoint(const Point3d &Q, double tolerance) {
  int index = -1;
  double minFlag = 100;

  int nSize = m_pointSet.size();
  for (int i = 0; i < nSize; i++) {
    double maxVal = Mathd::Max(Mathd::Abs(m_pointSet[i].x - Q.x),
                               Mathd::Abs(m_pointSet[i].y - Q.y));
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

const void *GeoBSplineCurve3::GetDataPtr() const { return &(m_pointSet[0]); }

const int GeoBSplineCurve3::GetDataSize() const {
  size_t count = m_pointSet.size();
  return count * sizeof(Point3d);
}

}  // namespace geditor