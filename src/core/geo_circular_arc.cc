
#include "core/geo_circular_arc.h"
#include <algorithm>
#include "algorithm/mc_math.h"

namespace geditor {

GeoCircularArc::GeoCircularArc() : Geometry(GT_CIRCULAR_ARC) {}

GeoCircularArc::~GeoCircularArc() {}

int solveCenterPointOfCircle(std::vector<Point3d> pt, Point3d *center,
                             double *radiu) {
  double a1, b1, c1, d1;
  double a2, b2, c2, d2;
  double a3, b3, c3, d3;

  double x1 = pt[0].x, y1 = pt[0].y, z1 = pt[0].z;
  double x2 = pt[1].x, y2 = pt[1].y, z2 = pt[1].z;
  double x3 = pt[2].x, y3 = pt[2].y, z3 = pt[2].z;

  a1 = (y1 * z2 - y2 * z1 - y1 * z3 + y3 * z1 + y2 * z3 - y3 * z2);
  b1 = -(x1 * z2 - x2 * z1 - x1 * z3 + x3 * z1 + x2 * z3 - x3 * z2);
  c1 = (x1 * y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2);
  d1 = -(x1 * y2 * z3 - x1 * y3 * z2 - x2 * y1 * z3 + x2 * y3 * z1 +
         x3 * y1 * z2 - x3 * y2 * z1);

  a2 = 2 * (x2 - x1);
  b2 = 2 * (y2 - y1);
  c2 = 2 * (z2 - z1);
  d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

  a3 = 2 * (x3 - x1);
  b3 = 2 * (y3 - y1);
  c3 = 2 * (z3 - z1);
  d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

  double dInv = a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 +
                a3 * b1 * c2 - a3 * b2 * c1;
  if (dInv < 0.0000001) {
    return false;
  }

  center->x = -((b2 * c3 - b3 * c2) * d1 + (b3 * c1 - b1 * c3) * d2 +
                (b1 * c2 - b2 * c1) * d3) /
              dInv;
  center->y = -((a3 * c2 - a2 * c3) * d1 + (a1 * c3 - a3 * c1) * d2 +
                (a2 * c1 - a1 * c2) * d3) /
              dInv;
  center->z = -((a2 * b3 - a3 * b2) * d1 + (a3 * b1 - a1 * b3) * d2 +
                (a1 * b2 - a2 * b1) * d3) /
              dInv;

  *radiu = sqrt((pt[0].x - center->x) * (pt[0].x - center->x) +
                (pt[0].y - center->y) * (pt[0].y - center->y) +
                (pt[0].z - center->z) * (pt[0].z - center->z));

  return true;
}

bool ArcCross(Point3d p1, Point3d p2, Point3d p3) {
  double fRet = (p2.x - p1.x) * (p3.y - p2.y) - (p2.y - p1.y) * (p3.x - p2.x);
  return fRet > 0;
}

double TheAngleFmTwoPoints(Point3d center, const Point3d point) {
  double angle1 = atan2((point.y - center.y), (point.x - center.x));
  if (angle1 < 0) {
    angle1 += 2 * 3.1415926535897932384626433832795;
  }
  return angle1;
}

void AngleOfArc(Point3d O, const Point3d A, const Point3d B, const Point3d C,
                double &startAngle, double &sweepAngle) {
  double OA = TheAngleFmTwoPoints(O, A);
  double OB = TheAngleFmTwoPoints(O, B);
  double OC = TheAngleFmTwoPoints(O, C);

  double fMax = OA > OC ? OA : OC;
  double fMin = OA < OC ? OA : OC;

  if (OB > fMin && OB < fMax)
    sweepAngle = fMax - fMin;
  else
    sweepAngle = 2 * 3.1415926535897932384626433832795 - (fMax - fMin);

  //��ʼ��
  bool bCross = ArcCross(A, B, C);  //��ʱ��=0,˳ʱ��=1
  startAngle = bCross ? OA : OC;
}

void GeoCircularArc::Hermite(std::vector<Point3d> &items) {
  int iCount = m_pointSet.size();

  if (iCount > 2) {
    if (solveCenterPointOfCircle(m_pointSet, &m_center, &m_radius)) {
      AngleOfArc(m_center, m_pointSet[0], m_pointSet[1], m_pointSet[2],
                 m_startAngle, m_sweepAngle);
      double dDet = 0.3 / m_radius;
      for (double i = m_startAngle; i <= m_startAngle + m_sweepAngle;
           i += dDet) {
        double fa = i;
        items.push_back(Point3d(m_center.x + cos(fa) * m_radius,
                                m_center.y + sin(fa) * m_radius, 0));
      }
      double fa = m_startAngle + m_sweepAngle;
      items.push_back(Point3d(m_center.x + cos(fa) * m_radius,
                              m_center.y + sin(fa) * m_radius, 0));
    } else {
      items.push_back(m_pointSet[0]);
      items.push_back(m_pointSet[1]);
      items.push_back(m_pointSet[2]);
    }
  } else if (iCount > 1) {
    items.push_back(m_pointSet[0]);
    items.push_back(m_pointSet[1]);
  }
}

int GeoCircularArc::IsPointInEdge(const Point3d &point, Point3d &outPnt,
                                  double tolerance, double &fal) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    double minLength = 555555;
    int findIndex = 0;
    Point3d nearPnt;

    std::vector<Point3d> items;
    Hermite(items);

    if (items.size() > 1) {
      Point3d start = items[0];

      for (int j = 1; j < items.size(); j++) {
        Point3d end = items[j];
        Point3d pt3d;
        double length = OnSegment(start, end, point, pt3d);
        if (minLength > length) {
          nearPnt = pt3d;
          findIndex = 1;
          minLength = length;
        }

        start = end;
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

void GeoCircularArc::MoveGeometry(double x, double y, double z) {
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

double GeoCircularArc::Length() const {
  double dSum = 0.0;

  int nSize = m_pointSet.size();
  if (nSize > 1) {
    Point3d start = m_pointSet[0];

    for (int i = 0; i < nSize; i++) {
      Point3d end = m_pointSet[i];

      dSum += sqrt((start.x - end.x) * (start.x - end.x) +
                   (start.y - end.y) * (start.y - end.y) +
                   (start.z - end.z) * (start.z - end.z));

      start = end;
    }
  }
  return dSum;
}

double GeoCircularArc::GetNeartPoint(const Point3d &point) const {
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

int GeoCircularArc::OnPoint(const Point3d &Q, double tolerance) {
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

const void *GeoCircularArc::GetDataPtr() const { return &(m_pointSet[0]); }

const int GeoCircularArc::GetDataSize() const {
  size_t count = m_pointSet.size();
  return count * sizeof(Point3d);
}

}  // namespace geditor
