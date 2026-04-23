#include "core/geo_arc_line.h"

#include <algorithm>
#include <cmath>

namespace geditor {

double GeoArcLine::CalR(const Point3d &a, const Point3d &b, Point3d &center,
                        Point3d &bb, double &dir) {
  bb = a;
  double rad = -atan2(a.y, a.x);
  double d2 = b.x * b.x + b.y * b.y;
  if (d2 < 1e-2) return -1;
  double srad = sin(rad);
  double crad = cos(rad);
  double x = crad * b.x - srad * b.y;
  double y = srad * b.x + crad * b.y;
  if (fabs(y) < 1e-2) return -1;
  double r = d2 * 0.5 / y;
  center.x = -srad * -r;
  center.y = crad * r;
  double radb = atan2(y, x);
  bb.x = cos(radb * 2) * a.x - sin(radb * 2) * a.y;
  bb.y = sin(radb * 2) * a.x + cos(radb * 2) * a.y;
  dir = radb * 2;
  return fabs(r);
}
double GeoArcLine::CalR(const std::vector<Point3d> &ps, int idx,
                        Point3d &center, Point3d &bb, double &dir) {
  if (idx < 2) return -1;
  auto a = ps[idx - 1] - ps[idx - 2];
  if (idx > 2) a = bb;
  auto b = ps[idx] - ps[idx - 1];
  auto ret = CalR(a, b, center, bb, dir);
  center += ps[idx - 1];
  return ret;
}

GeoArcLine::GeoArcLine() : Geometry(GT_ARC_LINE) {}

GeoArcLine::~GeoArcLine() {}

double GeoArcLine::TheAngleFmTwoPoints(Point3d center, const Point3d point) {
  double angle1 = atan2((point.y - center.y), (point.x - center.x));
  // if (angle1 < 0) {
  //   angle1 += 2 * 3.1415926535897932384626433832795;
  // }
  return angle1;
}

void GeoArcLine::Hermite(std::vector<Point3d> &items) {
  int iCount = m_pointSet.size();
  direct_.resize(iCount);
  idx_.clear();
  if (iCount > 2) {
    Point3d center, bb;
    double dir;
    items.push_back(m_pointSet[0]);
    items.push_back(m_pointSet[1]);
    idx_.push_back(0);
    idx_.push_back(1);
    auto dir0 = Point3d::Normalize(m_pointSet[0], m_pointSet[1]);
    direct_[0] = dir0;
    direct_[1] = dir0;
    for (int i = 2; i < iCount; ++i) {
      double r = CalR(m_pointSet, i, center, bb, dir);
      if (r > 0) {
        double rad0 = TheAngleFmTwoPoints(center, m_pointSet[i - 1]);
        double det = 0.3 / r;
        int n = fabs(dir) / det;
        if (dir < 0) det = -det;
        for (int j = 0; j <= n; j++) {
          double fa = rad0 + j * det;
          items.push_back(
              Point3d(center.x + cos(fa) * r, center.y + sin(fa) * r, 0));
          idx_.push_back(i);
        }
      } else {
        items.push_back(m_pointSet[i]);
        idx_.push_back(i);
      }
      direct_[i] = Point3d::Normalize(items.back(), m_pointSet[i]);
    }
    if (Point3d::Distance(items.back(), m_pointSet.back()) > 1e-3) {
      items.push_back(m_pointSet.back());
      idx_.push_back(m_pointSet.size() - 1);
    }
  } else if (iCount > 1) {
    items.push_back(m_pointSet[0]);
    items.push_back(m_pointSet[1]);
    idx_.push_back(0);
    idx_.push_back(1);
    auto dir = Point3d::Normalize(m_pointSet[0], m_pointSet[1]);
    direct_[0] = dir;
    direct_[1] = dir;
  }
  points_ = items;
}

int GeoArcLine::CopyGeometry(double offset, Geometry *line) {
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

int GeoArcLine::IsPointInEdge(const Point3d &point, Point3d &outPnt,
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
          findIndex = j;
          minLength = length;
        }

        start = end;
      }
    }

    if (tolerance > minLength) {
      outPnt = nearPnt;
      fal = tolerance;
      return idx_[findIndex];
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

void GeoArcLine::MoveGeometry(double x, double y, double z) {
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

double GeoArcLine::Length() const {
  double dSum = 0.0;

  int nSize = points_.size();
  if (nSize > 1) {
    Point3d start = points_[0];

    for (int i = 0; i < nSize; i++) {
      Point3d end = points_[i];

      dSum += sqrt((start.x - end.x) * (start.x - end.x) +
                   (start.y - end.y) * (start.y - end.y) +
                   (start.z - end.z) * (start.z - end.z));

      start = end;
    }
  }
  return dSum;
}

double GeoArcLine::GetNeartPoint(const Point3d &point) const {
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

int GeoArcLine::OnPoint(const Point3d &Q, double tolerance) {
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

const void *GeoArcLine::GetDataPtr() const { return &(m_pointSet[0]); }

const int GeoArcLine::GetDataSize() const {
  size_t count = m_pointSet.size();
  return count * sizeof(Point3d);
}

}  // namespace geditor
