
#include "core/geo_polygon.h"
#include "algorithm/mc_math.h"

namespace geditor {

GeoPolygon::GeoPolygon() : Geometry(GT_POLYGON) {}

GeoPolygon::GeoPolygon(GeometryType type) : Geometry(type) {}

GeoPolygon::~GeoPolygon() {}

double GeoPolygon::Length() const {
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

double GeoPolygon::GetArea() const {
  int point_num = m_pointSet.size();

  double s = 0;
  for (int i = 0; i < point_num; ++i) {
    Point3d point1 = m_pointSet[i];
    Point3d point2 = m_pointSet[(i + 1) % point_num];

    s += point1.x * point2.y - point1.y * point2.x;
  }

  return fabs(s / 2.0);
}

void GeoPolygon::Clear() { Geometry::Reset(); }

double GeoPolygon::GetNeartPoint(const Point3d &point) const {
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

int GeoPolygon::OnPoint(const Point3d &Q, double tolerance) {
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

const void *GeoPolygon::GetDataPtr() const { return &(m_pointSet[0]); }

const int GeoPolygon::GetDataSize() const {
  size_t count = m_pointSet.size();
  return count * sizeof(Point3d);
}
}  // namespace geditor
