
#include "core/geo_rectangle.h"

namespace geditor {

GeoRectangle::GeoRectangle() : Geometry(GT_RECTANGLE) {}

GeoRectangle::~GeoRectangle() {}

void GeoRectangle::Hermite(std::vector<Point3d> &items) {
  Point3d &sPnt = m_pointSet[0];
  Point3d &ePnt = m_pointSet[1];

  V3d vDir(ePnt.x - sPnt.x, ePnt.y - sPnt.y, ePnt.z - sPnt.z);
  vDir.normalize();

  V3d sDir = V3d(sPnt.x, sPnt.y, sPnt.z) - vDir * 2.0;
  V3d eDir = sDir + vDir * 4.0;

  V3d vUp(0, 0, 1);
  V3d vLeft = vDir.cross(vUp);

  V3d vPoints[4];
  vPoints[0] = sDir + vLeft * 1.0;
  vPoints[1] = eDir + vLeft * 1.0;
  vPoints[2] = eDir - vLeft * 1.0;
  vPoints[3] = sDir - vLeft * 1.0;

  for (int i = 0; i < 4; i++) {
    items.push_back(Point3d(vPoints[i][0], vPoints[i][1], vPoints[i][2]));
  }
}

void GeoRectangle::AppendVertex(double x, double y, double z) {
  int iCount = m_pointSet.size();
  if (iCount > 1) {
    Point3d pnt = ReCalc(m_pointSet[0], m_pointSet[1]);
    Geometry::AppendVertex(pnt);
  } else {
    Geometry::AppendVertex(Point3d(x, y, z));
  }
}

Point3d GeoRectangle::ReCalc(const Point3d &p1, const Point3d &p2) const {
  const double dHalfLen = 2;

  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dz = 0.0;

  V3d vDir(dx, dy, dz);
  if (vDir.squaredNorm() > dHalfLen * dHalfLen) {
    vDir.normalize();
    return Point3d(dHalfLen * vDir[0] + m_pointSet[0].x,
                   dHalfLen * vDir[1] + m_pointSet[0].y,
                   dHalfLen * vDir[2] + m_pointSet[0].z);
  } else {
    return Point3d(p2.x, p2.y, m_pointSet[0].z);
  }
}

void GeoRectangle::MoveVertex(const Point3d &point, int index) {
  int iCount = m_pointSet.size();
  if (index == 1) {
    Point3d pnt = ReCalc(m_pointSet[0], point);

    Geometry::MoveVertex(pnt, 1);
  } else if (index == 0) {
    double dx = point.x - m_pointSet[0].x;
    double dy = point.y - m_pointSet[0].y;
    double dz = point.z - m_pointSet[0].z;

    Geometry::MoveVertex(point, 0);
    Geometry::MoveVertex(Point3d(m_pointSet[1].x + dx, m_pointSet[1].y + dy,
                                 m_pointSet[1].z + dz),
                         1);
  } else {
    Geometry::MoveVertex(point, index);
  }
}

void GeoRectangle::AppendVertex(const Point3d &point) {
  int iCount = m_pointSet.size();
  if (iCount > 0) {
    Point3d pnt = ReCalc(m_pointSet[0], point);
    Geometry::AppendVertex(pnt);
  } else {
    Geometry::AppendVertex(point);
  }
}

int GeoRectangle::IsPointInEdge(const Point3d &point, Point3d &outPnt,
                                double tolerance, double &fal) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    BoundBox3d box(m_boundBox);
    box.Extend(tolerance);
    box.v_min_[2] = -100;
    box.v_max_[2] = 100;

    if (box.Contains(V3d(point.x, point.y, point.z))) {
      Point3d start = m_pointSet[0];

      for (int i = 1; i < nSize; i++) {
        Point3d end = m_pointSet[i];

        double length = OnSegment(start, end, point, outPnt);
        if (length < tolerance) {
          fal = length;
          return i;
        }

        start = end;
      }
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

  return 0;
}
}  // namespace geditor
