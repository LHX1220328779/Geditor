
#pragma once

#include "core/geo_polygon.h"
#include "core/geometry.h"

namespace geditor {

class GeoPolygonHole : public GeoPolygon {
 public:
  GeoPolygonHole();

  virtual ~GeoPolygonHole();

 public:
  GeoPolygon *GetHole(int index) const;

  int GetHoleCount() const;

  void AppendHole(GeoPolygon *polygon);

  int IsPointInEdge(const Point3d &point, Point3d &outPnt, double tolerance,
                    double &fal);

 public:
  void MoveVertex(const Point3d &point, int index);

  void MoveGeometry(const Point3d &vDir);

  void RemoveVertex(int index);

  Point3d GetVertex(int index) const;

 private:
  std::vector<GeoPolygon *> m_hole;
};

}  // namespace geditor
