
#pragma once

#include "core/geometry.h"

namespace geditor {

class GeoCircularArc : public Geometry {
 public:
  GeoCircularArc();

  virtual ~GeoCircularArc();

 public:
  void Hermite(std::vector<Point3d> &items) override;

  int IsPointInEdge(const Point3d &point, Point3d &outPnt, double tolerance,
                    double &fal);

  void MoveGeometry(double x, double y, double z);

  int OnPoint(const Point3d &Q, double tolerance);

  double Length() const;

  double GetNeartPoint(const Point3d &point) const override;

  const void *GetDataPtr() const;

  const int GetDataSize() const;

 private:
  Point3d m_center;
  double m_radius;
  double m_startAngle;
  double m_sweepAngle;
};

}  // namespace geditor
