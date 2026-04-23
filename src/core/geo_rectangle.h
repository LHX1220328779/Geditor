
#pragma once

#include "core/geometry.h"

namespace geditor {

class GeoRectangle : public Geometry {
 public:
  GeoRectangle();

  virtual ~GeoRectangle();

 public:
  int IsPointInEdge(const Point3d &point, Point3d &outPnt, double tolerance,
                    double &fal);

  int OnPoint(const Point3d &Q, double tolerance);

  // double GetNeartPoint(const Point3d &point) const override;

  void Hermite(std::vector<Point3d> &items) override;

  const void *GetDataPtr() const;

  const int GetDataSize() const;

  virtual void AppendVertex(double x, double y, double z);

  virtual void AppendVertex(const Point3d &point);

  virtual void MoveVertex(const Point3d &point, int index);

 private:

  Point3d ReCalc(const Point3d &p1, const Point3d &p2) const;
};

}  // namespace geditor
