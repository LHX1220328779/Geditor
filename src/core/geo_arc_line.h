#pragma once

#include "core/geometry.h"

namespace geditor {

class GeoArcLine : public Geometry {
 public:
  GeoArcLine();

  virtual ~GeoArcLine();

 public:
  void Hermite(std::vector<Point3d> &items) override;

  int CopyGeometry(double offset, Geometry *line) override;

  int IsPointInEdge(const Point3d &point, Point3d &outPnt, double tolerance,
                    double &fal);

  void MoveGeometry(double x, double y, double z);

  int OnPoint(const Point3d &Q, double tolerance);

  double Length() const;

  double GetNeartPoint(const Point3d &point) const override;

  const void *GetDataPtr() const;

  const int GetDataSize() const;

 private:
  double TheAngleFmTwoPoints(Point3d center, const Point3d point);

  double CalR(const Point3d &a, const Point3d &b, Point3d &center, Point3d &bb,
              double &dir);
  double CalR(const std::vector<Point3d> &ps, int idx, Point3d &center,
              Point3d &bb, double &dir);

  std::vector<Point3d> direct_;
  std::vector<Point3d> points_;
  std::vector<int> idx_;
};

}  // namespace geditor
