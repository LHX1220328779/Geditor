
#pragma once

#include <vector>
#include "core/point3d.h"

namespace geditor {

typedef struct sPoint {
  double x;
  double y;
  float curvature;
} sPoint;

class ComputeCurve {
 public:
  ComputeCurve();

  ~ComputeCurve();

  double dist_get(double x1, double y1, double x2, double y2);

  void FittingCurveBy2Vector(const std::vector<Point3d> &items,
                             std::vector<sPoint> &PathPoints);
};

}  // namespace geditor
