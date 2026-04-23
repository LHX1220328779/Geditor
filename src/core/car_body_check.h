
#pragma once

#include "algorithm/common.h"
#include "core/compute_curve.h"

namespace geditor {

class CarBodyCheck {
 public:
  std::vector<Point3d> GetAABBPoints(const Point3d &pt, const Point3d &dir);

  double GetCurvatureError(double curvature);

  void OffsetRoadPath(const std::vector<sPoint> &items,
                      std::vector<V3d> &outArray0, std::vector<V3d> &outArray1);

  void WriteOffPath(const std::vector<sPoint> &pathPoints,
                    std::vector<Point3d> &vPointSetItem,
                    const std::vector<V3d> &outArray0,
                    const std::vector<V3d> &outArray1);
};

}  // namespace geditor
