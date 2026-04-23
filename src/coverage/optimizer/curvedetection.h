#ifndef __TOOLBOX_OPTIMIZER_CURVEDETECTION_H__
#define __TOOLBOX_OPTIMIZER_CURVEDETECTION_H__

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "coverage/geometry/geoheader.h"

namespace optimizer {

class CurveDetection {
 public:
  CurveDetection();

  ~CurveDetection() = default;

  double RunDetection(geometry::SiteVec &c_path, int pathdirection = 0);

 private:
  double GetFitLength(int direction);

  bool GetTotalLength(geometry::SiteVec &path);

  double PointCheck(geometry::SiteVec &path);

 private:
  int fit_curve_smoothlen_;
  int current_path_direction_;
  double fit_curve_len;
  double fit_curve_reverse_len;
  double curve_end_;
  double fit_curve_minlen_;
  double total_length_;
  double curve_limit_;
  double curve_limit_max_;
};

}  // namespace optimizer
#endif