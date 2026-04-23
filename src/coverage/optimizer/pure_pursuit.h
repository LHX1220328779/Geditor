#ifndef __TOOLBOX_OPTIMIZER_PURE_PURSUIT_H__
#define __TOOLBOX_OPTIMIZER_PURE_PURSUIT_H__

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include "coverage/geometry/geoheader.h"
#include "coverage/math/math_common.h"

namespace optimizer {

const double k = 0.1;    //
const double dt = 0.05;  // (s)
const double V = 1.0;    // (m/s)
const double MAX = std::numeric_limits<double>::max();

class PurePursuit {
 public:
  PurePursuit();

  ~PurePursuit() = default;

 public:
  /**
   * @breif : to smooth a path with pure persuit
   * @param : path  to store the smoothed path
   * @return:  0 success
   * @return: -1 failure
   **/
  int Smooth(SiteVec &path);

  /**
   * @breif : init this smoother
   * @param : param_L  wheel base
   * @param : param_R  mininum tuning radius
   * @return: none
   **/
  void Init(const Site status, const double param_L = 0.7,
            const double param_R = 1.5, const double param_Lf = 2.5);

 private:
  /**
   * @breif : find target index by the latest status
   * @return:  0    success
   * @return: -1    failure
   **/
  int TargetInd();

  /**
   * @breif : calculate the input of vehicle
   * @param : delta      to store the angle of front wheel
   * @return:  0         success
   * @return: -1         failure
   **/
  int Control(double &delta);

  /**
   * @breif : update the status of vehicle
   * @param : delta target front wheel angle
   * @return:  0    success
   * @return: -1    failure
   **/
  int Update(const double delta);

  const double MaxDelta() { return std::atan2(L_, R_); }

 private:
  size_t target_index_;
  size_t last_near_;

  double L_;
  double R_;
  double Lf_;

  SiteVec raw_path_;
  SiteVec update_path_;
  Site status_;
};

}  // namespace optimizer

#endif  // __TOOLBOX_OPTIMIZER_PURE_PURSUIT_H__
