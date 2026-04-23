#ifndef __TOOLBOX_OPTIMIZER_MEANFILT_H__
#define __TOOLBOX_OPTIMIZER_MEANFILT_H__

#include "coverage/geometry/geoheader.h"

using geometry::Site;
using geometry::SiteVec;

namespace optimizer {
class MeanFilt {
 public:
  MeanFilt() {
    iner_path_.clear();
    update_path_.clear();
  }

  ~MeanFilt() = default;

 public:
  void Interpolation(SiteVec &to_smooth_path, double density = 0.05);

  void Filt(SiteVec &anchors, double density = 0.05);

 private:
  SiteVec iner_path_;
  SiteVec update_path_;
};
}  // namespace optimizer

#endif  // __TOOLBOX_OPTIMIZER_MEANFILT_H__