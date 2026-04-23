#ifndef INCLUDE_PATHPLANNER_BLANKET_HYBRIDPIPE_H_
#define INCLUDE_PATHPLANNER_BLANKET_HYBRIDPIPE_H_

namespace coverage {

using geometry::Site;
using geometry::SiteVec;

class HybridPipe {
 public:
  HybridPipe();
  ~HybridPipe();
  bool Interface();
  bool Planning();

 private:
  double x_offset_;
  double y_offset_;
};

}  // namespace coverage

#endif  // INCLUDE_PATHPLANNER_BLANKET_HYBRIDPIPE_H_
