#ifndef __COMMON_GEOMETRY_PATH_H__
#define __COMMON_GEOMETRY_PATH_H__

#include "geoheader.h"
#include "site.h"

namespace geometry {

class Path {
 public:
  Path() {
    points.clear();
    forward_flag = true;
  }
  Path(SiteVec &vec) {
    points = vec;
    forward_flag = true;
  }
  ~Path(){};

 public:
  void add(double x, double y) { points.push_back(Site(x, y)); }

 public:
  SiteVec points;
  bool forward_flag;
};

}  // namespace geometry
#endif  // __COMMON_GEOMETRY_PATH_H__