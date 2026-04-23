#ifndef INCLUDE_GEOMETRY_BSPLINE_H_
#define INCLUDE_GEOMETRY_BSPLINE_H_

#include <vector>

#include "geometry/geoheader.h"

namespace geometry {

class Bspline {
 public:
  Bspline();
  ~Bspline();
  bool GetPath(geometry::SiteVec &path, const geometry::SiteVec &sample);
};

}  // namespace geometry

#endif  // INCLUDE_GEOMETRY_BSPLINE_H_
