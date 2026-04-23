#include "coverage/geometry/dubins.h"

namespace geometry {
void Dubins::GetPath(SiteVec& list, Site start, Site end, double radius,
                     double density) {
  double q0[] = {start.x, start.y, start.angle * M_PI / 180};
  double q1[] = {end.x, end.y, end.angle * M_PI / 180};

  DubinsPath path;
  dubins_shortest_path(&path, q0, q1, radius);

  double q[3];
  double x = 0.0;
  double length = dubins_path_length(&path);
  while (x < length) {
    dubins_path_sample(&path, x, &q[0]);
    list.emplace_back(q[0], q[1]);
    x += density;
  }
}
}  // namespace geometry
