#ifndef INCLUDE_PATHPLANNER_OBSTACLE_H_
#define INCLUDE_PATHPLANNER_OBSTACLE_H_

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "coverage/geometry/geoheader.h"
#include "coverage/pathplanner/dpcarmodel.h"

namespace pathplanner {
namespace dpplanner {

class Obstacle {
 public:
  Obstacle() = default;

  ~Obstacle() = default;

  void ObstacleUpdate(const std::vector<Dppoint> &obs);

  bool CollisionDetection(const Dppoint pt, const double headlen,
                          const double taillen, const double halflen);

  bool CollisionDetection(const std::vector<Dppoint> &refpath,
                          const double headlen, const double taillen,
                          const double halflen);

 private:
  std::map<int, std::set<int>> obs_;
};

}  // namespace dpplanner
}  // namespace pathplanner

#endif  // INCLUDE_PATHPLANNER_OBSTACLE_H_
