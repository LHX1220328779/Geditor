#include "pathplanner/obstacle.h"

namespace pathplanner {
namespace dpplanner {

void Obstacle::ObstacleUpdate(const std::vector<Dppoint> &obs) {
  if (obs.empty()) obs_.clear();
  for (const auto p : obs) {
    int x = static_cast<int>(p.x * 10.0);
    int y = static_cast<int>(p.y * 10.0);
    if (x > -20 && x < 80 && y > -30 && y < 30) obs_[x].insert(y);
  }
}

bool Obstacle::CollisionDetection(const Dppoint pt, const double headlen,
                                  const double taillen, const double halflen) {
  if (obs_.empty()) return false;
  std::vector<Dppoint> poly, boundary;
  CarModel(poly, boundary, pt, headlen, taillen, halflen);
  int x_min = static_cast<int>(boundary[0].x * 10.0);
  int y_min = static_cast<int>(boundary[0].y * 10.0);
  int x_max = static_cast<int>(boundary[1].x * 10.0);
  int y_max = static_cast<int>(boundary[1].y * 10.0);
  std::map<int, std::set<int>>::iterator obs_xmin = obs_.lower_bound(x_min);
  // if all item in the obstacle map is bigger than x_min, obs_xmin return the
  // first item. if all item in the obstacle map is smaller than x_min. obs_xmin
  // return the last item. [Attention] the last item is out of boundary
  if (obs_xmin->first > x_max || !obs_.count(obs_xmin->first)) return false;
  for (; obs_xmin->first <= x_max, obs_xmin != obs_.end(); ++obs_xmin) {
    auto obs_ymin = obs_xmin->second.lower_bound(y_min);
    if (*obs_ymin > y_max || !obs_[obs_xmin->first].count(*obs_ymin)) continue;
    for (; *obs_ymin <= y_max, obs_ymin != obs_[obs_xmin->first].end();
         ++obs_ymin) {
      Dppoint temp;
      temp.x = obs_xmin->first / 10.0;
      temp.y = (*obs_ymin) / 10.0;
      if (IsInsideFootprint(temp, poly)) return true;
    }
  }
  return false;
}

bool Obstacle::CollisionDetection(const std::vector<Dppoint> &refpath,
                                  const double headlen, const double taillen,
                                  const double halflen) {
  if (refpath.size() < 2 || obs_.empty()) return false;
  if (CollisionDetection(refpath.front(), headlen, taillen, halflen))
    return true;
  double sample_len = 0.0;
  for (std::size_t i = 1; i < refpath.size(); ++i) {
    double single_len = std::hypot(refpath[i].x - refpath[i - 1].x,
                                   refpath[i].y - refpath[i - 1].y);
    sample_len += single_len;
    if (sample_len > 0.8) {
      if (CollisionDetection(refpath[i], headlen, taillen, halflen))
        return true;
    }
  }
  if (CollisionDetection(refpath.back(), headlen, taillen, halflen))
    return true;
  return false;
}

}  // namespace dpplanner
}  // namespace pathplanner
