#include "pathplanner/dpplanner.h"

namespace pathplanner {
namespace dpplanner {

const double SAMPLE_LENGTH = 3.0;
const double SAMPLE_WIDTH = 0.5;
const int PARALLEL_SAMPLE_COUNT = 4;

using geometry::Site;
using geometry::SiteVec;

DynamicProgramming::DynamicProgramming() {
  std::cout << "Dynamic Program construct" << std::endl;
}

DynamicProgramming::DynamicProgramming(const geometry::SiteVec &ref_path)
    : ref_path_(ref_path) {
  SampleWayPoints();
}

void DynamicProgramming::SampleWayPoints() {
  ref_path_length_ = 0.0;
  if (ref_path_.size() < 2) return;
  samplelist_.emplace_back(0);
  int sample_num = 1;
  for (std::size_t i = 0; i < ref_path_.size() - 1; ++i) {
    ref_path_length_ += std::hypot(ref_path_[i + 1].x - ref_path_[i].x,
                                   ref_path_[i + 1].y - ref_path_[i].y);
    if (ref_path_length_ > sample_num * SAMPLE_LENGTH) {
      samplelist_.emplace_back(i);
      ++sample_num;
    }
  }
  if (samplelist_.back() == ref_path_.size() - 1) return;
  samplelist_.emplace_back(ref_path_.size() - 1);
  return;
}

bool DynamicProgramming::CheckValid() {
  // if the sample list is less than 3,
  // it means that the reference path is too short to sample center points
  if (samplelist_.size() < 3) return false;
  return true;
}

bool DynamicProgramming::CreateRoadGraph() {
  // check the point size of the ref path.
  // it is impossible to generate waypoints if the size id too small
  if (!CheckValid()) return false;
  // push the first point to the list
  std::vector<geometry::Site> start_level_points;
  start_level_points.push_back(ref_path_[samplelist_[0]]);
  waypoints_.push_back(start_level_points);
  // sample the middle points
  for (std::size_t i = 1; i < samplelist_.size() - 1; ++i) {
    waypoints_.push_back(GetParallelSample(samplelist_[i]));
  }
  // push the end point to the list
  std::vector<geometry::Site> end_level_points;
  start_level_points.push_back(ref_path_[samplelist_[samplelist_.size() - 1]]);
  waypoints_.push_back(start_level_points);
  if (waypoints_.size() < 3) return false;
  return true;
}

std::vector<geometry::Site> DynamicProgramming::GetParallelSample(int index) {
  std::vector<geometry::Site> seeds;
  for (int i = 0; i < PARALLEL_SAMPLE_COUNT; ++i) {
    seeds.push_back(ref_path_[index]);
    geometry::Site l_point, r_point;
    l_point.x = seeds[0].x - SAMPLE_WIDTH * (i + 1) * sin(seeds[0].angle);
    l_point.y = seeds[0].y + SAMPLE_WIDTH * (i + 1) * cos(seeds[0].angle);
    l_point.angle = seeds[0].angle;
    r_point.x = seeds[0].x + SAMPLE_WIDTH * (i + 1) * sin(seeds[0].angle);
    r_point.y = seeds[0].y - SAMPLE_WIDTH * (i + 1) * cos(seeds[0].angle);
    r_point.angle = seeds[0].angle;
    seeds.push_back(l_point);
    seeds.push_back(r_point);
  }
  return seeds;
}

void DynamicProgramming::UpdateObstacleTree() { return; }

}  // namespace dpplanner
}  // namespace pathplanner
