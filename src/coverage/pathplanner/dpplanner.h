#ifndef INCLUDE_PATHPLANNER_DPPLANNER_H_
#define INCLUDE_PATHPLANNER_DPPLANNER_H_

#include <iostream>
#include <string>
#include <vector>
#include "coverage/geometry/geoheader.h"

namespace pathplanner {
namespace dpplanner {

struct SLpoint : geometry::Site {
  double s;
  double l;
};

class DynamicProgramming {
 public:
  /**
   * @brief : the constructor for DynamicProgramming class
   * @param : none
   * @return: none
   **/
  DynamicProgramming();

  /**
   * @brief : the constructor for DynamicProgramming class
   * @param : none
   * @return: none
   **/
  explicit DynamicProgramming(const geometry::SiteVec &ref_path);

  /**
   * @brief : the deconstructor for DynamicProgramming class
   * @param : none
   * @return: none
   **/
  ~DynamicProgramming() = default;

  /**
   * @brief : get the start point
   * @param : node
   * @return: the start node
   **/
  geometry::Site GetStart() const { return start_; }

  /**
   * @brief : get the end point
   * @param : none
   * @return: the end node
   **/
  geometry::Site GetGoal() const { return goal_; }

  /**
   * @brief : path plan using dynamic programming method
   * @param : none
   * @return: true if we can get a optimized path;
              false if we can not find a valid path
  **/
  bool DPPlanner();

 private:
  /**
   * @brief : get the length of the reference path
   * @param : none
   * @return: none
   **/
  void SampleWayPoints();

  /**
   * @brief : create a graph for way points
   * @param : none
   * @return: true if we can sample the reference path
   **/
  bool CheckValid();

  /**
   * @brief : create a graph for way points
   * @param : none
   * @return: true if we can sample the reference path
   **/
  bool CreateRoadGraph();

  /**
   * @brief : get the parallel samplepoints from the samplelist
   * @param : none
   * @return: the parallel samplepoints
   **/
  std::vector<geometry::Site> GetParallelSample(int index);

  /**
   * @brief : get the parallel samplepoints from the samplelist
   * @param : none
   * @return: the parallel samplepoints
   **/
  void UpdateObstacleTree();

 private:
  double ref_path_length_;
  geometry::Site start_;
  geometry::Site goal_;
  geometry::SiteVec ref_path_;
  std::vector<std::vector<geometry::Site> > waypoints_;
  std::vector<int> samplelist_;
};

}  // namespace dpplanner
}  // namespace pathplanner

#endif  // INCLUDE_PATHPLANNER_DPPLANNER_H_
