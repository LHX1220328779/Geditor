#ifndef __TOOLBOX_PATHPLANNER_ASTAR_H__
#define __TOOLBOX_PATHPLANNER_ASTAR_H__

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "coverage/geometry/geoheader.h"

namespace astar {

using geometry::Site;
using geometry::SiteVec;

const double EPSINON = 1e-5;
const double kMax = std::numeric_limits<double>::max();

/**
/* @brief : the point designed for A* algorithm
/* @param : none
/* @return: none
**/
struct Point {
  Point() {
    site = Site();
    G = 0;
    H = 0;
    parent = nullptr;
  }

  ~Point() = default;

  Site site;
  double G;  // G, the cost of going from the parent point to current point
  double H;  // H, the distance from current point to goal point
  std::shared_ptr<Point> parent;

  void operator=(const Point &p) {
    this->site = p.site;
    this->G = p.G;
    this->H = p.H;
    this->parent = p.parent;
  }

  double F() const { return G + H; };
};

/**
/* @brief : used for find the optimal start point
/* @param : none
/* @return: none
**/
struct Section {
  Section() {
    head = Site();
    tail = Site();
  }

  Section(Site &s1, Site &s2) {
    head = Site(s1);
    tail = Site(s2);
  }

  Site head;
  Site tail;

  void operator=(Section &s) {
    head = s.head;
    tail = s.tail;
  }

  double distance() { return (tail - head).mold(); }

  double angle() {
    return std::atan2(tail.y - head.y, tail.x - head.x) * 180 / M_PI;
  }

  double distance2point(Site p) {
    double dis = kMax;
    double judge_relative_pos =
        ((p.x - head.x) * (tail.x - head.x) +
         (p.y - head.y) * (tail.y - head.y)) /
        (pow((tail.x - head.x), 2) + pow((tail.y - head.y), 2));

    if (judge_relative_pos <= 0) {
      dis = std::hypot(p.x - head.x, p.y - head.y);
    } else if (judge_relative_pos >= 1) {
      dis = kMax;
    } else {
      dis = std::fabs((head.x - p.x) * (tail.y - p.y) -
                      (tail.x - p.x) * (head.y - p.y)) /
            std::hypot(tail.x - head.x, tail.y - head.y);
    }
    return dis;
  }
};

class Astar {
 public:
  Astar(std::string file_path);

  ~Astar() = default;

 public:
  /**
   * @brief : set the pos of a car, find a path from this position
   * @param : none
   * @return: none
   **/
  void SetLocPos(double x, double y, double angle);

  /**
   * @brief : the main interface of this algorithm
   * @param : goal, the indexs of goal
   * @param : anchor, return the points' indexs that the path need to pass
   * @return: if get a path return true, if not, retrun false
   **/
  bool GeneratePath(int goal, SiteVec &result, std::vector<int> &anchor);

  /**
   * @brief : the main interface of this algorithm
   * @param : goal, the indexs of goal
   * @param : anchor, return the points' indexs that the path need to pass
   * @return: if get a path return true, if not, retrun false
   **/
  bool GeneratePath(Site &start, Site &goal, SiteVec &result);

  /**
   * @brief : erase a bad path
   * @param : the index of the path's head and tail
   * @return: none
   **/
  void ErasePath(int from, int to);

  /**
   * @brief : erase a bad point, that cann't pass
   * @param : target, is a vector of points' indexs
   * @return: none
   **/
  void ErasePoint(std::vector<int> target);

 private:
  /**
   * @brief : load globpath-map, get the coordination of main points and their
   *          topological relationship
   * @param : none
   * @return: none
   **/
  void InputData();

  /**
   * @brief : find the optimal point based on the current vehicle position as
   *          the starting point of the search algorightm
   * @param : none
   * @return: none
   **/
  int GetStart();

  /**
   * @brief : the kernal of this algorithm
   * @param : the indeopen_list_x of start index and goal index
   * @return: bool, true if success, false otherwise
   **/
  bool GetAnchors(const int &begin, const int &end);

  /**
   * @brief : get the index based on the coordination
   * @param : site, the coordination
   * @return: the index of the coordination
   **/
  int FindIndex(Site &site) {
    for (int i = 0; i < verts_num_; ++i) {
      if ((site - verts_[i]).mold() < EPSINON) return i;
    }
    return -1;
  }

  /**
   * @brief : output calculation result
   * @param : the name of the seg, for example, 1 for 1-seg
   * @return: none
   **/
  void PushResult(int goal);

  /**
   * @brief : get the status of this algorithm
   * @param : none
   * @return: bool, true if OK, false if not
   **/
  bool OK() const { return status_; }

  /**
   * @brief : set the status of this algorithm
   * @param : true, if OK; false if not
   * @return: none
   **/
  void OK(bool status) { status_ = status; }

 private:
  /**
   * @brief : determine if the specific point is in the open list
   * @param : to_detect, the specific point
   * @param : transport, the pointer of the specific point in open list
   * @return: true if in open list, false in other cases
   **/
  bool InOpenlist(std::shared_ptr<Point> to_detect) {
    bool result = false;
    for (auto it = open_list_.begin(); it != open_list_.end(); ++it) {
      if ((*it)->site == to_detect->site) {
        result = true;
        to_detect = *it;
        break;
      }
    }
    return result;
  }

  /**
   * @brief : calculate G value of a point, consider about angle
   * @param : parent point of current point
   * @return: G value
   **/
  double GetGvalue(std::shared_ptr<Point> target,
                   std::shared_ptr<Point> current);

  /**
   * @brief : calculate H value of a point, consider about angle
   * @param : target point
   * @return: H value
   **/
  double GetHvalue(std::shared_ptr<Point> current);

 private:
  bool status_;
  std::string file_path_;

  int verts_num_;
  SiteVec verts_;
  SiteVec update_path_;
  std::vector<SiteVec> erased_path_;
  std::vector<std::pair<int, int>> want_erase_;

  std::vector<std::vector<double>> cost_;
  std::vector<std::shared_ptr<Point>> open_list_;
  std::vector<std::shared_ptr<Point>> close_list_;

  Site locpos_;
  Site goal_;
};

}  // namespace astar

#endif  // __TOOLBOX_PATHPLANNER_ASTAR_H__
