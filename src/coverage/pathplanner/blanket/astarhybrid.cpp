#include "pathplanner/blanket/astarhybrid.h"
#include <fstream>
#include <sstream>
#include "geometry/dubins.h"
#include "pathplanner/blanket/obsdetection.h"

using namespace std;

/**********************************************
 * Function: AstarHybrid
 * Description: the construct function for AstarHybrid
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
AstarHybrid::AstarHybrid() {}

/**********************************************
 * Function: AstarHybrid
 * Description: the construct function for AstarHybrid
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
AstarHybrid::AstarHybrid(const ccpp::Pointf& start_point,
                         const ccpp::Pointf& end_point, const cv::Mat& bmap) {
  start_ = ccpp::Node3D(start_point);
  end_ = ccpp::Node3D(end_point);
  map_ = bmap.clone();
  obsmap_ = new ObsDetection(map_);
  steplength_ = ccpp::cvcarmodel::cv_car_radius * 7.5 / 180.0 * M_PI;
  // cout << "start" << endl;
}

/**********************************************
 * Function: AstarHybrid
 * Description: rewrite the input method for younan
 * Input: None
 * Output: None
 * Return: None
 * Others:
 **********************************************/
AstarHybrid::AstarHybrid(const geometry::Site& start_point,
                         const geometry::Site& end_point,
                         const std::string& bmap) {
  ccpp::Pointf _start_point_, _end_point_;
  _start_point_.x = start_point.x * 20.0;
  _start_point_.y = start_point.y * 20.0;
  _start_point_.yaw = NormalRad(start_point.angle * M_PI / 180.0);
  _end_point_.x = end_point.x * 20.0;
  _end_point_.y = end_point.y * 20.0;
  _end_point_.yaw = NormalRad(end_point.angle * M_PI / 180.0);
  start_ = ccpp::Node3D(_start_point_);
  end_ = ccpp::Node3D(_end_point_);
  map_ = MapBinary(bmap);
  obsmap_ = new ObsDetection(map_);
  steplength_ = ccpp::cvcarmodel::cv_car_radius * 7.5 / 180.0 * M_PI;
  // cout << "start" << endl;
}

/**********************************************
 * Function: AstarHybrid
 * Description: the desconstruct function for AstarHybrid
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
AstarHybrid::~AstarHybrid() {}

bool AstarHybrid::MapBinary(const std::string& mappath) {
  cv::Mat color_map;
  cv::Mat gray_map;
  cv::Mat binary_map;
  color_map = cv::imread(mappath).clone();
  if (color_map.empty()) {
    std::cout << "orig image is empty" << std::endl;
    return false;
  }
  for (int i = 0; i < color_map.rows; i++) {
    for (int j = 0; j < color_map.cols; j++) {
      if (color_map.at<cv::Vec3b>(i, j)[0] == 0 &&
          color_map.at<cv::Vec3b>(i, j)[1] == 0 &&
          color_map.at<cv::Vec3b>(i, j)[2] == 0) {
        color_map.at<cv::Vec3b>(i, j)[0] = 0;
        color_map.at<cv::Vec3b>(i, j)[1] = 0;
        color_map.at<cv::Vec3b>(i, j)[2] = 0;
      } else {
        color_map.at<cv::Vec3b>(i, j)[0] = 255;
        color_map.at<cv::Vec3b>(i, j)[1] = 255;
        color_map.at<cv::Vec3b>(i, j)[2] = 255;
      }
    }
  }
  // turn to the gray image
  cv::cvtColor(color_map, gray_map, cv::COLOR_BGR2GRAY);
  cv::threshold(gray_map, binary_map, 145, 255, cv::THRESH_BINARY);
  map_ = binary_map.clone();
  return true;
}

float AstarHybrid::NormalRad(float angle) {
  float a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  if (fabs(a) < 1e-3 || fabs(a - 2.0 * M_PI) < 1e-3) return -M_PI;
  if (fabs(a - M_PI) < 1e-3) return 0.0;
  return a - M_PI;
}

/**********************************************
 * Function: IsGoal
 * Description:
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool AstarHybrid::IsGoal(const ccpp::Node3D& node) {
  if (fabs(node.x - end_.x) <= 1e-2 && fabs(node.y - end_.y) <= 1e-2 &&
      fabs(node.yaw - end_.yaw) <= 1e-2) {
    return true;
  }
  return false;
}

bool AstarHybrid::Dubinsvalid(ccpp::Node3D& cur,
                              std::vector<ccpp::Pointf>& d_curve,
                              float& d_length) {
  bool valid_flag = true;
  double q0[3] = {cur.x, cur.y, cur.yaw};
  double q1[3] = {end_.x, end_.y, end_.yaw};
  geometry::DubinsPath dubins_path;
  // geometry::dubins_init(q0, q1, ccpp::cvcarmodel::cv_car_radius,
  // &dubins_path);
  geometry::dubins_shortest_path(&dubins_path, q0, q1,
                                 ccpp::cvcarmodel::cv_car_radius);
  double x = 0.0;
  double length = geometry::dubins_path_length(&dubins_path);
  while (x < length) {
    double q[3];
    geometry::dubins_path_sample(&dubins_path, x, q);
    if (obsmap_->CheckObsCollision(ccpp::Pointf(q[0], q[1], q[2]))) {
      valid_flag = false;
    }
    d_curve.push_back(ccpp::Pointf(q[0], q[1], q[2]));
    x += 2;
  }
  d_curve.push_back(ccpp::Pointf(end_.x, end_.y, end_.yaw));
  d_length = length;
  return valid_flag;
}

bool AstarHybrid::UpdateH(ccpp::Node3D& cur,
                          std::vector<ccpp::Pointf>& d_curve) {
  float d_length;
  float o_length;
  bool cur_valid_flag = Dubinsvalid(cur, d_curve, d_length);
  o_length = sqrt(pow(cur.x - end_.x, 2) + pow(cur.y - end_.y, 2));
  // std:;cout << "length:" << d_length << o_length;
  cur.h = o_length > d_length ? o_length : d_length;
  // cur.h = o_length;
  if (cur_valid_flag) {
    return true;
  }
  return false;
}

bool AstarHybrid::SetOpen(ccpp::Node3D& cur) {
  cur.o = true;
  cur.c = false;
  return true;
}

bool AstarHybrid::SetClose(ccpp::Node3D& cur) {
  cur.o = false;
  cur.c = true;
  return true;
}

bool AstarHybrid::AddCloseList(ccpp::Node3D& node) { return true; }

std::vector<ccpp::Node3D> AstarHybrid::CreateForwardPoints(ccpp::Node3D& node) {
  std::vector<ccpp::Node3D> forwards;
  for (int i = 0; i < 3; i++) {
    float x, y, yaw;
    ccpp::Node3D p;
    p.x = node.x + dx[i] * cos(node.yaw) - dy[i] * sin(node.yaw);
    p.y = node.y + dx[i] * sin(node.yaw) + dy[i] * cos(node.yaw);
    p.yaw = NormalRad(node.yaw + dt[i]);
    p.g = 0;
    p.h = 0;
    p.pre = &node;
    forwards.push_back(p);
  }
  return forwards;
}

std::vector<ccpp::Pointf> AstarHybrid::AHPlanning() {
  int index = 0;
  int end_flag = 0;
  ccpp::UnorderT closelist_;
  ccpp::PriorityList openlist;
  ccpp::Node3D nstart(start_);
  ccpp::Node3D nend(end_);
  ccpp::Node3D last;
  std::vector<ccpp::Pointf> path;
  std::vector<ccpp::Pointf> pathfit;
  std::vector<ccpp::Pointf> d_curve;
  // cout << "start:" << nstart.x << ","
  //                  << nstart.y << ","
  //                  << nstart.yaw << endl;
  // cout << "end:" << nend.x << ","
  //                << nend.y << ","
  //                << nend.yaw << endl;
  // closelist_.push(nstart, nstart);
  // cost_so_far_.push(nstart, 0.0);
  closelist_[nstart] = nstart;
  cout << closelist_.size() << endl;
  cost_so_far_[nstart] = 0.0;
  bool start_valid = UpdateH(nstart, d_curve);
  // std::cout << "," << nstart.h << "," << nstart.g << std::endl;
  SetOpen(nstart);
  if (start_valid) {
    nend.pre = &nstart;
    closelist_[nend] = nstart;
    std::cout << "no obstacles" << std::endl;
    end_flag = END_START;
    last = nstart;
    return d_curve;
    // return nend;
  }
  openlist.push(nstart);

  while (!openlist.empty()) {
    // cout << openlist.size() << endl;
    ccpp::Node3D cur_node = openlist.top();
    openlist.pop();
    // cout << openlist.size() << endl;
    // cout << cur_node.x << ","
    //      << cur_node.y << ","
    //      << cur_node.yaw << ","
    //      << cur_node.g << ","
    //      << cur_node.h << ","
    //      << cur_node.g + cur_node.h << ",pop" << endl;
    if (IsGoal(cur_node)) {
      end_flag = END_NORMAL;
      last = cur_node;
      cout << "to the goal" << endl;
      break;
      // return cur_node;
    }
    if (index > 1000) {
      end_flag = END_ERROR;
      cout << "too many times" << endl;
      break;
    }
    for (auto& node : CreateForwardPoints(cur_node)) {
      // cout << "start iterate" << endl;
      if ((!closelist_.count(node)) &&
          (!obsmap_->CheckObsCollision(
              ccpp::Pointf(node.x, node.y, node.yaw)))) {
        // cout << "no collision" << endl;
        float new_g = cur_node.g + steplength_ * 5;
        // node.g += cur_node->g + steplength_;
        if ((!cost_so_far_.count(node)) ||
            (new_g + 1e-1 < cost_so_far_[node])) {
          std::vector<ccpp::Pointf> d_curves;
          node.g = new_g;
          bool dubins_flag = UpdateH(node, d_curves);
          cost_so_far_[node] = new_g;
          closelist_[node] = cur_node;
          // cout << node.x << ","
          //      << node.y << ","
          //      << node.yaw << ","
          //      << node.g << ","
          //      << node.h << ","
          //      << node.g + node.h << endl;
          openlist.push(node);
          if (dubins_flag) {
            cout << "is no obstacles" << endl;
            nend.pre = &node;
            end_flag = END_LAST;
            last = node;
            break;
            // return nend;
          }
        }
      }
    }
    if (end_flag != 0) break;
    index++;
    // cout << "index:" << index << endl;
  }
  path = ReconstructPath(last, end_flag, closelist_);
  pathfit = SamplePath(path);

  cout << "end_flag:" << end_flag << ", path size:" << path.size() << endl;
  return pathfit;
}

bool AstarHybrid::AHPlanning(geometry::SiteVec& final) {
  int index = 0;
  int end_flag = 0;
  ccpp::UnorderT closelist_;
  ccpp::PriorityList openlist;
  ccpp::Node3D nstart(start_);
  ccpp::Node3D nend(end_);
  ccpp::Node3D last;
  std::vector<ccpp::Pointf> path;
  std::vector<ccpp::Pointf> pathfit;
  std::vector<ccpp::Pointf> d_curve;
  cout << "start:" << nstart.x << "," << nstart.y << "," << nstart.yaw << endl;
  cout << "end:" << nend.x << "," << nend.y << "," << nend.yaw << endl;
  // closelist_.push(nstart, nstart);
  // cost_so_far_.push(nstart, 0.0);
  closelist_[nstart] = nstart;
  cout << closelist_.size() << endl;
  cost_so_far_[nstart] = 0.0;
  bool start_valid = UpdateH(nstart, d_curve);
  // std::cout << "," << nstart.h << "," << nstart.g << std::endl;
  SetOpen(nstart);
  if (start_valid) {
    nend.pre = &nstart;
    closelist_[nend] = nstart;
    std::cout << "no obstacles" << std::endl;
    end_flag = END_START;
    last = nstart;
    for (int i = 0; i < d_curve.size(); i++) {
      final.push_back(geometry::Site(d_curve[i].x / 20.0, d_curve[i].y / 20.0,
                                     d_curve[i].yaw * 180.0 / M_PI));
    }
    return true;
    // return nend;
  }
  openlist.push(nstart);

  while (!openlist.empty()) {
    // cout << openlist.size() << endl;
    ccpp::Node3D cur_node = openlist.top();
    openlist.pop();
    // cout << openlist.size() << endl;
    // cout << cur_node.x << ","
    //      << cur_node.y << ","
    //      << cur_node.yaw << ","
    //      << cur_node.g << ","
    //      << cur_node.h << ","
    //      << cur_node.g + cur_node.h << ",pop" << endl;
    if (IsGoal(cur_node)) {
      end_flag = END_NORMAL;
      last = cur_node;
      cout << "to the goal" << endl;
      break;
      // return cur_node;
    }
    if (index > 1000) {
      end_flag = END_ERROR;
      cout << "too many times" << endl;
      break;
    }
    for (auto& node : CreateForwardPoints(cur_node)) {
      // cout << "start iterate" << endl;
      if ((!closelist_.count(node)) &&
          (!obsmap_->CheckObsCollision(
              ccpp::Pointf(node.x, node.y, node.yaw)))) {
        // cout << "no collision" << endl;
        float new_g = cur_node.g + steplength_ * 5;
        // node.g += cur_node->g + steplength_;
        if ((!cost_so_far_.count(node)) ||
            (new_g + 1e-1 < cost_so_far_[node])) {
          std::vector<ccpp::Pointf> d_curves;
          node.g = new_g;
          bool dubins_flag = UpdateH(node, d_curves);
          cost_so_far_[node] = new_g;
          closelist_[node] = cur_node;
          // cout << node.x << ","
          //      << node.y << ","
          //      << node.yaw << ","
          //      << node.g << ","
          //      << node.h << ","
          //      << node.g + node.h << endl;
          openlist.push(node);
          if (dubins_flag) {
            cout << "is no obstacles" << endl;
            nend.pre = &node;
            end_flag = END_LAST;
            last = node;
            break;
            // return nend;
          }
        }
      }
    }
    if (end_flag != 0) break;
    index++;
    // cout << "index:" << index << endl;
  }
  path = ReconstructPath(last, end_flag, closelist_);
  SamplePath(path, final);
  // cout << "index:" << index << endl;
  cout << "end_flag:" << end_flag << ", path size:" << path.size() << endl;
  return true;
}

std::vector<ccpp::Pointf> AstarHybrid::ReconstructPath(
    ccpp::Node3D& last, int& flag, ccpp::UnorderT& closelist_) {
  std::vector<ccpp::Pointf> path;
  ccpp::Node3D cur = last;
  cout << "close list size" << closelist_.size() << endl;
  if (flag == END_NORMAL) {
    while (!(cur == start_)) {
      path.push_back(ccpp::Pointf(cur.x, cur.y, cur.yaw));
      // cout << cur.x << ","
      //     << cur.y << ","
      //     << cur.yaw << endl;
      cur = closelist_[cur];
    }
    path.push_back(ccpp::Pointf(cur.x, cur.y, cur.yaw));
    std::reverse(path.begin(), path.end());
  } else if (flag == END_LAST) {
    while (!(cur == start_)) {
      path.push_back(ccpp::Pointf(cur.x, cur.y, cur.yaw));
      // cout << cur.x << ","
      //     << cur.y << ","
      //     << cur.yaw << endl;
      cur = closelist_[cur];
    }
    path.push_back(ccpp::Pointf(cur.x, cur.y, cur.yaw));
    std::reverse(path.begin(), path.end());
    std::vector<ccpp::Pointf> d_curve;
    float d_length;
    bool cur_valid_flag = Dubinsvalid(last, d_curve, d_length);
    for (int i = 0; i < d_curve.size(); i++) {
      // cout << d_curve[i].x << ","
      //      << d_curve[i].y << ","
      //      << d_curve[i].yaw << endl;
      path.push_back(d_curve[i]);
    }
  }
  return path;
}

std::vector<ccpp::Pointf> AstarHybrid::SamplePath(
    std::vector<ccpp::Pointf>& path) {
  std::vector<ccpp::Pointf> fit;
  if (!path.empty()) {
    for (int i = 0; i < path.size() - 1; i++) {
      float length =
          std::hypot(path[i].x - path[i + 1].x, path[i].y - path[i + 1].y);
      fit.push_back(path[i]);
      // if (length > 2.5) {
      //  double angle_dis = NormalRad(NormalRad(path[i+1].yaw)
      //  -NormalRad(path[i].yaw)); if (angle_dis >= 1e-2) { } else if
      //  (angle_dis <= -1e-2) { } else {
      //  }
      //}

      if (length > 2.2) {
        ccpp::Pointf p;
        p.x = (path[i].x + path[i + 1].x) / 2.0;
        p.y = (path[i].y + path[i + 1].y) / 2.0;
        p.yaw = path[i].yaw;
        fit.push_back(p);
      }
    }
    fit.push_back(path.back());
  }
  return fit;
}

bool AstarHybrid::SamplePath(std::vector<ccpp::Pointf>& path,
                             geometry::SiteVec& final) {
  if (path.empty()) {
    return false;
  }
  for (int i = 0; i < path.size() - 1; i++) {
    float length =
        std::hypot(path[i].x - path[i + 1].x, path[i].y - path[i + 1].y);
    final.push_back(geometry::Site(path[i].x / 20.0, path[i].y / 20.0,
                                   path[i].yaw * 180.0 / M_PI));
    if (length > 2.2) {
      geometry::Site p;
      p.x = (path[i].x + path[i + 1].x) / 2.0;
      p.y = (path[i].y + path[i + 1].y) / 2.0;
      p.angle = path[i].yaw * 180.0 / M_PI;
      final.push_back(p);
    }
  }
  final.push_back(geometry::Site(path.back().x / 20.0, path.back().y / 20.0,
                                 path.back().yaw * 180.0 / M_PI));
  return true;
}
