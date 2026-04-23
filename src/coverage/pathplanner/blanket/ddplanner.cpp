#include "pathplanner/blanket/ddplanner.h"
#include "cmath"
#include "geometry/dubins.h"
#include "pathplanner/blanket/commontypes.h"

DDplanner::DDplanner(const ccpp::Pointf& start, const ccpp::Pointf& goal,
                     const cv::Mat& bmap) {
  start_ = start;
  goal_ = goal;
  binary_map_ = bmap.clone();
  min_xrand_ = 0;
  max_xrand_ = binary_map_.cols;
  min_yrand_ = 0;
  max_yrand_ = binary_map_.rows;
  goal_sample_rate_ = 10;
  max_iter_ = 100;

  goal_dist_limit_ = 1.0;
  goal_yaw_limit_ = 0.3;

  obsdect_ = new ObsDetection(binary_map_);
}

DDplanner::DDplanner() {}

DDplanner::~DDplanner() {}

bool DDplanner::Initialize() {
  root_ = new RRTNode;
  root_->cur = start_;
  root_->parent = NULL;
  root_->cost = 0.0;
  lastnode_ = root_;
  nodelist_.push_back(root_);

  return true;
}

RRTNode* DDplanner::GetRandomNode() {
  RRTNode* temp;
  bool correct_flag = true;
  while (correct_flag) {
    if (rand() % 100 < 20) {
      temp = new RRTNode;
      temp->cur = goal_;
      return temp;
    } else {
      int x_r = rand() % (max_xrand_ - min_xrand_ + 1) + min_xrand_;
      int y_r = rand() % (max_yrand_ - min_yrand_ + 1) + min_yrand_;
      if (binary_map_.at<uchar>(y_r, x_r) == 0) continue;
      float x_rand = static_cast<float>(x_r);
      float y_rand = static_cast<float>(y_r);
      float angle_rand = std::fmod(rand(), 360.0);
      angle_rand = angle_rand >= 180.0 ? angle_rand - 360.0 : angle_rand;
      float yaw_rand = angle_rand * M_PI / 180.0;
      if (!obsdect_->CheckObsCollision(
              ccpp::Pointf(x_rand, y_rand, yaw_rand))) {
        std::cout << "rand:" << x_rand << "," << y_rand << "," << yaw_rand
                  << std::endl;
        std::cout << "valid" << std::endl;
        temp = new RRTNode;
        temp->cur = ccpp::Pointf(x_rand, y_rand, yaw_rand);
        return temp;
      }
    }
  }
  return NULL;
}

RRTNode* DDplanner::GetNearestNode(RRTNode* node) {
  float mindist = std::numeric_limits<float>::max();
  RRTNode* nearest = NULL;
  for (int i = 0; i < nodelist_.size(); i++) {
    float dist = pow(nodelist_[i]->cur.x - node->cur.x, 2) +
                 pow(nodelist_[i]->cur.y - node->cur.y, 2) +
                 pow(nodelist_[i]->cur.yaw - node->cur.yaw, 2);
    if (dist < mindist) {
      mindist = dist;
      nearest = nodelist_[i];
    }
  }
  return nearest;
}

std::vector<ccpp::Pointf> DDplanner::DubinsConfig(RRTNode* start,
                                                  RRTNode* end) {
  std::vector<ccpp::Pointf> d_curve;
  double q0[3] = {start->cur.x, start->cur.y, start->cur.yaw};
  double q1[3] = {end->cur.x, end->cur.y, end->cur.yaw};
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
    d_curve.push_back(ccpp::Pointf(q[0], q[1], q[2]));
    x += 3;
  }
  d_curve.push_back(end->cur);
  return d_curve;
}

float DDplanner::DubinsLength(RRTNode* start, RRTNode* end) {
  double q0[3] = {start->cur.x, start->cur.y, start->cur.yaw};
  double q1[3] = {end->cur.x, end->cur.y, end->cur.yaw};
  geometry::DubinsPath dubins_path;
  // geometry::dubins_init(q0, q1, ccpp::cvcarmodel::cv_car_radius,
  // &dubins_path);
  geometry::dubins_shortest_path(&dubins_path, q0, q1,
                                 ccpp::cvcarmodel::cv_car_radius);
  double x = 0.0;
  double length = geometry::dubins_path_length(&dubins_path);
  return length;
}

// TODO
bool DDplanner::CollisionDetection(std::vector<ccpp::Pointf>& curve) {
  for (int i = 0; i < curve.size(); i++) {
    if (obsdect_->CheckObsCollision(curve[i])) {
      return true;
    }
  }
  return false;
}

std::vector<int> DDplanner::GetNearNodes(RRTNode* node,
                                         std::vector<RRTNode*>& list) {
  std::vector<int> nearnum;
  int cur_count = nodelist_.size();
  float r = 10.0 * sqrt(log(cur_count) / cur_count);
  // float r = 40;
  for (int i = 0; i < cur_count; i++) {
    float dist = pow(node->cur.x - nodelist_[i]->cur.x, 2) +
                 pow(node->cur.y - nodelist_[i]->cur.y, 2) +
                 pow(node->cur.yaw - nodelist_[i]->cur.yaw, 2);
    if (dist < r) {
      list.push_back(nodelist_[i]);
      nearnum.push_back(i);
    }
  }
  return nearnum;
}

// I doubt the correctness of this function
RRTNode* DDplanner::chooseparent(RRTNode* node,
                                 std::vector<RRTNode*>& nearnodes) {
  if (nearnodes.size() == 0) {
    return node;
  }
  float mincost = node->cost;
  int minindex = 0;
  for (int i = 0; i < nearnodes.size(); i++) {
    std::vector<ccpp::Pointf> newcurve = DubinsConfig(nearnodes[i], node);
    float newcost = DubinsLength(nearnodes[i], node) + nearnodes[i]->cost;
    if (!CollisionDetection(newcurve)) {
      if (newcost < mincost) {
        mincost = newcost;
        minindex = i;
      }
    }
  }
  std::vector<ccpp::Pointf> updatecurve =
      DubinsConfig(nearnodes[minindex], node);
  node->curve = updatecurve;
  node->parent = nearnodes[minindex];
  node->cost = mincost;
  return node;
}

// TODO
bool DDplanner::Rewire(RRTNode* node, std::vector<int>& nearnum) {
  int node_count = nodelist_.size();
  if (nearnum.empty()) return true;
  for (int i = 0; i < nearnum.size(); i++) {
    RRTNode* nearnode = nodelist_[i];
    std::vector<ccpp::Pointf> newcurve =
        DubinsConfig(nodelist_[node_count - 1], nearnode);
    float newcost = DubinsLength(nodelist_[node_count - 1], nearnode) +
                    nodelist_[node_count - 1]->cost;
    if ((!CollisionDetection(newcurve)) && newcost < nearnode->cost) {
      nodelist_[i]->curve.clear();
      nodelist_[i]->curve = newcurve;
      nodelist_[i]->parent = nodelist_[node_count - 1];
      nodelist_[i]->cost = newcost;
    }
  }
  return true;
}

bool DDplanner::ReachGoal(RRTNode* node) {
  // TODO
  if (fabs(node->cur.x - goal_.x) < goal_dist_limit_ &&
      fabs(node->cur.y - goal_.y) < goal_dist_limit_ &&
      fabs(node->cur.yaw - goal_.yaw) < goal_yaw_limit_ - 0.2) {
    return true;
  }
  return false;
}

int DDplanner::GetBestIndex() {
  std::vector<int> dist_set;
  std::vector<int> yaw_set;
  float mindist = std::numeric_limits<float>::max();
  int n_index;
  for (int i = 0; i < nodelist_.size(); i++) {
    float dist = pow(nodelist_[i]->cur.x - goal_.x, 2) +
                 pow(nodelist_[i]->cur.y - goal_.y, 2);
    if (dist < goal_dist_limit_) {
      dist_set.push_back(i);
    }
  }
  for (int i = 0; i < dist_set.size(); i++) {
    if (fabs(nodelist_[dist_set[i]]->cur.yaw - goal_.yaw) < goal_yaw_limit_) {
      yaw_set.push_back(dist_set[i]);
    }
  }
  if (yaw_set.empty()) return std::numeric_limits<int>::max();
  for (int i = 0; i < yaw_set.size(); i++) {
    if (nodelist_[yaw_set[i]]->cost < mindist) {
      mindist = nodelist_[yaw_set[i]]->cost;
      n_index = yaw_set[i];
    }
  }
  return n_index;
}

std::vector<ccpp::Pointf> DDplanner::gen_final_course(int bestindex) {
  std::vector<ccpp::Pointf> path;
  RRTNode* temp = nodelist_[bestindex];
  path.push_back(goal_);
  while (temp->parent) {
    int count = temp->curve.size();
    for (int i = 0; i < count; i++) {
      path.push_back(temp->curve[count - i - 1]);
    }
    temp = temp->parent;
  }
  return path;
}

std::vector<ccpp::Pointf> DDplanner::RRTPlanning() {
  std::vector<ccpp::Pointf> path;
  Initialize();
  for (int i = 0; i < max_iter_; i++) {
    RRTNode* newnode = GetRandomNode();
    if (newnode == NULL) {
      continue;
    }
    RRTNode* neareatnode = GetNearestNode(newnode);
    std::vector<ccpp::Pointf> newcurve = DubinsConfig(neareatnode, newnode);
    newnode->curve = newcurve;
    newnode->parent = neareatnode;
    newnode->cost = DubinsLength(neareatnode, newnode) + neareatnode->cost;
    if (!CollisionDetection(newcurve)) {
      std::cout << "first curve" << std::endl;
      std::vector<RRTNode*> nearnodes;
      std::vector<int> nearnum = GetNearNodes(newnode, nearnodes);
      newnode = chooseparent(newnode, nearnodes);
      nodelist_.push_back(newnode);
      Rewire(newnode, nearnum);
      // if (ReachGoal(newnode)) {
      //   //TODO
      //   break;
      // }
    }
  }
  int bestindex = GetBestIndex();
  if (bestindex == std::numeric_limits<int>::max()) {
    return path;
  }
  path = gen_final_course(bestindex);

  return std::vector<ccpp::Pointf>();
}