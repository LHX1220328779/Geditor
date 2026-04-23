#include "pathplanner/blanket/coverageplanning.h"

#include "geometry/dubins.h"
#include "pathplanner/blanket/astarhybrid.h"
#include "pathplanner/blanket/ddplanner.h"

#if CV_VERSION_MAJOR > 3
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#define CV_RETR_CCOMP cv::RETR_CCOMP
#endif

using namespace std;
using namespace cv;
using namespace ccpp;

/**********************************************
 * Function: CoveragePlanning
 * Description: the construct function for CoveragePlanning
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
CoveragePlanning::CoveragePlanning() {}

/**********************************************
 * Function: CoveragePlanning
 * Description: the construct function for CoveragePlanning
 * Input: contourstracing----basic param of the contourstracing class
 *        init_p----the initial position when the car enter the cleaning region
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
CoveragePlanning::CoveragePlanning(const ContoursTracing& contourstracing,
                                   const ccpp::Pointf& start_point,
                                   const ccpp::Pointf& end_point,
                                   const int& index) {
  start_p_ = start_point;
  end_p_ = end_point;
  write_seg_ = index;
  binary_image_ = contourstracing.binary_image_.clone();
  single_contour_mat_ = binary_image_.clone();
  free_contours_ = contourstracing.free_contours_;
  connection_map_ = contourstracing.connection_map_;
  traversing_order_ = contourstracing.traversing_order_;
  obs_dect_ = new ObsDetection(binary_image_);
  rows_ = binary_image_.rows;
  cols_ = binary_image_.cols;
  imwrite("cover.jpg", binary_image_);
  // GetWholeContours();
  GetOuterContours();
  GetRegionBound();
  GetContoursMap();
}

/**********************************************
 * Function: ~CoveragePlanning
 * Description: the disconstruct function for CoveragePlanning
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
CoveragePlanning::~CoveragePlanning() {}

// bool CoveragePlanning::GetWholeContours() {
//   std::vector<std::vector<Point> > contours;
//   std::vector<Vec4i> hierarchy;
//   findContours(single_contour_mat_,
//                  contours,
//                  hierarchy,
//                  RETR_EXTERNAL,
//                  CV_CHAIN_APPROX_SIMPLE);
//   connection_map_ = contours[0];
// }

float CoveragePlanning::NormalizeAngle(float angle) {
  float a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  if (fabs(a) < 1e-3 || fabs(a - 2.0 * M_PI) < 1e-3) return -M_PI;
  if (fabs(a - M_PI) < 1e-3) return 0.0;
  return a - M_PI;
}

float CoveragePlanning::NormalizeDeg(float angle) {
  float a = std::fmod(angle + 180.0, 2.0 * 180.0);
  if (a < 0.0) {
    a += 360.0;
  }
  if (fabs(a) < 1e-3 || fabs(a - 360.0) < 1e-3) return -180.0;
  if (fabs(a - 180.0) < 1e-3) return 0.0;
  return a - 180.0;
}

/**********************************************
 * Function: GetThreeGoals
 * Description:
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<ccpp::Pointf> CoveragePlanning::GetThreeGoals(
    const ccpp::Pointf& p) {
  std::vector<ccpp::Pointf> goal;
  ccpp::Pointf p1, p2;
  p1.x = p.x + ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y - ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = NormalizeAngle(-M_PI);

  p2.x = p.x + ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = NormalizeAngle(-M_PI / 2.0);

  goal.push_back(p);
  goal.push_back(p1);
  goal.push_back(p2);
  return goal;
}

/**********************************************
 * Function: ClearContourMat
 * Description: for test, clear the pixel of single_contour_mat_.
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::ClearContourMat() {
  for (int i = 0; i < rows_; i++) {
    for (int j = 0; j < cols_; j++) {
      single_contour_mat_.at<uchar>(i, j) = 0;
    }
  }
  return true;
}

/**********************************************
 * Function: SetSingleFreespace
 * Description: for test, clear the pixel of single_contour_mat_.
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::SetSingleFreespace(const int& index) {
  ClearContourMat();
  for (int i = 0; i < free_contours_[index].size(); i++) {
    single_contour_mat_.at<uchar>(free_contours_[index][i]) = 255;
  }
  return true;
}

/**********************************************
 * Function: GetOuterContours
 * Description: get the outer contours of every subregion
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::GetOuterContours() {
  if (free_contours_.empty()) {
    return false;
  }
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::map<int, std::vector<cv::Point> >::iterator it;
  for (it = free_contours_.begin(); it != free_contours_.end(); it++) {
    SetSingleFreespace(it->first);
    contours.clear();
    hierarchy.clear();
    findContours(single_contour_mat_, contours, hierarchy, RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
      outer_contours_.insert(std::make_pair(it->first, contours[0]));
      // outer_contours_[it->first].push_back(contours[0]);
    }
  }
  return true;
}

/**********************************************
 * Function: ContoursExpanding
 * Description: for every sub region , expand the contours to keep the car in a
 *safe distance Input: None Output: None Return: None Others: None
 **********************************************/
bool CoveragePlanning::ContoursExpanding(const int& index,
                                         const int& safe_expand_dis) {
  ClearContourMat();
  for (int i = 0; i < free_contours_[index].size(); i++) {
    for (int x = free_contours_[index][i].x - ccpp::cvcarmodel::cv_car_width -
                 safe_expand_dis;
         x < free_contours_[index][i].x + ccpp::cvcarmodel::cv_car_width +
                 safe_expand_dis;
         x++) {
      for (int y = free_contours_[index][i].y - ccpp::cvcarmodel::cv_car_width -
                   safe_expand_dis;
           y < free_contours_[index][i].y + ccpp::cvcarmodel::cv_car_width +
                   safe_expand_dis;
           y++) {
        if (x < 0 || y < 0 || x >= cols_ || y >= rows_) {
          continue;
        }
        single_contour_mat_.at<uchar>(free_contours_[index][i]) = 255;
      }
    }
  }
  return true;
}

/**********************************************
 * Function: GetInnerContours
 * Description: get the inner contours of every subregion
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::GetInnerContours() {
  if (free_contours_.empty()) {
    return false;
  }
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::map<int, std::vector<cv::Point> >::iterator it;
  for (it = free_contours_.begin(); it != free_contours_.end(); it++) {
    ContoursExpanding(it->first, 3);
    contours.clear();
    hierarchy.clear();
    findContours(single_contour_mat_, contours, hierarchy, RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
      // inner_contours_[it->first].push_back(contours[0]);
      inner_contours_.insert(std::make_pair(it->first, contours[0]));
    }
  }
  return true;
}

/**********************************************
 * Function: GetLeftUpperPoint
 * Description:
 * Input: index----the index of the subregion
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::GetLeftUpperPoint(const int& index, int& x,
                                                 int& y) {
  ccpp::Pointi point;
  std::vector<int> cur_bound = regionbound_[index];
  if (x == 0 || y == 0) {
    std::cout << "init" << std::endl;
    int leftest = cur_bound[0] + ccpp::cvcarmodel::cv_car_width;
    int leftest_neigh = leftest + ccpp::cvcarmodel::cv_car_radius;
    std::vector<int>::iterator ymin_boundary_it =
        std::min_element(std::begin(free_contours_map_[index][leftest]),
                         std::end(free_contours_map_[index][leftest]));
    std::vector<int>::iterator ymin_neigh1_it = std::min_element(
        std::begin(free_contours_map_[index][leftest +
                                             ccpp::cvcarmodel::cv_car_radius]),
        std::end(free_contours_map_[index][leftest +
                                           ccpp::cvcarmodel::cv_car_radius]));
    std::vector<int>::iterator ymin_neigh2_it = std::min_element(
        std::begin(
            free_contours_map_[index]
                              [leftest + ccpp::cvcarmodel::cv_car_radius * 2]),
        std::end(
            free_contours_map_[index]
                              [leftest + ccpp::cvcarmodel::cv_car_radius * 2]));
    int ymin_bound = *ymin_boundary_it;
    int ymin_neigh1 = *ymin_neigh1_it;
    int ymin_neigh2 = *ymin_neigh2_it;
    int y_ = max(ymin_bound, max(ymin_neigh1, ymin_neigh2));
    y_ = y_ + ccpp::cvcarmodel::cv_car_radius + ccpp::cvcarmodel::cv_car_width;
    point.x = leftest;
    point.y = y_;
    point.yaw = 90.0 * M_PI / 180.0;
  } else {
    std::vector<int>::iterator ymax_bound_it =
        std::max_element(std::begin(free_contours_map_[index][x]),
                         std::end(free_contours_map_[index][x]));
    int ymax_bound = *ymax_bound_it;
    if (y > ymax_bound - ccpp::cvcarmodel::cv_car_radius -
                ccpp::cvcarmodel::cv_car_width) {
      x += 1;
      std::vector<int>::iterator ymin_boundary_it =
          std::min_element(std::begin(free_contours_map_[index][x]),
                           std::end(free_contours_map_[index][x]));
      std::vector<int>::iterator ymin_neigh1_it = std::min_element(
          std::begin(
              free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius]),
          std::end(
              free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius]));
      std::vector<int>::iterator ymin_neigh2_it = std::min_element(
          std::begin(
              free_contours_map_[index]
                                [x + ccpp::cvcarmodel::cv_car_radius * 2]),
          std::end(
              free_contours_map_[index]
                                [x + ccpp::cvcarmodel::cv_car_radius * 2]));
      int ymin_bound = *ymin_boundary_it;
      int ymin_neigh1 = *ymin_neigh1_it;
      int ymin_neigh2 = *ymin_neigh2_it;
      int y_ = max(ymin_bound, max(ymin_neigh1, ymin_neigh2));
      y_ =
          y_ + ccpp::cvcarmodel::cv_car_radius + ccpp::cvcarmodel::cv_car_width;
      point.x = x;
      point.y = y_;
      point.yaw = 90.0 * M_PI / 180.0;
    } else {
      y += 2;
      point.x = x;
      point.y = y;
      point.yaw = 90.0 * M_PI / 180.0;
    }
  }
  return point;
}

bool CoveragePlanning::GetDubinsPath(const ccpp::Pointf& start,
                                     const ccpp::Pointf& end,
                                     std::vector<ccpp::Pointf>& d_curve) {
  // std::vector<ccpp::Pointf> d_curve;
  double q0[3] = {start.x, start.y, start.yaw};
  double q1[3] = {end.x, end.y, end.yaw};
  geometry::DubinsPath dubins_path;
  // geometry::dubins_init(q0, q1, ccpp::cvcarmodel::cv_car_radius,
  //                           &dubins_path);
  geometry::dubins_shortest_path(&dubins_path, q0, q1,
                                 ccpp::cvcarmodel::cv_car_radius);
  double x = 0.0;
  double length = geometry::dubins_path_length(&dubins_path);
  while (x < length) {
    double q[3];
    geometry::dubins_path_sample(&dubins_path, x, q);
    d_curve.push_back(ccpp::Pointf(q[0], q[1], q[2]));
    // if(obs_dect_->CheckObsCollision(ccpp::Pointf(q[0], q[1], q[2]))) {
    //   return false;
    // }
    x += 2;
  }
  d_curve.push_back(end);
  return true;
  ;
}

bool CoveragePlanning::CircleFit(ccpp::Pointi& p_cent, float& angle,
                                 bool anticlockwise,
                                 std::vector<ccpp::Pointf>& path) {
  float step = 0.05;
  if (anticlockwise) {  // shunshizhen
    while (step < angle) {
      ccpp::Pointf p;
      p.x = p_cent.x + ccpp::cvcarmodel::cv_car_radius *
                           cos(NormalizeAngle(M_PI / 2 + p_cent.yaw - step));
      p.y = p_cent.y + ccpp::cvcarmodel::cv_car_radius *
                           sin(NormalizeAngle(M_PI / 2 + p_cent.yaw - step));
      p.yaw = NormalizeAngle(p_cent.yaw - step);
      path.push_back(p);
      step += 0.05;
    }
  } else {
    while (step < angle) {
      ccpp::Pointf p;
      p.x = p_cent.x + ccpp::cvcarmodel::cv_car_radius *
                           cos(NormalizeAngle(-M_PI / 2 + p_cent.yaw + step));
      p.y = p_cent.y + ccpp::cvcarmodel::cv_car_radius *
                           sin(NormalizeAngle(-M_PI / 2 + p_cent.yaw + step));
      p.yaw = NormalizeAngle(p_cent.yaw + step);
      path.push_back(p);
      step += 0.05;
    }
  }
  return true;
}
/**********************************************
 * Function: ToStart
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<ccpp::Pointf> CoveragePlanning::ToStart(const int& index,
                                                    const ccpp::Pointf& init_p,
                                                    ccpp::Pointi& start) {
  std::vector<ccpp::Pointf> path1;
  std::vector<ccpp::Pointf> path2;
  ccpp::Pointi p1_cent;
  float angle;
  bool start_flag = false;
  int start_x = 0;
  int start_y = 0;
  while (!start_flag) {
    ccpp::Pointi leftupper_point = GetLeftUpperPoint(index, start_x, start_y);
    ccpp::Pointf leftupper_pointf;
    leftupper_pointf.x = static_cast<float>(leftupper_point.x);
    leftupper_pointf.y = static_cast<float>(leftupper_point.y);
    leftupper_pointf.yaw = 90.0 * M_PI / 180.0;
    start = leftupper_point;
    std::vector<ccpp::Pointf> goals = GetThreeGoals(leftupper_pointf);
    std::cout << "planning" << std::endl;
    std::cout << init_p.x << "," << init_p.y << "," << init_p.yaw << std::endl;
    std::cout << leftupper_pointf.x << "," << leftupper_pointf.y << ","
              << leftupper_pointf.yaw << std::endl;
    for (int i = 0; i < goals.size(); i++) {
      AstarHybrid ahplanner(init_p, goals[2 - i], binary_image_.clone());
      path1.clear();
      path1 = ahplanner.AHPlanning();
      if (path1.empty()) {
        start_flag = false;
        continue;
      } else {
        start_flag = true;
        std::cout << "path_size:" << path1.size() << std::endl;
        p1_cent.x = goals[0].x + ccpp::cvcarmodel::cv_car_radius;
        p1_cent.y = goals[0].y;
        p1_cent.yaw = NormalizeAngle(goals[2 - i].yaw);
        angle = fabs(NormalizeAngle(goals[2 - i].yaw) -
                     NormalizeAngle(goals[0].yaw));
        angle = angle > M_PI ? 2 * M_PI - angle : angle;
        CircleFit(p1_cent, angle, true, path2);
        for (int j = 0; j < path2.size(); j++) {
          path1.push_back(path2[j]);
        }
        break;
      }
    }
    if (!start_flag) {
      start_x = leftupper_point.x;
      start_y = leftupper_point.y;
    }
  }
  return path1;
}

/**********************************************
 * Function: GetxlowCurvePoint
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::GetxlowCurvePoint(const ccpp::Pointi p,
                                                 const bool& inflag) {
  ccpp::Pointi cur;
  bool upper_flag;
  if (fabs(p.yaw - (M_PI / 2.0)) < 0.05) {
    upper_flag = false;
    if (inflag) {
      cur.yaw = p.yaw - (M_PI / 2.0);
      cur.x = p.x - ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y - ccpp::cvcarmodel::cv_car_radius;
    } else {
      cur.yaw = p.yaw + (M_PI / 2.0);
      cur.x = p.x - ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y + ccpp::cvcarmodel::cv_car_radius;
    }
  } else if (fabs(p.yaw + (M_PI / 2.0)) < 0.05) {
    upper_flag = true;
    if (inflag) {
      cur.yaw = p.yaw + (M_PI / 2.0);
      cur.x = p.x - ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y + ccpp::cvcarmodel::cv_car_radius;
    } else {
      cur.yaw = p.yaw - (M_PI / 2.0);
      cur.x = p.x - ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y - ccpp::cvcarmodel::cv_car_radius;
    }
  }
  float yaw = NormalizeAngle(cur.yaw);
  cur.yaw = yaw;
  return cur;
}

/**********************************************
 * Function: GetxhighCurvePoint
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::GetxhighCurvePoint(const ccpp::Pointi p,
                                                  const bool& inflag) {
  ccpp::Pointi cur;
  bool upper_flag;
  if (fabs(p.yaw - (M_PI / 2.0)) < 0.05) {
    upper_flag = false;
    if (inflag) {
      cur.yaw = p.yaw + (M_PI / 2.0);
      cur.x = p.x + ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y - ccpp::cvcarmodel::cv_car_radius;
    } else {
      cur.yaw = p.yaw - (M_PI / 2.0);
      cur.x = p.x + ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y + ccpp::cvcarmodel::cv_car_radius;
    }
  } else if (fabs(p.yaw + (M_PI / 2.0)) < 0.05) {
    upper_flag = true;
    if (inflag) {
      cur.yaw = p.yaw - (M_PI / 2.0);
      cur.x = p.x + ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y + ccpp::cvcarmodel::cv_car_radius;
    } else {
      cur.yaw = p.yaw + (M_PI / 2.0);
      cur.x = p.x + ccpp::cvcarmodel::cv_car_radius;
      cur.y = p.y - ccpp::cvcarmodel::cv_car_radius;
    }
  }
  float yaw = NormalizeAngle(cur.yaw);
  cur.yaw = yaw;
  return cur;
}

bool CoveragePlanning::GetDownLeftGoals(ccpp::Pointi& p,
                                        std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi p1, p2;
  p1.x = p.x + ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y + ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = 0.0;
  p2.x = p.x + ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = -90.0 * M_PI / 180.0;
  neigh.push_back(p1);
  neigh.push_back(p2);
  neigh.push_back(p);
  return (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p1) ||
          obs_dect_->CheckObsCollision(p2));
}

/**********************************************
 * Function: Searchdownleftlimit
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::Searchdownleftlimit(
    const int& x, const int& index, std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi point;
  bool flag = true;
  std::vector<int>::iterator ymin_it =
      std::min_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymax_it =
      std::max_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymax_neigh1_it = std::max_element(
      std::begin(
          free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius]),
      std::end(free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius]));
  std::vector<int>::iterator ymax_neigh2_it = std::max_element(
      std::begin(
          free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius * 2]),
      std::end(
          free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius * 2]));
  int ymax = *ymax_it;
  int ymax_neigh1 = *ymax_neigh1_it;
  int ymax_neigh2 = *ymax_neigh2_it;
  int y = min(ymax, min(ymax_neigh1, ymax_neigh2));
  y = y - ccpp::cvcarmodel::cv_car_radius - ccpp::cvcarmodel::cv_car_width;
  // int y = ymax - ccpp::cvcarmodel::cv_car_radius -
  // ccpp::cvcarmodel::cv_car_width;

  while (flag) {
    point.x = x;
    point.y = y;
    point.yaw = 90.0 * M_PI / 180.0;
    std::cout << "downleft:" << point.x << "," << point.y << std::endl;
    neigh.clear();
    flag = GetDownLeftGoals(point, neigh);
    if (!flag) {
      if (y - ccpp::cvcarmodel::cv_car_radius - ccpp::cvcarmodel::cv_car_width <
          *ymin_it) {
        return ccpp::Pointi(0, 0, 0);
      }
      return point;
    }
    y--;
  }
  return ccpp::Pointi(0, 0, 0);
}

bool CoveragePlanning::GetDownRightGoals(ccpp::Pointi& p,
                                         std::vector<ccpp::Pointi>& neigh) {
  std::vector<ccpp::Pointi> p_set;
  ccpp::Pointi p1, p2;
  p1.x = p.x - ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y + ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = 0.0;
  p2.x = p.x - ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = 90.0 * M_PI / 180.0;
  neigh.push_back(p1);
  neigh.push_back(p2);
  neigh.push_back(p);
  return (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p1) ||
          obs_dect_->CheckObsCollision(p2));
}

/**********************************************
 * Function: Searchdownrightlimit
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::Searchdownrightlimit(
    const int& x, const int& index, std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi point;
  bool flag = true;
  // TODO
  std::vector<int>::iterator ymin_it =
      std::min_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymax_it =
      std::max_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymax_neigh1_it = std::max_element(
      std::begin(
          free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius]),
      std::end(free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius]));
  std::vector<int>::iterator ymax_neigh2_it = std::max_element(
      std::begin(
          free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius * 2]),
      std::end(
          free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius * 2]));
  int ymax = *ymax_it;
  int ymax_neigh1 = *ymax_neigh1_it;
  int ymax_neigh2 = *ymax_neigh2_it;
  int y = min(ymax, min(ymax_neigh1, ymax_neigh2));
  y = y - ccpp::cvcarmodel::cv_car_radius - ccpp::cvcarmodel::cv_car_width;
  // int y = ymax - ccpp::cvcarmodel::cv_car_radius -
  // ccpp::cvcarmodel::cv_car_width;
  while (flag) {
    point.x = x;
    point.y = y;
    point.yaw = -90.0 * M_PI / 180.0;
    std::cout << "downright:" << point.x << "," << point.y << std::endl;
    neigh.clear();
    flag = GetDownRightGoals(point, neigh);
    if (!flag) {
      if (y - ccpp::cvcarmodel::cv_car_radius - ccpp::cvcarmodel::cv_car_width <
          *ymin_it) {
        return ccpp::Pointi(0, 0, 0);
      }
      return point;
    }
    y--;
  }
  return ccpp::Pointi(0, 0, 0);
}

ccpp::Pointi CoveragePlanning::Searchdownright(const int& xinit,
                                               const int& index,
                                               std::vector<ccpp::Pointi>& neigh,
                                               const int& xmax) {
  int x = xinit;
  while (x < xmax - ccpp::cvcarmodel::cv_car_width) {
    ccpp::Pointi p = Searchdownrightlimit(x, index, neigh);
    if (p.x == x && p.y == 0) {
      x++;
    } else {
      return p;
    }
  }
  return ccpp::Pointi(x, 0, 0);
}

bool CoveragePlanning::GetUpRightGoals(ccpp::Pointi& p,
                                       std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi p1, p2;
  p1.x = p.x - ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y - ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = -M_PI;
  p2.x = p.x - ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = 90.0 * M_PI / 180.0;
  neigh.push_back(p1);
  neigh.push_back(p2);
  neigh.push_back(p);
  return (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p1) ||
          obs_dect_->CheckObsCollision(p2));
}
/**********************************************
 * Function: Searchuprightlimit
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::Searchuprightlimit(
    const int& x, const int& index, std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi point;
  bool flag = true;
  // TODO
  std::vector<int>::iterator ymax_it =
      std::max_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymin_it =
      std::min_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymin_neigh1_it = std::min_element(
      std::begin(
          free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius]),
      std::end(free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius]));
  std::vector<int>::iterator ymin_neigh2_it = std::min_element(
      std::begin(
          free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius * 2]),
      std::end(
          free_contours_map_[index][x - ccpp::cvcarmodel::cv_car_radius * 2]));
  int ymin = *ymin_it;
  int ymin_neigh1 = *ymin_neigh1_it;
  int ymin_neigh2 = *ymin_neigh2_it;
  int y = max(ymin, max(ymin_neigh1, ymin_neigh2));
  y = y + ccpp::cvcarmodel::cv_car_radius + ccpp::cvcarmodel::cv_car_width;
  // int y = ymin + ccpp::cvcarmodel::cv_car_radius +
  // ccpp::cvcarmodel::cv_car_width;
  while (flag) {
    point.x = x;
    point.y = y;
    point.yaw = -90.0 * M_PI / 180.0;
    std::cout << "upright:" << point.x << "," << point.y << std::endl;
    neigh.clear();
    flag = GetUpRightGoals(point, neigh);
    if (!flag) {
      if (y + ccpp::cvcarmodel::cv_car_radius + ccpp::cvcarmodel::cv_car_width >
          *ymax_it) {
        return ccpp::Pointi(0, 0, 0);
      }
      return point;
    }
    y++;
  }
  return ccpp::Pointi(0, 0, 0);
}

bool CoveragePlanning::GetUpLeftGoals(ccpp::Pointi& p,
                                      std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi p1, p2;
  p1.x = p.x + ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y - ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = -M_PI;
  p2.x = p.x + ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = -90.0 * M_PI / 180.0;
  neigh.push_back(p1);
  neigh.push_back(p2);
  neigh.push_back(p);
  return (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p1) ||
          obs_dect_->CheckObsCollision(p2));
}
/**********************************************
 * Function: searchupleftlimit
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ccpp::Pointi CoveragePlanning::Searchupleftlimit(
    const int& x, const int& index, std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi point;
  bool flag = true;
  // TODO
  std::vector<int>::iterator ymax_it =
      std::max_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymin_it =
      std::min_element(std::begin(free_contours_map_[index][x]),
                       std::end(free_contours_map_[index][x]));
  std::vector<int>::iterator ymin_neigh1_it = std::min_element(
      std::begin(
          free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius]),
      std::end(free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius]));
  std::vector<int>::iterator ymin_neigh2_it = std::min_element(
      std::begin(
          free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius * 2]),
      std::end(
          free_contours_map_[index][x + ccpp::cvcarmodel::cv_car_radius * 2]));
  int ymin = *ymin_it;
  int ymin_neigh1 = *ymin_neigh1_it;
  int ymin_neigh2 = *ymin_neigh2_it;
  int y = max(ymin, max(ymin_neigh1, ymin_neigh2));
  y = y + ccpp::cvcarmodel::cv_car_radius + ccpp::cvcarmodel::cv_car_width;
  // int y = ymin + ccpp::cvcarmodel::cv_car_radius +
  // ccpp::cvcarmodel::cv_car_width;
  while (flag) {
    point.x = x;
    point.y = y;
    point.yaw = 90.0 * M_PI / 180.0;
    std::cout << "upleft:" << point.x << "," << point.y << std::endl;
    neigh.clear();
    flag = GetUpLeftGoals(point, neigh);
    if (!flag) {
      if (y + ccpp::cvcarmodel::cv_car_radius + ccpp::cvcarmodel::cv_car_width >
          *ymax_it) {
        return ccpp::Pointi(0, 0, 0);
      }
      return point;
    }
    y++;
  }
  return ccpp::Pointi(0, 0, 0);
}

/**********************************************
 * Function: GetCarBoundary
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<int> CoveragePlanning::GetCarBoundary(const ccpp::Pointi& p,
                                                  const bool& upper_flag) {
  std::vector<int> car_bound;
  int x_min;
  int x_max;
  int y_min;
  int y_max;
  x_min = p.x - ccpp::cvcarmodel::cv_half_car_width;
  x_max = p.x + ccpp::cvcarmodel::cv_half_car_width;
  if (upper_flag) {
    y_min = p.x - ccpp::cvcarmodel::cv_front2base;
    y_max = p.x + ccpp::cvcarmodel::cv_tail2base;
  } else {
    y_min = p.y - ccpp::cvcarmodel::cv_tail2base;
    y_max = p.y + ccpp::cvcarmodel::cv_front2base;
  }
  car_bound.push_back(x_min);
  car_bound.push_back(x_max);
  car_bound.push_back(y_min);
  car_bound.push_back(y_max);
  return car_bound;
}

/**********************************************
 * Function: GetSubregionBound
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<int> CoveragePlanning::GetSubregionBound(const int& index) {
  std::vector<int> subregionbound;
  int x_min = std::numeric_limits<int>::max();
  int x_max = std::numeric_limits<int>::min();
  int y_min = std::numeric_limits<int>::max();
  int y_max = std::numeric_limits<int>::min();
  for (int i = 0; i < outer_contours_[index].size(); i++) {
    if (outer_contours_[index][i].x < x_min) {
      x_min = outer_contours_[index][i].x;
    }
    if (outer_contours_[index][i].x > x_max) {
      x_max = outer_contours_[index][i].x;
    }
    if (outer_contours_[index][i].y < y_min) {
      y_min = outer_contours_[index][i].y;
    }
    if (outer_contours_[index][i].y > y_max) {
      y_max = outer_contours_[index][i].y;
    }
  }
  subregionbound.push_back(x_min);
  subregionbound.push_back(x_max);
  subregionbound.push_back(y_min);
  subregionbound.push_back(y_max);
std:;
  cout << "bound-" << index << ":" << x_min << "," << x_max << "," << y_min
       << "," << y_max << std::endl;
  return subregionbound;
}

/**********************************************
 * Function: GetSubregionBound
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::GetRegionBound() {
  for (int i = 0; i < free_contours_.size(); i++) {
    regionbound_.insert(std::make_pair(i, GetSubregionBound(i)));
  }
  return true;
}

/**********************************************
 * Function: GetContoursMap
 * Description:
 * Input:
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::GetContoursMap() {
  for (int i = 0; i < free_contours_.size(); i++) {
    std::map<int, std::vector<int> > singlemap;
    for (int j = 0; j < free_contours_[i].size(); j++) {
      singlemap[free_contours_[i][j].x].push_back(free_contours_[i][j].y);
    }
    free_contours_map_.insert(std::make_pair(i, singlemap));
  }
  return true;
}

bool CoveragePlanning::GetXline(std::vector<ccpp::Pointi>& p1,
                                std::vector<ccpp::Pointi>& p2,
                                std::vector<ccpp::Pointf>& path) {
  std::vector<ccpp::Pointf> path1;
  ccpp::Pointi p1_cent;
  ccpp::Pointi p2_cent;
  float angle;
  std::cout << "GetXline:" << p1.size() << "," << p2.size() << std::endl;
  for (int i = 0; i < p1.size(); i++) {
    for (int j = 0; j < p2.size(); j++) {
      AstarHybrid ahplanner1(p1[i], p2[j], binary_image_.clone());
      path1.clear();
      path1 = ahplanner1.AHPlanning();
      if (path1.size() <= 2) {
        std::cout << "error" << std::endl;
        continue;
      }
      if (fabs(p1[2].yaw - 90 * M_PI / 180.0) < 1e-2) {
        p1_cent.x = p1[2].x + ccpp::cvcarmodel::cv_car_radius;
      } else {
        p1_cent.x = p1[2].x - ccpp::cvcarmodel::cv_car_radius;
      }
      p1_cent.y = p1[2].y;
      p1_cent.yaw = NormalizeAngle(p1[2].yaw);
      angle = fabs(NormalizeAngle(p1[i].yaw) - NormalizeAngle(p1[2].yaw));
      angle = angle > M_PI ? 2 * M_PI - angle : angle;
      CircleFit(p1_cent, angle, true, path);
      // GetDubinsPath(p1[2], p1[i], path);
      for (int k = 0; k < path1.size(); k++) {
        path.push_back(path1[k]);
      }
      if (fabs(p2[2].yaw - 90 * M_PI / 180.0) < 1e-2) {
        p2_cent.x = p2[2].x + ccpp::cvcarmodel::cv_car_radius;
      } else {
        p2_cent.x = p2[2].x - ccpp::cvcarmodel::cv_car_radius;
      }
      p2_cent.y = p2[2].y;
      p2_cent.yaw = NormalizeAngle(p2[j].yaw);
      angle = fabs(NormalizeAngle(p2[j].yaw) - NormalizeAngle(p2[2].yaw));
      angle = angle > M_PI ? 2 * M_PI - angle : angle;
      CircleFit(p2_cent, angle, true, path);
      // GetDubinsPath(p2[i], p2[2], path);
      std::cout << "ok" << std::endl;
      return true;
    }
  }
  return false;
}

bool CoveragePlanning::GetYline(ccpp::Pointi p1, ccpp::Pointi p2, bool downflag,
                                std::vector<ccpp::Pointf>& path) {
  float length = 0;
  if (downflag) {
    while (length < p2.y - p1.y) {
      path.push_back(ccpp::Pointi(p1.x, p1.y + length, p1.yaw));
      length += 2;
    }
    path.push_back(p2);
  } else {
    while (length < p1.y - p2.y) {
      path.push_back(ccpp::Pointi(p1.x, p1.y - length, p1.yaw));
      length += 2;
    }
    path.push_back(p2);
  }
  return true;
}

bool CoveragePlanning::Searchdownlimit(const int& x, int& new_x,
                                       const int& index, const int& xmax,
                                       std::vector<ccpp::Pointi>& ldpoints,
                                       std::vector<ccpp::Pointi>& lupoints) {
  int cur = x;
  int ymin, ymin1, ymin2, ymax, ymax1, ymax2;
  std::vector<int>::iterator ymin_it, ymin1_it, ymin2_it, ymax_it, ymax1_it,
      ymax2_it;
  int ylittle, ybig;
  bool downflag = true;
  bool upflag = true;
  while (cur < xmax - ccpp::cvcarmodel::cv_car_width) {
    ccpp::Pointi left_down_point, left_up_point;
    ymin_it = std::min_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymax_it = std::max_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymin1_it = std::min_element(
        std::begin(
            free_contours_map_[index][cur + ccpp::cvcarmodel::cv_car_radius]),
        std::end(
            free_contours_map_[index][cur + ccpp::cvcarmodel::cv_car_radius]));
    ymax1_it = std::max_element(
        std::begin(
            free_contours_map_[index][cur + ccpp::cvcarmodel::cv_car_radius]),
        std::end(
            free_contours_map_[index][cur + ccpp::cvcarmodel::cv_car_radius]));
    ymin2_it = std::min_element(
        std::begin(
            free_contours_map_[index]
                              [cur + ccpp::cvcarmodel::cv_car_radius * 2]),
        std::end(
            free_contours_map_[index]
                              [cur + ccpp::cvcarmodel::cv_car_radius * 2]));
    ymax2_it = std::max_element(
        std::begin(
            free_contours_map_[index]
                              [cur + ccpp::cvcarmodel::cv_car_radius * 2]),
        std::end(
            free_contours_map_[index]
                              [cur + ccpp::cvcarmodel::cv_car_radius * 2]));
    ymin = *ymin_it;
    ymin1 = *ymin1_it;
    ymin2 = *ymin2_it;
    ymax = *ymax_it;
    ymax1 = *ymax1_it;
    ymax2 = *ymax2_it;
    ylittle = max(ymin, max(ymin1, ymin2)) + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_car_width;
    ybig = min(ymax, max(ymax1, ymax2)) - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_width;

    while (downflag || upflag) {
      if (ybig < ylittle) {
        std::cout << "bad x" << std::endl;
        cur++;
        break;
      }
      left_down_point.x = cur;
      left_down_point.y = ybig;
      left_down_point.yaw = 90.0 * M_PI / 180.0;
      left_up_point.x = cur;
      left_up_point.y = ylittle;
      left_up_point.yaw = 90.0 * M_PI / 180.0;
      ldpoints.clear();
      lupoints.clear();
      downflag = GetDownLeftGoals(left_down_point, ldpoints);
      upflag = GetUpLeftGoals(left_up_point, lupoints);
      if (downflag) {
        ybig--;
      }
      if (upflag) {
        ylittle++;
      }
      if ((!downflag) && (!upflag)) {
        new_x = cur;
        return true;
      }
    }
  }
  return false;
}

bool CoveragePlanning::Searchuplimit(const int& x, int& new_x, const int& index,
                                     const int& xmax,
                                     std::vector<ccpp::Pointi>& rdpoints,
                                     std::vector<ccpp::Pointi>& rupoints) {
  int cur = x;
  int ymin, ymin1, ymin2, ymax, ymax1, ymax2;
  int ylittle, ybig;
  bool downflag = true;
  bool upflag = true;
  while (cur < xmax) {
    ccpp::Pointi right_down_point, right_up_point;
    std::vector<int>::iterator ymin_it =
        std::min_element(std::begin(free_contours_map_[index][cur]),
                         std::end(free_contours_map_[index][cur]));
    std::vector<int>::iterator ymin1_it = std::min_element(
        std::begin(
            free_contours_map_[index][cur - ccpp::cvcarmodel::cv_car_radius]),
        std::end(
            free_contours_map_[index][cur - ccpp::cvcarmodel::cv_car_radius]));
    std::vector<int>::iterator ymin2_it = std::min_element(
        std::begin(
            free_contours_map_[index]
                              [cur - ccpp::cvcarmodel::cv_car_radius * 2]),
        std::end(
            free_contours_map_[index]
                              [cur - ccpp::cvcarmodel::cv_car_radius * 2]));
    std::vector<int>::iterator ymax_it =
        std::max_element(std::begin(free_contours_map_[index][cur]),
                         std::end(free_contours_map_[index][cur]));
    std::vector<int>::iterator ymax1_it = std::max_element(
        std::begin(
            free_contours_map_[index][cur - ccpp::cvcarmodel::cv_car_radius]),
        std::end(
            free_contours_map_[index][cur - ccpp::cvcarmodel::cv_car_radius]));
    std::vector<int>::iterator ymax2_it = std::max_element(
        std::begin(
            free_contours_map_[index]
                              [cur - ccpp::cvcarmodel::cv_car_radius * 2]),
        std::end(
            free_contours_map_[index]
                              [cur - ccpp::cvcarmodel::cv_car_radius * 2]));
    ymin = *ymin_it;
    ymin1 = *ymin1_it;
    ymin2 = *ymin2_it;
    ymax = *ymax_it;
    ymax1 = *ymax2_it;
    ymax2 = *ymax2_it;
    ylittle = max(ymin, max(ymin1, ymin2)) + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_car_width;
    ybig = min(ymax, max(ymax1, ymax2)) - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_width;

    while (downflag || upflag) {
      if (ybig < ylittle) {
        std::cout << "bad x" << std::endl;
        cur++;
        break;
      }
      right_down_point.x = cur;
      right_down_point.y = ybig;
      right_down_point.yaw = -90.0 * M_PI / 180.0;
      right_up_point.x = cur;
      right_up_point.y = ylittle;
      right_up_point.yaw = -90.0 * M_PI / 180.0;
      rdpoints.clear();
      rupoints.clear();
      downflag = GetDownRightGoals(right_down_point, rdpoints);
      upflag = GetUpRightGoals(right_up_point, rupoints);
      if (downflag) {
        ybig--;
      }
      if (upflag) {
        ylittle++;
      }
      if ((!downflag) && (!upflag)) {
        new_x = cur;
        return true;
      }
    }
  }
  return false;
}

std::vector<ccpp::Pointf> CoveragePlanning::Searchstartlimit(
    const int& index, const ccpp::Pointf& init_p, const int& xmin,
    const int& xmax, int& new_x) {
  std::vector<ccpp::Pointi> ldpoints;
  std::vector<ccpp::Pointi> lupoints;
  std::vector<ccpp::Pointf> path1;
  std::vector<ccpp::Pointf> path2;
  std::vector<ccpp::Pointi> goals;
  std::vector<int>::iterator ymin_it, ymin1_it, ymin2_it, ymax_it, ymax1_it,
      ymax2_it;
  int cur = xmin;
  int ymin, ymin1, ymin2, ymax, ymax1, ymax2;
  int ylittle, ybig;
  bool downflag = true;
  bool upflag = true;
  std::cout << "start search the start point in region " << index << std::endl;
  std::cout << "index:" << index << ",cur" << cur << ",xmin" << xmin << ",xmax"
            << xmax << "carwidth" << ccpp::cvcarmodel::cv_car_width
            << std::endl;
  while (cur < xmax - ccpp::cvcarmodel::cv_car_width) {
    std::cout << "start search1" << std::endl;

    ccpp::Pointi left_down_point, left_up_point;
    ymin_it = std::min_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymin = *ymin_it;
    ymax_it = std::max_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymax = *ymax_it;
    if (cur + ccpp::cvcarmodel::cv_car_radius < xmax) {
      ymin1_it = std::min_element(
          std::begin(
              free_contours_map_[index][cur + ccpp::cvcarmodel::cv_car_radius]),
          std::end(free_contours_map_[index]
                                     [cur + ccpp::cvcarmodel::cv_car_radius]));
      ymax1_it = std::max_element(
          std::begin(
              free_contours_map_[index][cur + ccpp::cvcarmodel::cv_car_radius]),
          std::end(free_contours_map_[index]
                                     [cur + ccpp::cvcarmodel::cv_car_radius]));
      ymin1 = *ymin1_it;
      ymax1 = *ymax1_it;
    } else {
      ymin1 = ymin;
      ymax1 = ymax;
    }
    if (cur + ccpp::cvcarmodel::cv_car_radius * 2 < xmax) {
      ymin2_it = std::min_element(
          std::begin(
              free_contours_map_[index]
                                [cur + ccpp::cvcarmodel::cv_car_radius * 2]),
          std::end(
              free_contours_map_[index]
                                [cur + ccpp::cvcarmodel::cv_car_radius * 2]));
      ymax2_it = std::max_element(
          std::begin(
              free_contours_map_[index]
                                [cur + ccpp::cvcarmodel::cv_car_radius * 2]),
          std::end(
              free_contours_map_[index]
                                [cur + ccpp::cvcarmodel::cv_car_radius * 2]));
      ymin2 = *ymin2_it;
      ymax2 = *ymax2_it;
    } else {
      ymin2 = ymin;
      ymax2 = ymax;
    }
    ylittle = max(ymin, max(ymin1, ymin2)) + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_car_width;
    ybig = min(ymax, max(ymax1, ymax2)) - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_width;

    std::cout << "start search2" << std::endl;
    while (downflag || upflag) {
      if (ybig < ylittle) {
        std::cout << "ybig:" << ybig << ",ylittle" << ylittle << std::endl;
        std::cout << "current x is:" << cur << std::endl;
        std::cout << "bad x" << std::endl;
        cur++;
        break;
      }
      // std::cout << "normal test cur:" << cur << std::endl;
      left_down_point.x = cur;
      left_down_point.y = ybig;
      left_down_point.yaw = 90.0 * M_PI / 180.0;
      left_up_point.x = cur;
      left_up_point.y = ylittle;
      left_up_point.yaw = 90.0 * M_PI / 180.0;
      ldpoints.clear();
      lupoints.clear();
      downflag = GetDownLeftGoals(left_down_point, ldpoints);
      upflag = GetUpLeftGoals(left_up_point, lupoints);
      if (downflag) {
        ybig--;
      }
      if (upflag) {
        ylittle++;
      }
      if ((!downflag) && (!upflag)) {
        goals.clear();
        goals.push_back(lupoints[1]);
        goals.push_back(lupoints[0]);
        goals.push_back(lupoints[2]);
        for (int i = 0; i < goals.size(); i++) {
          AstarHybrid ahplanner(init_p, goals[i], binary_image_.clone());
          path1.clear();
          path1 = ahplanner.AHPlanning();
          if (!path1.empty()) {
            std::cout << "path_size:" << path1.size() << std::endl;
            ccpp::Pointi p1_cent;
            p1_cent.x = goals[2].x + ccpp::cvcarmodel::cv_car_radius;
            p1_cent.y = goals[2].y;
            p1_cent.yaw = NormalizeAngle(goals[i].yaw);
            float angle = fabs(NormalizeAngle(goals[i].yaw) -
                               NormalizeAngle(goals[2].yaw));
            angle = angle > M_PI ? 2 * M_PI - angle : angle;
            CircleFit(p1_cent, angle, true, path2);
            for (int j = 0; j < path2.size(); j++) {
              path1.push_back(path2[j]);
            }
            new_x = cur;
            return path1;
          }
        }
        cur++;
        break;
      }
    }
  }
  new_x = 0;
  return path1;
}

bool CoveragePlanning::Searchendlimit(const int& x, int& new_x,
                                      const int& index, const int& xmin) {
  std::vector<ccpp::Pointi> rdpoints;
  std::vector<ccpp::Pointi> rupoints;
  int cur = x;
  int ymin, ymin1, ymin2, ymax, ymax1, ymax2;
  std::vector<int>::iterator ymin_it, ymin1_it, ymin2_it, ymax_it, ymax1_it,
      ymax2_it;
  int ylittle, ybig;
  bool downflag = true;
  bool upflag = true;
  std::cout << "search the end index of region " << index << std::endl;
  while (cur > xmin) {
    ccpp::Pointi right_down_point, right_up_point;
    ymin_it = std::min_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymin = *ymin_it;
    ymax_it = std::max_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymax = *ymax_it;
    if (cur - ccpp::cvcarmodel::cv_car_radius > xmin) {
      ymin1_it = std::min_element(
          std::begin(
              free_contours_map_[index][cur - ccpp::cvcarmodel::cv_car_radius]),
          std::end(free_contours_map_[index]
                                     [cur - ccpp::cvcarmodel::cv_car_radius]));
      ymax1_it = std::max_element(
          std::begin(
              free_contours_map_[index][cur - ccpp::cvcarmodel::cv_car_radius]),
          std::end(free_contours_map_[index]
                                     [cur - ccpp::cvcarmodel::cv_car_radius]));
      ymin1 = *ymin1_it;
      ymax1 = *ymax1_it;
    } else {
      ymin1 = ymin;
      ymax1 = ymax;
    }
    if (cur - ccpp::cvcarmodel::cv_car_radius * 2 > xmin) {
      ymin2_it = std::min_element(
          std::begin(
              free_contours_map_[index]
                                [cur - ccpp::cvcarmodel::cv_car_radius * 2]),
          std::end(
              free_contours_map_[index]
                                [cur - ccpp::cvcarmodel::cv_car_radius * 2]));
      ymax2_it = std::max_element(
          std::begin(
              free_contours_map_[index]
                                [cur - ccpp::cvcarmodel::cv_car_radius * 2]),
          std::end(
              free_contours_map_[index]
                                [cur - ccpp::cvcarmodel::cv_car_radius * 2]));
      ymin2 = *ymin2_it;
      ymax2 = *ymax2_it;
    } else {
      ymin2 = ymin;
      ymax2 = ymax;
    }
    ylittle = max(ymin, max(ymin1, ymin2)) + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_car_width;
    ybig = min(ymax, max(ymax1, ymax2)) - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_width;

    while (downflag || upflag) {
      if (ybig < ylittle) {
        std::cout << "bad x" << std::endl;
        cur--;
        break;
      }
      right_down_point.x = cur;
      right_down_point.y = ybig;
      right_down_point.yaw = -90.0 * M_PI / 180.0;
      right_up_point.x = cur;
      right_up_point.y = ylittle;
      right_up_point.yaw = -90.0 * M_PI / 180.0;
      rdpoints.clear();
      rupoints.clear();
      downflag = GetDownRightGoals(right_down_point, rdpoints);
      upflag = GetUpRightGoals(right_up_point, rupoints);
      if (downflag) {
        ybig--;
      }
      if (upflag) {
        ylittle++;
      }
      if ((!downflag) && (!upflag)) {
        new_x = cur;
        return true;
      }
    }
  }
  return false;
}

int CoveragePlanning::GetDownGoals(const ccpp::Pointi& p,
                                   std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi p1, p2, p3, p4;
  p1.x = p.x + ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y + ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = 0;
  p2.x = p.x + ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = -90.0 * M_PI / 180.0;
  p3.x = p.x - ccpp::cvcarmodel::cv_car_radius;
  p3.y = p.y + ccpp::cvcarmodel::cv_car_radius;
  p3.yaw = -M_PI;
  p4.x = p.x - ccpp::cvcarmodel::cv_car_radius * 2;
  p4.y = p.y;
  p4.yaw = -90.0 * M_PI / 180.0;
  ccpp::Pointi p1_c, p2_c, p3_c, p4_c;
  p1_c.x = p.x + ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p1_c.y = p.y + ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p1_c.yaw = 45.0 * M_PI / 180.0;
  p2_c.x = p.x + ccpp::cvcarmodel::cv_car_radius +
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p2_c.y = p.y + ccpp::cvcarmodel::cv_car_radius;
  p2_c.yaw = -45.0 * M_PI / 180.0;
  p3_c.x = p.x - ccpp::cvcarmodel::cv_car_radius +
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p3_c.y = p.y + ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p3_c.yaw = 135.0 * M_PI / 180.0;
  p4_c.x = p.x - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p4_c.y = p.y + ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p4_c.yaw = -135.0 * M_PI / 180.0;
  bool rflag =
      (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p1) ||
       obs_dect_->CheckObsCollision(p2) || obs_dect_->CheckObsCollision(p1_c) ||
       obs_dect_->CheckObsCollision(p2_c));
  bool lflag =
      (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p3) ||
       obs_dect_->CheckObsCollision(p4) || obs_dect_->CheckObsCollision(p3_c) ||
       obs_dect_->CheckObsCollision(p4_c));
  neigh.clear();
  if ((!rflag) && (!lflag)) {
    neigh.push_back(p2);
    neigh.push_back(p2_c);
    neigh.push_back(p4);
    neigh.push_back(p4_c);
    neigh.push_back(p1);
    neigh.push_back(p1_c);
    neigh.push_back(p3);
    neigh.push_back(p3_c);
    neigh.push_back(p);
    return 3;
  } else if ((!rflag) && lflag) {
    neigh.push_back(p2);
    neigh.push_back(p2_c);
    neigh.push_back(p1);
    neigh.push_back(p1_c);
    neigh.push_back(p);
    return 1;
  } else if (rflag && (!lflag)) {
    neigh.push_back(p4);
    neigh.push_back(p4_c);
    neigh.push_back(p3);
    neigh.push_back(p3_c);
    neigh.push_back(p);
    return 2;
  } else {
    return 0;
  }
}

int CoveragePlanning::GetUpGoals(const ccpp::Pointi& p,
                                 std::vector<ccpp::Pointi>& neigh) {
  ccpp::Pointi p1, p2, p3, p4;
  p1.x = p.x + ccpp::cvcarmodel::cv_car_radius;
  p1.y = p.y - ccpp::cvcarmodel::cv_car_radius;
  p1.yaw = -M_PI;
  p2.x = p.x + ccpp::cvcarmodel::cv_car_radius * 2;
  p2.y = p.y;
  p2.yaw = -90.0 * M_PI / 180.0;
  p3.x = p.x - ccpp::cvcarmodel::cv_car_radius;
  p3.y = p.y - ccpp::cvcarmodel::cv_car_radius;
  p3.yaw = 0;
  p4.x = p.x - ccpp::cvcarmodel::cv_car_radius * 2;
  p4.y = p.y;
  p4.yaw = -90.0 * M_PI / 180.0;
  ccpp::Pointi p1_c, p2_c, p3_c, p4_c;
  p1_c.x = p.x + ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p1_c.y = p.y - ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p1_c.yaw = 135.0 * M_PI / 180.0;
  p2_c.x = p.x + ccpp::cvcarmodel::cv_car_radius +
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p2_c.y = p.y - ccpp::cvcarmodel::cv_car_radius;
  p2_c.yaw = -135.0 * M_PI / 180.0;
  p3_c.x = p.x - ccpp::cvcarmodel::cv_car_radius +
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p3_c.y = p.y - ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p3_c.yaw = 45.0 * M_PI / 180.0;
  p4_c.x = p.x - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p4_c.y = p.y - ccpp::cvcarmodel::cv_car_radius * C_HALF_RAD;
  p4_c.yaw = -45.0 * M_PI / 180.0;
  bool rflag =
      (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p1) ||
       obs_dect_->CheckObsCollision(p2) || obs_dect_->CheckObsCollision(p1_c) ||
       obs_dect_->CheckObsCollision(p2_c));
  bool lflag =
      (obs_dect_->CheckObsCollision(p) || obs_dect_->CheckObsCollision(p3) ||
       obs_dect_->CheckObsCollision(p4) || obs_dect_->CheckObsCollision(p3_c) ||
       obs_dect_->CheckObsCollision(p4_c));
  neigh.clear();
  if ((!rflag) && (!lflag)) {
    neigh.push_back(p2);
    neigh.push_back(p2_c);
    neigh.push_back(p4);
    neigh.push_back(p4_c);
    neigh.push_back(p1);
    neigh.push_back(p1_c);
    neigh.push_back(p3);
    neigh.push_back(p3_c);
    neigh.push_back(p);
    return 3;
  } else if ((!rflag) && lflag) {
    neigh.push_back(p2);
    neigh.push_back(p2_c);
    neigh.push_back(p1);
    neigh.push_back(p1_c);
    neigh.push_back(p);
    return 1;
  } else if (rflag && (!lflag)) {
    neigh.push_back(p4);
    neigh.push_back(p4_c);
    neigh.push_back(p3);
    neigh.push_back(p3_c);
    neigh.push_back(p);
    return 2;
  } else {
    return 0;
  }
}

std::vector<ccpp::Pointf> CoveragePlanning::Searchlittlestart(
    const int& index, const ccpp::Pointf& init_p, const int& xmin,
    const int& xmax, int& new_x, ccpp::Pointi& start) {
  std::cout << "Searchlittlestart" << std::endl;
  int cur = xmin;
  ccpp::Pointi uppoint, downpoint;
  std::vector<ccpp::Pointf> path1;
  std::vector<ccpp::Pointf> path2;
  std::vector<ccpp::Pointi> goals;
  std::vector<ccpp::Pointi> upoints, dpoints;
  std::vector<int>::iterator ymin_it, ymax_it;
  int ymin, ymax, ylittle, ybig;
  int downflag = 0;
  int upflag = 0;
  while (cur < xmax) {
    ymin_it = std::min_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymin = *ymin_it;
    ymax_it = std::max_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymax = *ymax_it;
    ylittle = ymin + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_half_car_width;
    ybig = ymax - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_half_car_width;
    while ((!downflag) || (!upflag)) {
      if (ylittle > ybig) {
        std::cout << "bad x" << std::endl;
        cur++;
        break;
      }
      uppoint.x = cur;
      uppoint.y = ylittle;
      uppoint.yaw = 90.0 * M_PI / 180.0;
      downpoint.x = cur;
      downpoint.y = ybig;
      downpoint.yaw = 90.0 * M_PI / 180.0;
      downflag = GetDownGoals(downpoint, dpoints);
      upflag = GetUpGoals(uppoint, upoints);
      if (upflag == 0) {
        ylittle++;
      }
      if (downflag == 0) {
        ybig--;
      }
      if (upflag && downflag) {
        goals.clear();
        if (upflag == 1 || upflag == 2) {
          goals.push_back(upoints[0]);
          goals.push_back(upoints[1]);
          goals.push_back(upoints[2]);
          goals.push_back(upoints[3]);
          goals.push_back(upoints[4]);
        } else {
          goals.push_back(upoints[0]);
          goals.push_back(upoints[1]);
          goals.push_back(upoints[2]);
          goals.push_back(upoints[3]);
          goals.push_back(upoints[4]);
          goals.push_back(upoints[5]);
          goals.push_back(upoints[6]);
          goals.push_back(upoints[7]);
          goals.push_back(upoints[8]);
        }
        // //test
        // start = goals.back();
        // new_x = cur;
        // std::vector<ccpp::Pointf> path1;
        // path1.push_back(goals.back());
        // return path1;rosrun ivpathplanner
        // //test
        for (int i = 0; i < goals.size(); i++) {
          AstarHybrid ahplanner(init_p, goals[i], binary_image_.clone());
          path1.clear();
          path1 = ahplanner.AHPlanning();
          if (!path1.empty()) {
            std::cout << "path_size:" << path1.size() << std::endl;
            ccpp::Pointi p1_cent;
            bool clockwise = true;
            if (goals[i].x > goals.back().x) {
              clockwise = true;
              p1_cent.x = goals.back().x + ccpp::cvcarmodel::cv_car_radius;
              p1_cent.y = goals.back().y;
              p1_cent.yaw = NormalizeAngle(goals[i].yaw);
            } else {
              clockwise = false;
              p1_cent.x = goals.back().x - ccpp::cvcarmodel::cv_car_radius;
              p1_cent.y = goals.back().y;
              p1_cent.yaw = NormalizeAngle(goals[i].yaw);
            }
            float angle = fabs(NormalizeAngle(goals[i].yaw) -
                               NormalizeAngle(goals.back().yaw));
            angle = angle > M_PI ? 2 * M_PI - angle : angle;
            CircleFit(p1_cent, angle, clockwise, path2);
            for (int j = 0; j < path2.size(); j++) {
              path1.push_back(path2[j]);
            }
            start = goals.back();
            new_x = cur;
            std::cout << "the start point of region " << index << "is "
                      << start.x << "," << start.y << "," << start.yaw
                      << std::endl;
            return path1;
          }
        }
        ylittle++;
        upflag = 0;
        downflag = 0;
      }
    }
  }
  start = ccpp::Pointi(0, 0, 0);
  new_x = 0;
  return path1;
}

bool CoveragePlanning::Searchlittleend(const int& x, int& new_x,
                                       const int& index, const int& xmin) {
  std::cout << "Searchlittleend 1" << std::endl;
  int cur = x;
  std::vector<int>::iterator ymin_it, ymax_it;
  ccpp::Pointi uppoint, downpoint;
  std::vector<ccpp::Pointi> upoints, dpoints;
  int ymin, ymax, ylittle, ybig;
  int downflag = 0;
  int upflag = 0;
  while (cur > xmin) {
    ymin_it = std::min_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymin = *ymin_it;
    ymax_it = std::max_element(std::begin(free_contours_map_[index][cur]),
                               std::end(free_contours_map_[index][cur]));
    ymax = *ymax_it;
    ylittle = ymin + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_half_car_width;
    ybig = ymax - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_half_car_width;
    while ((!downflag) || (!upflag)) {
      if (ylittle > ybig) {
        cur--;
        break;
      }
      uppoint.x = cur;
      uppoint.y = ylittle;
      uppoint.yaw = 90.0 * M_PI / 180.0;
      downpoint.x = cur;
      downpoint.y = ybig;
      downpoint.yaw = 90.0 * M_PI / 180.0;
      downflag = GetDownGoals(downpoint, dpoints);
      upflag = GetUpGoals(uppoint, upoints);
      if (downflag == 0) {
        ybig -= 1;
      }
      if (upflag == 0) {
        ylittle++;
      }
      if (upflag && downflag) {
        new_x = cur;
        return true;
      }
    }
  }
  return false;
}

bool CoveragePlanning::Searchfirstline(const int& cur, const int& xmax,
                                       const int& index,
                                       std::vector<ccpp::Pointi>& upoints,
                                       std::vector<ccpp::Pointi>& dpoints) {
  std::cout << "search first line 1" << cur << std::endl;
  int curx = cur;
  ccpp::Pointi uppoint, downpoint;
  std::vector<int>::iterator ymin_it, ymax_it;
  int ymin, ymax, ylittle, ybig;
  int downflag = 0;
  int upflag = 0;
  ymin_it = std::min_element(std::begin(free_contours_map_[index][curx]),
                             std::end(free_contours_map_[index][curx]));
  ymin = *ymin_it;
  ymax_it = std::max_element(std::begin(free_contours_map_[index][curx]),
                             std::end(free_contours_map_[index][curx]));
  ymax = *ymax_it;
  ylittle = ymin + ccpp::cvcarmodel::cv_car_radius +
            ccpp::cvcarmodel::cv_half_car_width;
  ybig = ymax - ccpp::cvcarmodel::cv_car_radius -
         ccpp::cvcarmodel::cv_half_car_width;
  while ((!downflag) || (!upflag)) {
    if (ylittle > ybig) {
      std::cout << "bad x" << std::endl;
      break;
    }
    uppoint.x = cur;
    uppoint.y = ylittle;
    uppoint.yaw = 90.0 * M_PI / 180.0;
    downpoint.x = cur;
    downpoint.y = ybig;
    downpoint.yaw = 90.0 * M_PI / 180.0;
    downflag = GetDownGoals(downpoint, dpoints);
    upflag = GetUpGoals(uppoint, upoints);
    if (downflag == 0) ybig--;
    if (upflag == 0) ylittle++;
    if (upflag && downflag) {
      return true;
    }
  }
  return false;
}

bool CoveragePlanning::Searchsecondline(const int& cur, ccpp::Pointi& new_p,
                                        const int& xmax, const int& index,
                                        std::vector<ccpp::Pointi>& source,
                                        std::vector<ccpp::Pointi>& upoints,
                                        std::vector<ccpp::Pointi>& dpoints,
                                        std::vector<ccpp::Pointf>& path) {
  std::cout << "Searchsecondline 1" << std::endl;
  int curx = cur;
  ccpp::Pointi uppoint, downpoint;
  std::vector<int>::iterator ymin_it, ymax_it;
  int ymin, ymax, ylittle, ybig;
  int downflag = 0;
  int upflag = 0;
  while (curx < xmax) {
    ymin_it = std::min_element(std::begin(free_contours_map_[index][curx]),
                               std::end(free_contours_map_[index][curx]));
    ymin = *ymin_it;
    ymax_it = std::max_element(std::begin(free_contours_map_[index][curx]),
                               std::end(free_contours_map_[index][curx]));
    ymax = *ymax_it;
    ylittle = ymin + ccpp::cvcarmodel::cv_car_radius +
              ccpp::cvcarmodel::cv_half_car_width;
    ybig = ymax - ccpp::cvcarmodel::cv_car_radius -
           ccpp::cvcarmodel::cv_half_car_width;
    while ((!downflag) || (!upflag)) {
      if (ylittle > ybig) {
        std::cout << "bad x" << std::endl;
        curx++;
        break;
      }
      uppoint.x = cur;
      uppoint.y = ylittle;
      uppoint.yaw = 90.0 * M_PI / 180.0;
      downpoint.x = cur;
      downpoint.y = ybig;
      downpoint.yaw = 90.0 * M_PI / 180.0;
      downflag = GetDownGoals(downpoint, dpoints);
      upflag = GetUpGoals(uppoint, upoints);
      if (downflag == 0) ybig--;
      if (upflag == 0) ylittle++;
      if (upflag && downflag) {
        // //test
        // path.push_back(upoints.back());
        // return true;
        // //test
        bool lineflag = Getlittleline(source, upoints, path);
        if (!lineflag) {
          ylittle++;
          upflag = 0;
          downflag = 0;
        } else {
          new_p = upoints.back();
          return true;
        }
      }
    }
  }
  return false;
}

bool CoveragePlanning::Getlittleline(std::vector<ccpp::Pointi>& p1,
                                     std::vector<ccpp::Pointi>& p2,
                                     std::vector<ccpp::Pointf>& path) {
  std::vector<ccpp::Pointf> path1;
  ccpp::Pointi p1_cent;
  ccpp::Pointi p2_cent;
  float angle;
  bool clockwise;
  std::cout << "Getlittleline:" << p1.size() << "," << p2.size() << std::endl;
  for (int i = 0; i < p1.size(); i++) {
    for (int j = 0; j < p2.size(); j++) {
      AstarHybrid ahplanner1(p1[i], p2[j], binary_image_.clone());
      path1.clear();
      path1 = ahplanner1.AHPlanning();
      if (path1.size() <= 2) {
        std::cout << "error" << std::endl;
        continue;
      }
      if (p1[i].x < p1.back().x) {
        clockwise = false;
        p1_cent.x = p1.back().x - ccpp::cvcarmodel::cv_car_radius;
      } else {
        clockwise = true;
        p1_cent.x = p1.back().x + ccpp::cvcarmodel::cv_car_radius;
      }
      p1_cent.y = p1.back().y;
      p1_cent.yaw = NormalizeAngle(p1.back().yaw);
      angle = fabs(NormalizeAngle(p1[i].yaw) - NormalizeAngle(p1.back().yaw));
      angle = angle > M_PI ? 2 * M_PI - angle : angle;
      CircleFit(p1_cent, angle, clockwise, path);
      // GetDubinsPath(p1[2], p1[i], path);
      for (int k = 0; k < path1.size(); k++) {
        path.push_back(path1[k]);
      }
      if (p2[j].x < p2.back().x) {
        clockwise = false;
        p2_cent.x = p2.back().x - ccpp::cvcarmodel::cv_car_radius;
      } else {
        clockwise = true;
        p2_cent.x = p2.back().x + ccpp::cvcarmodel::cv_car_radius;
      }
      p2_cent.y = p2.back().y;
      p2_cent.yaw = NormalizeAngle(p2[j].yaw);
      angle = fabs(NormalizeAngle(p2[j].yaw) - NormalizeAngle(p2.back().yaw));
      angle = angle > M_PI ? 2 * M_PI - angle : angle;
      CircleFit(p2_cent, angle, clockwise, path);
      // GetDubinsPath(p2[i], p2[2], path);
      std::cout << "ok" << std::endl;
      return true;
    }
  }
  return false;
}
bool CoveragePlanning::Searchlittledown(ccpp::Pointi& start,
                                        ccpp::Pointi& new_p, const int& index,
                                        const int& xmax,
                                        std::vector<ccpp::Pointf>& path) {
  // TODO(zhangfuqiang)
  bool secondline;
  int cur = start.x;
  ccpp::Pointi uppoint, downpoint;
  std::vector<ccpp::Pointi> upoints1, dpoints1, upoints2, dpoints2;
  std::vector<int>::iterator ymin_it, ymax_it;
  int ymin, ymax;
  int downflag = 0;
  int upflag = 0;
  bool firstline = Searchfirstline(cur, xmax, index, upoints1, dpoints1);
  std::cout << "path_size" << path.size() << std::endl;
  GetYline(start, dpoints1.back(), true, path);
  if (cur + ccpp::cvcarmodel::cv_car_width < xmax) {
    secondline =
        Searchsecondline(cur + ccpp::cvcarmodel::cv_car_width, new_p, xmax,
                         index, dpoints1, upoints2, dpoints2, path);
  } else {
    if (dpoints1.size() == 5) {
      if (dpoints1[0].x < dpoints1.back().x) {
        ccpp::Pointi p_cent;
        p_cent.x = dpoints1.back().x - ccpp::cvcarmodel::cv_car_radius;
        p_cent.y = dpoints1.back().y;
        p_cent.yaw = dpoints1.back().yaw;
        float angle = fabs(NormalizeAngle(dpoints1[0].yaw) -
                           NormalizeAngle(dpoints1.back().yaw));
        angle = angle > M_PI ? 2 * M_PI - angle : angle;
        CircleFit(p_cent, angle, false, path);
      } else {
        ccpp::Pointi p_cent;
        p_cent.x = dpoints1.back().x + ccpp::cvcarmodel::cv_car_radius;
        p_cent.y = dpoints1.back().y;
        p_cent.yaw = dpoints1.back().yaw;
        float angle = fabs(NormalizeAngle(dpoints1[0].yaw) -
                           NormalizeAngle(dpoints1.back().yaw));
        angle = angle > M_PI ? 2 * M_PI - angle : angle;
        CircleFit(p_cent, angle, true, path);
      }
    } else {
      ccpp::Pointi p_cent;
      p_cent.x = dpoints1.back().x + ccpp::cvcarmodel::cv_car_radius;
      p_cent.y = dpoints1.back().y;
      p_cent.yaw = dpoints1.back().yaw;
      float angle = fabs(NormalizeAngle(dpoints1[0].yaw) -
                         NormalizeAngle(dpoints1.back().yaw));
      angle = angle > M_PI ? 2 * M_PI - angle : angle;
      CircleFit(p_cent, angle, true, path);
    }
    return false;
  }
  return secondline;
}

std::vector<ccpp::Pointf> CoveragePlanning::SingleRegionCoverPlanningTest(
    const int& index, const ccpp::Pointf& init_p) {
  std::vector<ccpp::Pointf> path;
  std::vector<ccpp::Pointf> path1;
  std::vector<ccpp::Pointf> rawpath;
  std::vector<ccpp::Pointi> ldpoints;
  std::vector<ccpp::Pointi> rdpoints;
  std::vector<ccpp::Pointi> rupoints;
  std::vector<ccpp::Pointi> lupoints;
  ccpp::Pointi start, new_p;
  ccpp::Pointi start_p, end_p;
  int start_x, end_x, down_x, up_x;
  static int count = 0;
  bool upflag = true;
  bool downflag = true;
  bool x_flag = false;
  bool newdownflag;
  int new_down_x = 0;
  int new_up_x = 0;
  std::cout << "start single" << std::endl;
  std::vector<int> start_bound = GetCarBoundary(start, false);
  std::vector<int> cur_bound = regionbound_[index];
  if (cur_bound[1] - cur_bound[0] > (ccpp::cvcarmodel::cv_half_car_width +
                                     ccpp::cvcarmodel::cv_car_radius * 2) *
                                        2 &&
      cur_bound[3] - cur_bound[2] > (ccpp::cvcarmodel::cv_half_car_width +
                                     ccpp::cvcarmodel::cv_car_radius) *
                                        2) {
    std::cout << "it is a large region " << index << std::endl;
    path = Searchstartlimit(
        index, init_p, cur_bound[0] + ccpp::cvcarmodel::cv_half_car_width,
        cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width, start_x);
    if (path.empty()) return path;
    upflag = Searchendlimit(cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width,
                            end_x, index,
                            cur_bound[0] + ccpp::cvcarmodel::cv_half_car_width);
    if (!upflag) return path;
    int middle_x = static_cast<int>((start_x + end_x) / 2.0);
    down_x = start_x;
    up_x = middle_x;
    int clean_width = up_x - down_x;
    if (clean_width <
        ccpp::cvcarmodel::cv_car_radius * 2 + ccpp::cvcarmodel::cv_car_width) {
      clean_width =
          ccpp::cvcarmodel::cv_car_radius * 2 + ccpp::cvcarmodel::cv_car_width;
      up_x = down_x + clean_width;
    }
    std::cout << "startx:" << down_x << ", endx:" << end_x
              << ", clean_width:" << clean_width << std::endl;
    while (up_x <= end_x) {
      count++;
      // std::cout << "the circle count is :" << count << std::endl;
      // std::cout << "start clean" << std::endl;
      downflag = Searchdownlimit(
          down_x, new_down_x, index,
          cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width - 4, ldpoints,
          lupoints);
      if (!downflag) break;
      upflag =
          Searchuplimit(up_x, new_up_x, index,
                        cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width - 4,
                        rdpoints, rupoints);
      if (!upflag) break;
      // std::cout << "left: up ---> down start" << endl;
      GetYline(lupoints[2], ldpoints[2], true, path);
      // std::cout << "down: left ---> right start" << endl;
      x_flag = GetXline(ldpoints, rdpoints, path);
      // std::cout << "right: down ---> up start" << endl;
      GetYline(rdpoints[2], rupoints[2], false, path);
      down_x = new_down_x + ccpp::cvcarmodel::cv_car_width;
      up_x = new_up_x + ccpp::cvcarmodel::cv_car_width;
      newdownflag = Searchdownlimit(
          down_x, new_down_x, index,
          cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width - 4, ldpoints,
          lupoints);
      if (!newdownflag) break;
      // std::cout << "up: right ---> left start" << endl;
      GetXline(rupoints, lupoints, path);
    }
  } else if (cur_bound[1] - cur_bound[0] <=
                 (ccpp::cvcarmodel::cv_half_car_width +
                  ccpp::cvcarmodel::cv_car_radius * 2) *
                     2 &&
             cur_bound[3] - cur_bound[2] >
                 (ccpp::cvcarmodel::cv_half_car_width +
                  ccpp::cvcarmodel::cv_car_radius) *
                     2) {
    std::cout << "it is a small region " << index << std::endl;
    std::vector<ccpp::Pointf> littlepath;
    ccpp::Pointf start_point;
    if (path.empty()) {
      start_point = init_p;
    } else {
      start_point = path.back();
    }
    littlepath = Searchlittlestart(
        index, start_point, cur_bound[0] + ccpp::cvcarmodel::cv_half_car_width,
        cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width, start_x, start);
    if (littlepath.empty()) {
      std::cout << "this region can not find a start point" << std::endl;
      return path;
    }
    for (int i = 0; i < littlepath.size(); i++) {
      path.push_back(littlepath[i]);
    }
    bool end_flag = Searchlittleend(
        cur_bound[1] - ccpp::cvcarmodel::cv_half_car_width, end_x, index,
        cur_bound[0] + ccpp::cvcarmodel::cv_half_car_width);
    if (end_flag == false) return path;
    down_x = start_x;
    start_p = start;
    std::cout << "startx:" << down_x << ", endx:" << end_x << std::endl;
    while (down_x < end_x) {
      downflag = Searchlittledown(start_p, new_p, index, end_x, path);
      if (!downflag) break;
      down_x = new_p.x;
      start_p = new_p;
    }
  }
  return path;
}

std::vector<ccpp::Pointf> CoveragePlanning::ToEnd(ccpp::Pointf& start) {
  std::vector<ccpp::Pointf> path;
  AstarHybrid ahplanner(start, end_p_, binary_image_.clone());
  path = ahplanner.AHPlanning();
  return path;
}

/**********************************************
 * Function: CompletePlanning
 * Description:
 * Input: index----the index of the subregion
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool CoveragePlanning::CompletePlanning(std::string& mappath) {
  std::vector<ccpp::Pointf> path;
  ccpp::Pointf start_point = start_p_;
  bool start_flag = true;
  int regionsize = free_contours_.size();
  std::cout << "regionsize:" << regionsize << std::endl;
  if (obs_dect_->CheckObsCollision(start_p_)) {
    std::cout << "start point is not in the clean region" << std::endl;
    return false;
  }
  if (obs_dect_->CheckObsCollision(end_p_)) {
    std::cout << "end point is not in the clean region" << std::endl;
    return false;
  }

  for (int i = 0; i < regionsize; i++) {
    // std::vector<ccpp::Pointf> single = SingleRegionCoverPlanning(i,
    // start_point);
    std::vector<ccpp::Pointf> single =
        SingleRegionCoverPlanningTest(i, start_point);
    if (!single.empty()) {
      for (int j = 0; j < single.size(); j++) {
        path.push_back(single[j]);
      }
      start_point = path.back();
    } else {
      std::cout << i << "region is not cleaned" << std::endl;
    }
  }
  std::cout << "to the end points" << std::endl;
  std::vector<ccpp::Pointf> endpoints;
  endpoints = ToEnd(path.back());
  for (int i = 0; i < endpoints.size(); i++) {
    path.push_back(endpoints[i]);
  }
  std::string pathName = "";
  // ros::NodeHandle mh;
  // mh.param("routemap", pathName, pathName);

  if (path.size() < 2) return true;
  std::string blanket_path = mappath + std::to_string(write_seg_) + "-seg";
  std::cout << blanket_path << std::endl;
  std::fstream output_file;
  output_file.open(blanket_path, ofstream::out);
  for (int i = 0; i < path.size() - 1; i++) {
    float c_x = path[i].x / 20.0;
    float c_y = path[i].y / 20.0;
    float c_degree = NormalizeDeg(path[i].yaw * 180.0 / M_PI);
    float dis_err =
        std::hypot(c_x - path[i + 1].x / 20.0, c_y - path[i + 1].y / 20.0);
    if (dis_err < 1e-3) continue;
    // output_file << "0,0,0,0,0,0,0,0,0,0,0,0,0," << c_x << "," << c_y <<
    // ",0,0,"
    //             << c_degree << std::endl;
    output_file << c_x << "," << c_y << "," << c_degree << ",0" << std::endl;
  }
  output_file.close();
  return true;
}

bool CoveragePlanning::CompletePlanningPoly(std::vector<ccpp::Pointf>& path) {
  path.clear();
  ccpp::Pointf start_point = start_p_;
  bool start_flag = true;
  int regionsize = free_contours_.size();
  std::cout << "regionsize:" << regionsize << std::endl;
  if (obs_dect_->CheckObsCollision(start_p_)) {
    std::cout << "start point is not in the clean region" << std::endl;
    return false;
  }
  if (obs_dect_->CheckObsCollision(end_p_)) {
    std::cout << "end point is not in the clean region" << std::endl;
    return false;
  }

  for (int i = 0; i < regionsize; i++) {
    std::vector<ccpp::Pointf> single =
        SingleRegionCoverPlanningTest(i, start_point);
    if (!single.empty()) {
      for (int j = 0; j < single.size(); j++) {
        path.push_back(single[j]);
      }
      start_point = path.back();
    } else {
      std::cout << i << "region is not cleaned" << std::endl;
    }
  }
  std::cout << "to the end points" << std::endl;
  std::vector<ccpp::Pointf> endpoints;
  endpoints = ToEnd(path.back());
  for (int i = 0; i < endpoints.size(); i++) {
    path.push_back(endpoints[i]);
  }
  return true;
}