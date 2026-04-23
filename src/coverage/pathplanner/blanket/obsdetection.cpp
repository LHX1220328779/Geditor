#include "pathplanner/blanket/obsdetection.h"
using namespace cv;
/**********************************************
 * Function: ObsDetection
 * Description: the construct function for CoveragePlanning
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ObsDetection::ObsDetection() {}

/**********************************************
 * Function: ObsDetection
 * Description: the construct function for CoveragePlanning
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ObsDetection::ObsDetection(cv::Mat& map) {
  obsmap_ = map.clone();
  rows_ = obsmap_.rows;
  cols_ = obsmap_.cols;
  front_length_ = ccpp::cvcarmodel::cv_front2base;
  back_length_ = ccpp::cvcarmodel::cv_tail2base;
  half_width_ = ccpp::cvcarmodel::cv_half_car_width;
}

/**********************************************
 * Function: ~ObsDetection
 * Description: the desconstruct function for CoveragePlanning
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ObsDetection::~ObsDetection() {}

bool ObsDetection::CheckObsCollision(const ccpp::Pointf& point) {
  bool flag = false;
  std::vector<cv::Point2f> polygon = SetCarModel(point);
  std::vector<int> boundary = GetCarBoundary(polygon);
  // std::cout << "boundary:" << boundary[0] << ","
  //                         << boundary[1] << ","
  //                         << boundary[2] << ","
  //                         << boundary[3]<< std::endl;
  for (int x = boundary[0]; x < boundary[2]; x++) {
    for (int y = boundary[1]; y < boundary[3]; y++) {
      // if (y >= obsmap_.rows || x >= obsmap_.cols) {
      //   std::cout << "------------error--------------------";
      //   std::cout << x << "," << y << "," << obsmap_.rows << "," <<
      //   obsmap_.cols<< std::endl;
      // }
      if (obsmap_.at<uchar>(y, x) == 0) {
        // std::cout << "obs_point:" << x << "," << y << std::endl;
        if (cv::pointPolygonTest(polygon, cv::Point(x, y), false) != 1) {
          // std::cout << "boundary:" << boundary[0] << ","
          //                 << boundary[1] << ","
          //                 << boundary[2] << ","
          //                 << boundary[3]<< std::endl;
          return true;
        }
      }
    }
  }
  return false;
}

/**********************************************
 * Function: SetCarModel
 * Description:
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<cv::Point2f> ObsDetection::SetCarModel(const ccpp::Pointf point) {
  float x0 = point.x;
  float y0 = point.y;
  float theta = point.yaw;

  cv::Point2f p1, p2, p3, p4;
  std::vector<cv::Point2f> polygon;

  p1.x = x0 + front_length_ * cos(theta) - half_width_ * sin(theta);
  p1.y = y0 + front_length_ * sin(theta) + half_width_ * cos(theta);
  p2.x = x0 - back_length_ * cos(theta) - half_width_ * sin(theta);
  p2.y = y0 - back_length_ * sin(theta) + half_width_ * cos(theta);
  p3.x = x0 - back_length_ * cos(theta) + half_width_ * sin(theta);
  p3.y = y0 - back_length_ * sin(theta) - half_width_ * cos(theta);
  p4.x = x0 + front_length_ * cos(theta) + half_width_ * sin(theta);
  p4.y = y0 + front_length_ * sin(theta) - half_width_ * cos(theta);

  polygon.push_back(p1);
  polygon.push_back(p2);
  polygon.push_back(p3);
  polygon.push_back(p4);

  return polygon;
}

/**********************************************
 * Function: GetCarBoundary
 * Description:
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<int> ObsDetection::GetCarBoundary(
    const std::vector<cv::Point2f> polygon) {
  std::vector<int> boundary;
  // std::cout << "1:" << polygon[0].x << ":" << polygon[0].y <<std::endl;
  // std::cout << "1:" << polygon[1].x << ":" << polygon[1].y <<std::endl;
  // std::cout << "1:" << polygon[2].x << ":" << polygon[2].y <<std::endl;
  // std::cout << "1:" << polygon[3].x << ":" << polygon[3].y <<std::endl;
  float ax[4] = {polygon[0].x, polygon[1].x, polygon[2].x, polygon[3].x};
  float ay[4] = {polygon[0].y, polygon[1].y, polygon[2].y, polygon[3].y};
  std::sort(ax, ax + 4);
  std::sort(ay, ay + 4);
  int x_min = static_cast<int>(ax[0]);
  int y_min = static_cast<int>(ay[0]);
  int x_max = static_cast<int>(ax[3]) + 1;
  int y_max = static_cast<int>(ay[3]) + 1;
  boundary.push_back(x_min);
  boundary.push_back(y_min);
  boundary.push_back(x_max);
  boundary.push_back(y_max);
  return boundary;
}
