#include "pathplanner/blanketplanner/wallfollower.h"
#include <iostream>

#if CV_VERSION_MAJOR > 3
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#define CV_RETR_CCOMP cv::RETR_CCOMP
#endif

namespace pathplanner {
namespace coverage {

WallFollower::WallFollower() {
  filepath_ = "/home/geditor/Desktop/test.png";
  orig_image_ = cv::imread(filepath_).clone();
  rows_ = orig_image_.rows;
  cols_ = orig_image_.cols;
  Color2Binary();
}

WallFollower::WallFollower(const std::string &filepath) : filepath_(filepath) {
  orig_image_ = cv::imread(filepath_).clone();
  rows_ = orig_image_.rows;
  cols_ = orig_image_.cols;
  Color2Binary();
}

bool WallFollower::ContoursDetection() {
  if (orig_image_.empty()) {
    std::cout << "orig image is empty" << std::endl;
    return false;
  }
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_image_, contours, hierarchy, CV_RETR_CCOMP,
                   CV_CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    std::cout << "the image is a bug" << std::endl;
    return false;
  }
  return true;
}

bool WallFollower::Color2Binary() {
  if (orig_image_.empty()) {
    std::cout << "orig image is empty" << std::endl;
    return false;
  }
  cv::cvtColor(orig_image_, gray_image_, cv::COLOR_BGR2GRAY);
  cv::threshold(gray_image_, binary_image_, 145, 255, cv::THRESH_BINARY);
  return true;
}

}  // namespace coverage
}  // namespace pathplanner
