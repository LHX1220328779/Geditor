#ifndef INCLUDE_PATHPLANNER_BLANKETPLANNER_WALLFOLLOWER_H_
#define INCLUDE_PATHPLANNER_BLANKETPLANNER_WALLFOLLOWER_H_

#include <opencv2/opencv.hpp>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace pathplanner {
namespace coverage {

class WallFollower {
 public:
  WallFollower();
  explicit WallFollower(const std::string &filepath);
  ~WallFollower() = default;

 private:
  /**
   * @brief : find the max contours of the origin image
   * @param : none
   * @return: none
   **/
  bool ContoursDetection();
  /**
   * @brief : convert color image to binary image
   * @param : none
   * @return: return false if the origin image is empty,
              return true otherwise.
  **/
  bool Color2Binary();

 private:
  cv::Mat orig_image_;    // origin image
  cv::Mat gray_image_;    // gray image
  cv::Mat binary_image_;  // binary image
  std::string filepath_;  // save the path for the origin image
  int rows_;              // rows of the origin image
  int cols_;              // cols of the origin image
};

}  // namespace coverage
}  // namespace pathplanner

#endif  // INCLUDE_PATHPLANNER_BLANKETPLANNER_WALLFOLLOWER_H_
