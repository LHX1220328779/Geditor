#ifndef IVPATHPLANNING_PATHPLANNER_OBSDETECTION_H
#define IVPATHPLANNING_PATHPLANNER_OBSDETECTION_H

#include <opencv2/opencv.hpp>
#include "coverage/pathplanner/blanket/commontypes.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

class ObsDetection {
 public:
  ObsDetection();
  ObsDetection(cv::Mat& map);
  ~ObsDetection();
  bool CheckObsCollision(const ccpp::Pointf& point);

 private:
  std::vector<cv::Point2f> SetCarModel(const ccpp::Pointf point);
  std::vector<int> GetCarBoundary(const std::vector<cv::Point2f> polygon);

 private:
  cv::Mat obsmap_;
  int rows_;
  int cols_;
  float front_length_;
  float back_length_;
  float half_width_;
};

#endif