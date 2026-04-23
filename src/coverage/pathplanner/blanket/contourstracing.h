#ifndef IVPATHPLANNING_PATHPLANNER_CONTOURSTRACING_H
#define IVPATHPLANNING_PATHPLANNER_CONTOURSTRACING_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include "coverage/pathplanner/blanket/commontypes.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

class ContoursTracing {
 public:
  ContoursTracing();
  ContoursTracing(const std::string& filepath);
  ContoursTracing(const std::string& filepath, const int& index);
  ContoursTracing(const cv::Mat& origin);
  ContoursTracing(const std::string& filepath, const int& x, const int& y);
  ~ContoursTracing();
  bool ContoursDetection();
  bool SliceDecomposition();
  bool SliceDecomposition(const bool& fullflag);
  bool SingleContoursDetection();
  std::vector<cv::Point> SinglePathPlanning(const int& index);
  bool ShowImage();
  bool ShowBinaryImage();

 private:
  bool Color2Binary();
  bool Color2Binary(const int& index);
  bool ShowOrigImage();
  bool ShowGrayImage();
  bool ShowBinaryTestImage();
  bool ShowSingleContour(std::string name);
  bool FloodFilling();
  bool ClearContourMat();
  bool InsideFreeSpace(const int& row, const int& col);
  ccpp::Slice GetSingleSlice(const int& col);
  bool RectInterception();
  int IntersectionDetection(const int& first_source, const int& first_dest,
                            const int& second_source, const int& second_dest);
  std::vector<int> GetSegemnts(const ccpp::Segment& c_segment,
                               const ccpp::Slice& c_slice);
  bool SetRegionNum(ccpp::Slice& pre_slice, ccpp::Slice& cur_slice,
                    const int& cur_col);
  bool SetRegionNum(ccpp::Slice& pre_slice, ccpp::Slice& cur_slice,
                    const int& cur_col, const bool& fullflag);
  bool ContoursOptimization();
  bool GetContours();
  int CurrentSubregionDetection(const int& cur_x, const int& cur_y);

 public:
  // TEST
  bool TestDrawColorImage();
  bool TestShowContoursresult();
  bool TestContoursDetection();
  bool TestContourImage(const int& index, const bool& freeflag);
  std::fstream output_file;

 public:
  cv::Mat binary_image_;
  std::map<int, std::vector<cv::Point> > free_contours_;
  std::vector<std::vector<int> > connection_map_;
  std::vector<int> traversing_order_;

 private:
  std::string filepath_;
  cv::Mat orig_image_;
  cv::Mat gray_image_;
  cv::Mat binary_image_clone_;
  cv::Mat binary_image_clone_test_;
  cv::Mat single_contour_mat_;
  cv::Mat color_image_;
  std::vector<std::vector<cv::Point> > contours_;
  // std::map<int, std::vector<cv::Point> > free_contours_;
  std::map<int, std::vector<cv::Point> > obs_contours_;
  std::map<int, int> free_connection_;

 private:
  int curx_;
  int cury_;
  int rows_;
  int cols_;
  int free_index_;
  int obs_index_;
  int moving_step_;
  bool binary_flag_;
  std::vector<int> boundary_;
};

#endif
