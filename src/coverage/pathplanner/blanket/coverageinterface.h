#ifndef IVPATHPLANNING_PATHPLANNER_COVERAGEINTERFACE_H
#define IVPATHPLANNING_PATHPLANNER_COVERAGEINTERFACE_H

// #include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "coverage/geometry/geoheader.h"
#include "coverage/pathplanner/blanket/commontypes.h"
#include "coverage/pathplanner/blanket/obsdetection.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#define PATH_FILE_DEBUG 0

class CoverageInterface {
 public:
  typedef geometry::Site PointXY;
  CoverageInterface();
  // CoverageInterface(ros::NodeHandle nh);
  ~CoverageInterface();
  bool PlanningInterface(const double &sx, const double &sy, const double &syaw,
                         const double &dx, const double &dy, const double &dyaw,
                         const int &index, std::string &mappath);
  bool PlanningInterface(const double &sx, const double &sy, const double &syaw,
                         const double &dx, const double &dy, const double &dyaw,
                         const std::vector<double> &x,
                         const std::vector<double> &y);
  bool DetectInterface(const int &index);
  float NormalRAD(float angle);
  float NormalDEG(float angle);
  bool Polygon2Map(const std::vector<double> &x, const std::vector<double> &y);
  bool Pathvec2Sitevec(const std::vector<ccpp::Pointf> &path,
                       geometry::SiteVec &pathsite);
  bool GenerateSamplePoint(const geometry::SiteVec &site_path,
                           geometry::SiteVec &sample_point);
  bool AverageSimpling(const geometry::SiteVec &site_path,
                       geometry::SiteVec &sample_point);
  bool GenerateCurve(const geometry::SiteVec &sample_point,
                     geometry::SiteVec &final_path);
  void FromSample2Final(const geometry::SiteVec &sample_seg,
                        geometry::SiteVec &final_seg);
  void Path2File(const geometry::SiteVec path, const std::string &blanket_path);
  void Recovery(geometry::SiteVec &path);

 private:
  ccpp::Pointf start_;
  ccpp::Pointf end_;
  int clean_index;
  std::string basemap;
  cv::Mat origin_map_;
  int x_offset_;
  int y_offset_;

 public:
  geometry::SiteVec sample_point_;
  geometry::SiteVec final_path_;
};

#endif