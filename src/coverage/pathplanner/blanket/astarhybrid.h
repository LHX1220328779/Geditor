#ifndef IVPATHPLANNING_PATHPLANNER_ASTARHYBRID_H
#define IVPATHPLANNING_PATHPLANNER_ASTARHYBRID_H

#include <opencv2/opencv.hpp>
#include "coverage/pathplanner/blanket/commontypes.h"
#include "coverage/pathplanner/blanket/obsdetection.h"
#include "coverage/toolbox.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
// #include "site.h"

const int END_START = 1;
const int END_LAST = 2;
const int END_NORMAL = 3;
const int END_ERROR = 4;

class AstarHybrid {
 public:
  AstarHybrid();
  AstarHybrid(const ccpp::Pointf& start_point, const ccpp::Pointf& end_point,
              const cv::Mat& bmap);
  AstarHybrid(const geometry::Site& start_point,
              const geometry::Site& end_point, const std::string& bmap);
  ~AstarHybrid();
  bool MapBinary(const std::string& mappath);
  float NormalRad(float angle);
  bool IsGoal(const ccpp::Node3D& node);
  bool Dubinsvalid(ccpp::Node3D& cur, std::vector<ccpp::Pointf>& d_curve,
                   float& d_length);
  bool UpdateH(ccpp::Node3D& cur, std::vector<ccpp::Pointf>& d_curve);
  bool SetOpen(ccpp::Node3D& cur);
  bool SetClose(ccpp::Node3D& cur);
  bool AddCloseList(ccpp::Node3D& node);
  std::vector<ccpp::Node3D> CreateForwardPoints(ccpp::Node3D& node);
  std::vector<ccpp::Pointf> AHPlanning();
  bool AHPlanning(geometry::SiteVec& final);
  std::vector<ccpp::Pointf> ReconstructPath(ccpp::Node3D& last, int& flag,
                                            ccpp::UnorderT& closelist_);
  std::vector<ccpp::Pointf> SamplePath(std::vector<ccpp::Pointf>& path);
  bool SamplePath(std::vector<ccpp::Pointf>& path, geometry::SiteVec& final);

 private:
  ccpp::Node3D start_;
  ccpp::Node3D end_;
  cv::Mat map_;
  ObsDetection* obsmap_;
  ccpp::UnorderT closelist1_;
  ccpp::UnorderF cost_so_far_;
  ccpp::UnorderT openlist_;
  float steplength_;
};

#endif