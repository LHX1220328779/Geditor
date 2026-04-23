#ifndef IVPATHPLANNING_PATHPLANNER_COVERAGEPLANNING_H
#define IVPATHPLANNING_PATHPLANNER_COVERAGEPLANNING_H

#include "coverage/pathplanner/blanket/contourstracing.h"
#include "coverage/pathplanner/blanket/obsdetection.h"

const bool c_inflag = true;
const bool c_outflag = false;

class CoveragePlanning {
 public:
  CoveragePlanning();
  CoveragePlanning(const ContoursTracing& contourstracing,
                   const ccpp::Pointf& start_point,
                   const ccpp::Pointf& end_point, const int& index);
  ~CoveragePlanning();
  bool CompletePlanning(std::string& mappath);
  bool CompletePlanningPoly(std::vector<ccpp::Pointf>& path);

 private:
  float NormalizeAngle(float angle);
  float NormalizeDeg(float angle);
  std::vector<ccpp::Pointf> GetThreeGoals(const ccpp::Pointf& p);
  std::vector<ccpp::Pointf> GetFiveGoals(const ccpp::Pointf& p);
  int SearchIndex();
  bool ClearContourMat();
  bool SetSingleFreespace(const int& index);
  bool GetOuterContours();
  bool ContoursExpanding(const int& index, const int& safe_expand_dis);
  bool GetInnerContours();
  bool CleanOrderCheck(const int& index);
  ccpp::Pointi GetLeftUpperPoint(const int& index, int& x, int& y);
  bool GetDubinsPath(const ccpp::Pointf& start, const ccpp::Pointf& end,
                     std::vector<ccpp::Pointf>& d_curve);
  bool CircleFit(ccpp::Pointi& p_cent, float& angle, bool anticlockwise,
                 std::vector<ccpp::Pointf>& path);
  std::vector<ccpp::Pointf> ToStart(const int& index,
                                    const ccpp::Pointf& init_p,
                                    ccpp::Pointi& start);
  ccpp::Pointi GetxlowCurvePoint(const ccpp::Pointi p, const bool& inflag);
  ccpp::Pointi GetxhighCurvePoint(const ccpp::Pointi p, const bool& inflag);
  bool GetDownLeftGoals(ccpp::Pointi& p, std::vector<ccpp::Pointi>& neigh);
  bool GetDownRightGoals(ccpp::Pointi& p, std::vector<ccpp::Pointi>& neigh);
  bool GetUpRightGoals(ccpp::Pointi& p, std::vector<ccpp::Pointi>& neigh);
  bool GetUpLeftGoals(ccpp::Pointi& p, std::vector<ccpp::Pointi>& neigh);
  ccpp::Pointi Searchdownleftlimit(const int& x, const int& index,
                                   std::vector<ccpp::Pointi>& neigh);
  ccpp::Pointi Searchdownrightlimit(const int& x, const int& index,
                                    std::vector<ccpp::Pointi>& neigh);
  ccpp::Pointi Searchuprightlimit(const int& x, const int& index,
                                  std::vector<ccpp::Pointi>& neigh);
  ccpp::Pointi Searchupleftlimit(const int& x, const int& index,
                                 std::vector<ccpp::Pointi>& neigh);
  ccpp::Pointi Searchdownright(const int& xinit, const int& index,
                               std::vector<ccpp::Pointi>& neigh,
                               const int& xmax);
  bool Searchdownlimit(const int& x, int& new_x, const int& index,
                       const int& xmax, std::vector<ccpp::Pointi>& ldpoints,
                       std::vector<ccpp::Pointi>& lupoints);
  bool Searchuplimit(const int& x, int& new_x, const int& index,
                     const int& xmax, std::vector<ccpp::Pointi>& rdpoints,
                     std::vector<ccpp::Pointi>& rupoints);
  std::vector<ccpp::Pointf> Searchstartlimit(const int& index,
                                             const ccpp::Pointf& init_p,
                                             const int& xmin, const int& xmax,
                                             int& new_x);
  bool Searchendlimit(const int& x, int& new_x, const int& index,
                      const int& xmin);
  bool GetXline(std::vector<ccpp::Pointi>& p1, std::vector<ccpp::Pointi>& p2,
                std::vector<ccpp::Pointf>& path);
  bool GetYline(ccpp::Pointi p1, ccpp::Pointi p2, bool downflag,
                std::vector<ccpp::Pointf>& path);
  std::vector<int> GetCarBoundary(const ccpp::Pointi& p,
                                  const bool& upper_flag);
  std::vector<int> GetSubregionBound(const int& index);
  bool GetRegionBound();
  bool GetContoursMap();
  std::vector<ccpp::Pointf> ToEnd(ccpp::Pointf& start);
  int GetDownGoals(const ccpp::Pointi& p, std::vector<ccpp::Pointi>& neigh);
  int GetUpGoals(const ccpp::Pointi& p, std::vector<ccpp::Pointi>& neigh);
  std::vector<ccpp::Pointf> Searchlittlestart(const int& index,
                                              const ccpp::Pointf& init_p,
                                              const int& xmin, const int& xmax,
                                              int& new_x, ccpp::Pointi& start);
  bool Searchlittleend(const int& x, int& new_x, const int& index,
                       const int& xmin);
  bool Searchfirstline(const int& cur, const int& xmax, const int& index,
                       std::vector<ccpp::Pointi>& upoints,
                       std::vector<ccpp::Pointi>& dpoints);
  bool Searchsecondline(const int& cur, ccpp::Pointi& new_p, const int& xmax,
                        const int& index, std::vector<ccpp::Pointi>& source,
                        std::vector<ccpp::Pointi>& upoints,
                        std::vector<ccpp::Pointi>& dpoints,
                        std::vector<ccpp::Pointf>& path);
  bool Getlittleline(std::vector<ccpp::Pointi>& p1,
                     std::vector<ccpp::Pointi>& p2,
                     std::vector<ccpp::Pointf>& path);
  bool Searchlittledown(ccpp::Pointi& start, ccpp::Pointi& new_p,
                        const int& index, const int& xmax,
                        std::vector<ccpp::Pointf>& path);
  std::vector<ccpp::Pointf> SingleRegionCoverPlanning(
      const int& index, const ccpp::Pointf& init_p);
  std::vector<ccpp::Pointf> SingleRegionCoverPlanningTest(
      const int& index, const ccpp::Pointf& init_p);
  std::vector<ccpp::Pointi> InterRegionPlanning(const int& index1,
                                                const ccpp::Pointi& p_source,
                                                const int& index2,
                                                const ccpp::Pointi& p_dest);

 public:
  cv::Mat binary_image_;
  std::map<int, std::vector<cv::Point> > free_contours_;
  std::map<int, std::map<int, std::vector<int> > > free_contours_map_;
  std::vector<std::vector<int> > connection_map_;
  std::vector<int> traversing_order_;
  ccpp::Pointf start_p_;
  ccpp::Pointf end_p_;
  int write_seg_;
  int init_index_;

 private:
  std::map<int, std::vector<cv::Point> > outer_contours_;
  std::map<int, std::vector<cv::Point> > inner_contours_;
  std::map<int, std::vector<int> > regionbound_;
  ObsDetection* obs_dect_;
  cv::Mat single_contour_mat_;
  int rows_;
  int cols_;
};

#endif