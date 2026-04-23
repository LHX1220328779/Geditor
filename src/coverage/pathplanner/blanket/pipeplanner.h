#ifndef INCLUDE_PATHPLANNER_BLANKET_PIPEPLANNER_H_
#define INCLUDE_PATHPLANNER_BLANKET_PIPEPLANNER_H_

#include <algorithm>
#include <cmath>
#include <vector>
#include "coverage/geometry/geoheader.h"

namespace coverage {

using geometry::Site;
using geometry::SiteVec;

const double kTurningRadius = 2;
const double kSampleDis = 0.1;
const double kCarWidth = 1.0;

struct FilterCurve {
  int cur_l_ind;
  int cur_r_ind;
  int l_ind;
  int r_ind;
  Site l_pt;
  Site r_pt;
  SiteVec curves;
};

class PipePlanner {
 public:
  PipePlanner();
  ~PipePlanner() = default;
  bool Interface(const std::vector<double> &x, const std::vector<double> &y,
                 const std::vector<double> &x_edge,
                 const std::vector<double> &y_edge);
  bool Planning();

 public:
  SiteVec final_path_;
  std::vector<SiteVec> delau_triangle_;
  std::vector<std::pair<int, std::vector<int>>> delau_triangle_type_;
  SiteVec delau_triangle_center_;
  std::vector<SiteVec> test_path_;
  // SiteVec final_path_;
  /* for skeleton line */
  std::vector<int> main_ind_;
  SiteVec main_pt_;
  std::vector<SiteVec> leftpt_pair_;
  std::vector<SiteVec> rightpt_pair_;
  double max_distance_;
  std::vector<SiteVec> left_segment_;
  std::vector<SiteVec> right_segment_;
  /* for skeleton line */

  /* for edge curve */
  std::vector<Site> start_curve_;
  std::vector<Site> end_curve_;
  std::vector<FilterCurve> start_filter_;
  std::vector<FilterCurve> end_filter_;
  /* for edge curve */
  /*for DFS*/
  std::vector<std::vector<int>> relation_map_;
  std::vector<int> visited_;
  std::vector<int> father_;
  int max_dep_;
  int last_ind_;
  /*for DFS*/

 private:
  bool EdgeThinning();
  bool EdgeEroding();
  int FindNearInd(const Site &pt);
  bool EdgeSizeValid(const std::vector<double> &x,
                     const std::vector<double> &y);
  bool EdgeReconstruct(const std::vector<double> &x,
                       const std::vector<double> &y);
  bool TriangleGeneration();
  bool SkeletonGeneration();
  bool IsInSide(const geometry::Site &pt, const std::vector<Site> &polygon);
  std::pair<int, std::vector<int>> GetPtType(const SiteVec &poly);
  bool IsCommonEdge(const std::vector<int> &p1, const std::vector<int> &p2,
                    std::vector<int> &common);
  void DeepFirstSearch(int ind, int dep);
  bool GetMainSkeleton(std::vector<int> &p);
  bool CentralPtFilter(const std::vector<int> ind);
  bool SkeletonDilation();
  bool GenerateEdgeCurve();
  bool GenerateFiltCurve(int index);
  bool GenerateFilterCurve();
  bool PathConnect();
  bool PathSmooth();
  void Recovery(geometry::SiteVec &path);
  bool LineSampling(const geometry::Site &start, const geometry::Site &end,
                    std::vector<geometry::Site> &sample);
  bool ArcSampling(const geometry::Site &cent, const geometry::Site &start,
                   const geometry::Site &end, const double cross,
                   std::vector<geometry::Site> &sample);

  // basic math tool
 private:
  Site Incenter(const Site &a, const Site &b, const Site &c);
  bool LeftOfLine(const Site &p, const Site &p1, const Site &p2);
  double GetDisFromPtToLine(const geometry::Site &pt, const geometry::Site &p1,
                            const geometry::Site &p2);
  bool SameDirection(const geometry::Site &p1, const geometry::Site &p2,
                     const geometry::Site &p3, const geometry::Site &p4);
  bool CheckCrossOfLines(const geometry::Site &p1, const geometry::Site &p2,
                         const geometry::Site &p3, const geometry::Site &p4);
  double Mult(const geometry::Site &p0, const geometry::Site &p1,
              const geometry::Site &p2);
  bool IfAutoclockwise(const SiteVec &poly);
  Site GetCrossPt(const geometry::Site &p1, const geometry::Site &p2,
                  const geometry::Site &p3, const geometry::Site &p4);
  double Mult(const geometry::Site &p1, const geometry::Site &p2);

 private:
  bool state_;
  double x_offset_;
  double y_offset_;
  SiteVec orig_edge_;
  SiteVec edge1_;
  SiteVec edge2_;
};

}  // namespace coverage

#endif  // INCLUDE_PATHPLANNER_BLANKET_PIPEPLANNER_H_
