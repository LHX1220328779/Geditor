#ifndef INCLUDE_PATHPLANNER_BLANKET_HYBRIDRING_H_
#define INCLUDE_PATHPLANNER_BLANKET_HYBRIDRING_H_

#include <algorithm>
#include <cmath>
#include <vector>
#include "coverage/geometry/geoheader.h"
#include "coverage/math/cgl.h"

namespace coverage {

using geometry::Site;
using geometry::SiteVec;

struct TriaType {
  int type;
  std::vector<int> ind;  // represent the index of the pipe
  std::vector<int> relation;
};

struct PipeSet {
  int state;
  int start;
  int end;
  SiteVec pipe;
  std::vector<int> inds;
};

class HybridRing {
 public:
  HybridRing();
  ~HybridRing() = default;
  bool Interface(const std::vector<double> &x, const std::vector<double> &y);
  bool Interface(const std::vector<double> &x1, const std::vector<double> &y1,
                 const std::vector<double> &x2, const std::vector<double> &y2);
  bool Planning(int type = 2);

 public:
  bool GetOrigInnerRing(SiteVec &poly);
  bool GetOrigOuterRing(SiteVec &poly);
  bool GetInnerRing(SiteVec &poly);
  bool GetOuterRing(SiteVec &poly);
  bool GetExtRing(SiteVec &poly);
  bool GetTriangle(std::vector<SiteVec> &poly);
  bool GetRingpoly(SiteVec &poly);
  bool GetPipepoly();

 private:
  bool GetOffset(const std::vector<double> &x, const std::vector<double> &y);
  bool Edges2Circle();
  void FindNeighPt(std::vector<int> &neigh);
  bool CircleClassification(const SiteVec &edge1, const SiteVec &edge2);
  bool PolyOffset(double offsetdis, const SiteVec &orig, SiteVec &poly,
                  bool direction, std::vector<int> &match);
  bool RingOffset(SiteVec &path);
  bool PathOptimizing(SiteVec &poly);
  bool PathClipper(const SiteVec &poly, SiteVec &result, const double offset);
  bool PolygonCollision(const SiteVec &poly1, const SiteVec &poly2);
  bool FindNearOffset();
  bool PolyMatching(const std::vector<int> &match,
                    std::vector<std::vector<int>> &result);
  bool PolyDelaunay(const std::vector<std::vector<int>> &match,
                    std::vector<std::pair<bool, SiteVec>> &result);
  bool SkeletonDilation();
  bool PipeOffset();
  bool ReconstructPipe(const SiteVec &outeredge, const SiteVec &inneredge,
                       SiteVec &pipe, std::vector<int> &inds);
  bool TriangleGeneration(const SiteVec &poly, std::vector<SiteVec> &tria,
                          std::vector<TriaType> &triatype);
  TriaType GetTriaType(const SiteVec &tria, const SiteVec &poly);
  bool GetRelationMap(const std::vector<TriaType> &type);
  bool IsCommonEdge(const std::vector<int> &p1, const std::vector<int> &p2,
                    std::vector<int> &common);
  bool GetMainSkeleton(std::vector<int> &p);
  void DeepFirstSearch(int ind, int dep);
  bool CenterPtFilter(const std::vector<int> &inds, double &max_dis);
  bool PipePtDilation(const double dis);
  bool SingleDilation(int ind, int seg, const std::vector<SiteVec> &pts,
                      SiteVec &edge);
  // bool PipePtCheck();
  bool PipeWidthOffset();
  bool PipeEroding(const SiteVec &pipe, const std::vector<int> &inds,
                   double width, SiteVec &sub_poly);
  bool PipeWidthEroding(const SiteVec &pipe, const std::vector<int> &inds,
                        double width, SiteVec &sub_poly);
  int CheckCrossSeg2Inner(const Site &p1, const Site &p2, const int start,
                          const int end, int &ind, double &dis);
  bool PathConnection();
  bool LineSampling(const geometry::Site &start, const geometry::Site &end,
                    std::vector<geometry::Site> &sample);
  bool ArcSampling(const geometry::Site &cent, const geometry::Site &start,
                   const geometry::Site &end, const double cross,
                   std::vector<geometry::Site> &sample);
  bool ArcSampling(const geometry::Site &cent, const geometry::Site &start,
                   const geometry::Site &end, const double cross,
                   const double radius, std::vector<geometry::Site> &sample);
  bool BSplineOptiming(const SiteVec &poly);
  bool BSampling(const Site &start, const Site &end, const Site &next_direction,
                 const Site &last_direction, const std::vector<double> &param,
                 std::vector<Site> &sample);
  bool BLineSampling(const geometry::Site &start, const geometry::Site &end,
                     std::vector<geometry::Site> &sample);

 public:
  SiteVec final_path_;

 private:
  // math::cgl::BasicCgl cgllib_;
  bool status_;
  SiteVec orig_edge_;
  SiteVec orig_outer_edge_;
  SiteVec orig_inner_edge_;
  SiteVec outer_edge_;
  SiteVec inner_edge_;
  double x_offset_;
  double y_offset_;
  double nearoffset_;
  int nearest_ind_;
  int nearest_out_ind_;
  SiteVec ext_outer_edge_;
  std::vector<std::pair<bool, SiteVec>> triangles_;
  std::vector<SiteVec> ringpoly_;
  SiteVec pipe_;
  std::vector<int> inds_;
  std::vector<SiteVec> pipe_triangles_;
  std::vector<TriaType> triatype_;
  SiteVec pipe_main_pt_;
  std::vector<SiteVec> lpts_;
  std::vector<SiteVec> rpts_;
  std::vector<SiteVec> lsegs_;
  std::vector<SiteVec> rsegs_;
  // for dfs
  std::vector<std::vector<int>> relation_map_;
  std::vector<int> visited_;
  std::vector<int> father_;
  int max_dep_;
  int last_ind_;
  // for dfs
  std::vector<SiteVec> total_pipe_;
};

}  // namespace coverage

#endif  // INCLUDE_PATHPLANNER_BLANKET_HYBRIDRINGPLANNER_H_
