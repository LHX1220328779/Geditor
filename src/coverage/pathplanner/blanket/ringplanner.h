#ifndef INCLUDE_PATHPLANNER_BLANKET_RINGPLANNER_H_
#define INCLUDE_PATHPLANNER_BLANKET_RINGPLANNER_H_

#include <vector>
#include "coverage/geometry/geoheader.h"

namespace coverage {

// the polygons list(each polygon consist of some points)
using polygonlist = std::vector<std::vector<geometry::Site>>;
// a polygon made of several points
using polygonsite = std::vector<geometry::Site>;
// the edge list
using edgelist = std::vector<geometry::Site>;

const double carwidth = 1.0;
const double turning_radius = 2.0;
const double sample_dis = 0.1;
const bool debug_mode = true;

class RingPlanner {
 public:
  RingPlanner() = default;
  ~RingPlanner() = default;
  bool Interface(const double &sx, const double &sy, const double &syaw,
                 const double &dx, const double &dy, const double &dyaw,
                 const std::vector<double> &x, const std::vector<double> &y);
  /**
   * @brief : Normally planneing, generate a coverage path
   * @param : none
   * @return: none
   **/
  bool Planning();

 private:
  bool ReconstructPoly(const polygonlist &orig_poly, polygonlist &poly);
  bool AverageSimpling(geometry::SiteVec &sample_point,
                       const geometry::SiteVec &final_result);

  bool GenerateCurve(const geometry::SiteVec &source_data,
                     geometry::SiteVec &return_data);

  void Recovery(geometry::SiteVec &path);

  bool ConvertPoly(polygonlist &erode_poly);

  bool Out2Inner(polygonlist &erode_poly, polygonlist &dilate_poly,
                 polygonlist &connect);

  bool EdgeSizeValid(const std::vector<double> &x,
                     const std::vector<double> &y);
  bool PointValid();
  /**
   * @brief : check if the point is in the polygon
   * @param : pt -- the point to be checked
   *          polygon -- the polygon based of several points in order
   * @return: true if th point is inside of the polygon,
   *          return false otherwise
   **/
  bool IsInSide(const geometry::Site &pt, const polygonsite &polygon);
  /**
   * @brief : convert the points list into 2 polygons
   * @param : none
   * @return: none
   **/
  bool Edges2Circles();

  void FindNeighPt(std::vector<int> &neigh);

  bool CircleClassification(const polygonsite &edge1, const polygonsite &edge2);

  bool IfAutoclockwise(const polygonsite &poly);

  void PolygonEroding(polygonlist &poly);

  void PolygonDilating(const polygonsite &inner_poly, polygonlist &poly);

  void CurveConnecting(const polygonlist &polys);
  bool PtInterpolation();
  bool CurveSmoothing();
  bool SecondaryFilt();

  bool LineSampling(const geometry::Site &start, const geometry::Site &end,
                    std::vector<geometry::Site> &sample);

  bool ArcSampling(const geometry::Site &cent, const geometry::Site &start,
                   const geometry::Site &end, const double cross,
                   std::vector<geometry::Site> &sample);
  int CheckInner(const polygonsite &poly, const polygonsite &base,
                 std::vector<bool> &inner);
  int CheckOuter(const polygonsite &poly, const polygonsite &base,
                 std::vector<bool> &outer);

  int CheckCollision(const polygonsite &poly, const polygonsite &base,
                     std::vector<bool> &collision);

  bool Eroding(const polygonsite &poly, polygonsite &new_poly,
               std::vector<bool> &collision);

  bool Eroding(const polygonsite &poly, polygonsite &new_poly);

  bool Dilating(const polygonsite &poly, polygonsite &new_poly);

  bool Dilating(const polygonsite &poly, polygonsite &new_poly,
                std::vector<bool> &collision);

 private:
  // basic function
  bool CheckLeftOfEdge(const geometry::Site &p, const geometry::Site &p1,
                       const geometry::Site &p2);
  bool CheckCrossOfLines(const geometry::Site &p1, const geometry::Site &p2,
                         const geometry::Site &p3, const geometry::Site &p4);
  /**
   * @brief : get the cross product from p0p1 to p0p2
   * @param : p0 -- the common point
   *          p1 -- the first point
   *          p2 -- the second point
   * @return: the value of the cross product
   **/
  double Mult(const geometry::Site &p0, const geometry::Site &p1,
              const geometry::Site &p2);
  double Mult(const geometry::Site &p1, const geometry::Site &p2);
  geometry::Site GetCrossPt(const geometry::Site &p1, const geometry::Site &p2,
                            const geometry::Site &p3, const geometry::Site &p4);
  geometry::Site GetCrossPt(const geometry::Site &p1, const geometry::Site &p2,
                            const geometry::Site &p3, const geometry::Site &p4,
                            bool straight);
  double GetDisFromPtToLine(const geometry::Site &pt, const geometry::Site &p1,
                            const geometry::Site &p2);
  double GetNearestDis(const geometry::Site &pt, const polygonsite &poly);

 public:
  std::vector<geometry::Site> final_result_;
  std::vector<geometry::Site> final_polygon_;
  std::vector<geometry::Site> final_path_;
  std::vector<geometry::Site> outer_edge_;
  std::vector<geometry::Site> inner_edge_;
  std::vector<geometry::Site> orig_outer_edge_;
  std::vector<geometry::Site> orig_inner_edge_;
  std::vector<geometry::Site> test_poly_;
  std::vector<geometry::Site> test_poly_filter_;
  std::vector<geometry::Site> test_poly_interpolation_;
  // geometry::SiteVec main_pts;
  geometry::SiteVec test_circle;
  double x_offset_;
  double y_offset_;

 private:
  std::vector<double> x_edge_;
  std::vector<double> y_edge_;
  std::vector<geometry::Site> orig_edge_;
  geometry::Site start_;
  geometry::Site end_;
};

}  // namespace coverage

#endif  // INCLUDE_PATHPLANNER_BLANKET_RINGPLANNER_H_
