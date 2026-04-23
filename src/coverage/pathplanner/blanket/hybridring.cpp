#include "pathplanner/blanket/hybridring.h"

#include <iostream>
#include <limits>
#include <queue>
#include <stack>

#include "coverage/clipper/clipper.hpp"
#include "math/cgl.h"
#include "pathplanner/blanket/conststruct.h"
#include "pathplanner/blanket/delaunay.h"

namespace coverage {

HybridRing::HybridRing() { status_ = false; }

/*------------PUBLIC INTERFACE---------------------------*/
bool HybridRing::Interface(const std::vector<double> &x,
                           const std::vector<double> &y) {
  if (!GetOffset(x, y)) return false;
  if (!Edges2Circle()) return false;
  /* std::vector<int> match;
   if (!PolyOffset(kVehicleWidth,
                   orig_outer_edge_,
                   outer_edge_,
                   true,
                   match))
     return false;
   if (!PolyOffset(kVehicleWidth,
                   orig_inner_edge_,
                   inner_edge_,
                   false,
                   match))
     return false;*/
  status_ = true;
  return true;
}

bool HybridRing::Interface(const std::vector<double> &x1,
                           const std::vector<double> &y1,
                           const std::vector<double> &x2,
                           const std::vector<double> &y2) {
  if (x1.size() < 3 || y1.size() < 3 || x1.size() != y1.size()) return false;
  if (x2.size() < 3 || y2.size() < 3 || x2.size() != y2.size()) return false;
  auto x_min_it = std::min_element(std::begin(x1), std::end(x1));
  auto x_max_it = std::max_element(std::begin(x1), std::end(x1));
  auto y_min_it = std::min_element(std::begin(y1), std::end(y1));
  auto y_max_it = std::max_element(std::begin(y1), std::end(y1));
  double x_min = *x_min_it;
  double x_max = *x_max_it;
  double y_min = *y_min_it;
  double y_max = *y_max_it;
  x_offset_ = x_min - 3;
  y_offset_ = y_min - 3;
  for (std::size_t i = 0; i < x1.size(); i++) {
    geometry::Site temp;
    temp.x = x1[i] - x_offset_;
    temp.y = y1[i] - y_offset_;
    orig_outer_edge_.push_back(temp);
  }
  for (std::size_t i = 0; i < x2.size(); i++) {
    geometry::Site temp;
    temp.x = x2[i] - x_offset_;
    temp.y = y2[i] - y_offset_;
    orig_inner_edge_.push_back(temp);
  }
  status_ = true;
  return true;
}

bool HybridRing::Planning(int type) {
  if (false == status_) return false;
  SiteVec path;
  if (!RingOffset(path)) return false;
  // if (!PipeOffset()) return false;
  // if (!PipeWidthOffset()) return false;
  // if (!PathConnection()) return false;
  if (type == 1) {
    if (!PathOptimizing(path)) return false;
  } else if (type == 2) {
    if (!BSplineOptiming(path)) return false;
  }
  for (std::size_t i = 0; i < final_path_.size(); i++) {
    final_path_[i].x += x_offset_;
    final_path_[i].y += y_offset_;
  }
  return true;
}
/*------------PUBLIC INTERFACE---------------------------*/

/*------------PUBLIC TEST INTERFACE---------------------------*/
bool HybridRing::GetOrigInnerRing(SiteVec &poly) {
  poly.clear();
  poly.insert(poly.end(), orig_inner_edge_.begin(), orig_inner_edge_.end());
  return true;
}

bool HybridRing::GetOrigOuterRing(SiteVec &poly) {
  poly.clear();
  poly.insert(poly.end(), orig_outer_edge_.begin(), orig_outer_edge_.end());
  return true;
}

bool HybridRing::GetInnerRing(SiteVec &poly) {
  poly.clear();
  poly.insert(poly.end(), inner_edge_.begin(), inner_edge_.end());
  return true;
}

bool HybridRing::GetOuterRing(SiteVec &poly) {
  poly.clear();
  poly.insert(poly.end(), outer_edge_.begin(), outer_edge_.end());
  return true;
}

bool HybridRing::GetExtRing(SiteVec &poly) {
  poly.clear();
  poly.insert(poly.end(), ext_outer_edge_.begin(), ext_outer_edge_.end());
  return true;
}

bool HybridRing::GetRingpoly(SiteVec &poly) {
  poly.clear();
  // std::cout << "ring:" << ringpoly_.size() << std::endl;
  for (auto &p : ringpoly_) {
    // std::cout << p.size() << std::endl;
    poly.insert(poly.end(), p.begin(), p.end());
  }
  return true;
}

bool HybridRing::GetPipepoly() {
  // for (const auto &p : total_pipe_) {
  // std::cout << "total_pipe_:";
  // for (const auto &t : p) {
  //   std::cout << t.x << "," << t.y << ",";
  // }
  // std::cout << std::endl;
  // }
  return true;
}
/*------------PUBLIC TEST INTERFACE---------------------------*/

bool HybridRing::GetOffset(const std::vector<double> &x,
                           const std::vector<double> &y) {
  if (x.size() < 8 || y.size() < 8 || x.size() != y.size()) return false;
  auto x_min_it = std::min_element(std::begin(x), std::end(x));
  auto x_max_it = std::max_element(std::begin(x), std::end(x));
  auto y_min_it = std::min_element(std::begin(y), std::end(y));
  auto y_max_it = std::max_element(std::begin(y), std::end(y));
  double x_min = *x_min_it;
  double x_max = *x_max_it;
  double y_min = *y_min_it;
  double y_max = *y_max_it;
  x_offset_ = x_min - 3;
  y_offset_ = y_min - 3;
  for (std::size_t i = 0; i < x.size(); i++) {
    geometry::Site temp;
    temp.x = x[i] - x_offset_;
    temp.y = y[i] - y_offset_;
    orig_edge_.push_back(temp);
  }
  return true;
}

bool HybridRing::Edges2Circle() {
  std::vector<int> neigh_pt;
  FindNeighPt(neigh_pt);
  SiteVec edge1, edge2;
  if (neigh_pt[1] < neigh_pt[2]) {
    if (neigh_pt[0] != 0 || neigh_pt[3] != orig_edge_.size() - 1) return false;
    for (int i = neigh_pt[0]; i < neigh_pt[1]; i++) {
      edge1.push_back(orig_edge_[i]);
    }
    for (int i = neigh_pt[2]; i < neigh_pt[3]; i++) {
      edge2.push_back(orig_edge_[i]);
    }
  } else if (neigh_pt[1] > neigh_pt[3]) {
    for (int i = neigh_pt[2]; i < neigh_pt[3]; i++) {
      edge1.push_back(orig_edge_[i]);
    }
    for (int i = neigh_pt[1]; i < orig_edge_.size(); i++) {
      edge2.push_back(orig_edge_[i]);
    }
    for (int i = 0; i < neigh_pt[0]; i++) {
      edge2.push_back(orig_edge_[i]);
    }
  } else {
    return false;
  }
  if (!CircleClassification(edge1, edge2)) return false;
  // std::cout << "normal classify" << std::endl;
  return true;
}

void HybridRing::FindNeighPt(std::vector<int> &neigh) {
  double min_dis = std::numeric_limits<double>::max();
  int neigh_p1, neigh_p2, neigh_p3, neigh_p4;
  for (std::size_t i = 0; i < orig_edge_.size() - 1; ++i) {
    for (std::size_t j = i + 1; j < orig_edge_.size(); ++j) {
      double temp_dist = std::hypot(orig_edge_[i].x - orig_edge_[j].x,
                                    orig_edge_[i].y - orig_edge_[j].y);
      if (min_dis > temp_dist) {
        neigh_p1 = i;
        neigh_p2 = j;
        min_dis = temp_dist;
      }
    }
  }
  min_dis = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < orig_edge_.size() - 1; ++i) {
    for (std::size_t j = i + 1; j < orig_edge_.size(); ++j) {
      if (i == neigh_p1 || j == neigh_p2) continue;
      double temp_dist = std::hypot(orig_edge_[i].x - orig_edge_[j].x,
                                    orig_edge_[i].y - orig_edge_[j].y);
      if (min_dis > temp_dist) {
        neigh_p3 = i;
        neigh_p4 = j;
        min_dis = temp_dist;
      }
    }
  }
  if (neigh_p1 < neigh_p3) {
    neigh.push_back(neigh_p1);
    neigh.push_back(neigh_p2);
    neigh.push_back(neigh_p3);
    neigh.push_back(neigh_p4);
  } else {
    neigh.push_back(neigh_p3);
    neigh.push_back(neigh_p4);
    neigh.push_back(neigh_p1);
    neigh.push_back(neigh_p2);
  }
  return;
}

bool HybridRing::CircleClassification(const SiteVec &edge1,
                                      const SiteVec &edge2) {
  if (edge1.size() < 3 || edge2.size() < 3) return false;
  if (math::cgl::IsInside(edge1[0], edge2)) {
    // the edge1 is the inner edge, the edge2 is the outer edge
    for (const auto &p : edge2) {
      if (math::cgl::IsInside(p, edge1)) return false;
    }
    for (const auto &p : edge1) orig_inner_edge_.push_back(p);
    for (const auto &p : edge2) orig_outer_edge_.push_back(p);
  } else {
    // the edge2 is the inner edge, the edge1 is the outer edge
    for (const auto &p : edge2) {
      if (!math::cgl::IsInside(p, edge1)) return false;
    }
    for (const auto &p : edge2) orig_inner_edge_.push_back(p);
    for (const auto &p : edge1) orig_outer_edge_.push_back(p);
  }
  if (!math::cgl::IfAutoclockwise(orig_inner_edge_))
    std::reverse(orig_inner_edge_.begin(), orig_inner_edge_.end());
  if (!math::cgl::IfAutoclockwise(orig_outer_edge_)) {
    std::reverse(orig_outer_edge_.begin(), orig_outer_edge_.end());
  }
  return true;
}

bool HybridRing::PolyOffset(double offsetdis, const SiteVec &orig,
                            SiteVec &poly, bool direction,
                            std::vector<int> &match) {
  match.clear();
  if (orig.size() < 3) return false;
  // 1.edge set and nornalize it
  SiteVec edgeset, n_edgeset;
  int count = orig.size();
  for (int i = 0; i < count; i++) {
    int next = (i == count - 1) ? 0 : (i + 1);
    geometry::Site temp = orig[next] - orig[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  // 2.compute the eroding point
  SiteVec erodingpoly;
  std::vector<std::pair<Site, bool>> poly_flag;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : (i - 1);
    int end = i;
    double cross_product =
        math::cgl::CrossProduct(n_edgeset[start], n_edgeset[end]);
    Site eroding_pt;
    if (direction) {
      eroding_pt =
          (n_edgeset[end] - n_edgeset[start]) * offsetdis / cross_product +
          orig[end];
    } else {
      eroding_pt = orig[end] - (n_edgeset[end] - n_edgeset[start]) * offsetdis /
                                   cross_product;
    }
    erodingpoly.push_back(eroding_pt);
    poly_flag.push_back(std::make_pair(eroding_pt, false));
  }
  // 3. check if there is cross between the edges
  std::vector<std::vector<int>> cross_ind;
  for (int i = 0; i < count; i++) {
    std::vector<int> temp;
    cross_ind.push_back(temp);
    int next_i = i == count - 1 ? 0 : i + 1;
    cross_ind[i].push_back(next_i);
  }
  for (int i = 0; i < count; i++) {
    int before_i = i == 0 ? count - 1 : i - 1;
    int next_i = i == count - 1 ? 0 : i + 1;
    for (int j = i + 1; j < count; j++) {
      if (j == before_i || j == next_i) continue;
      int next_j = j == count - 1 ? 0 : j + 1;
      if (math::cgl::CheckCrossOfSegments(erodingpoly[i], erodingpoly[next_i],
                                          erodingpoly[j],
                                          erodingpoly[next_j])) {
        cross_ind[i].push_back(j);
        cross_ind[j].push_back(i);
      }
    }
  }
  SiteVec sub_polygon;
  std::vector<SiteVec> sub_polygons;
  std::vector<int> sub_ind;
  std::vector<std::vector<int>> sub_inds;
  int check_num = 0;
  int start_loop = 0;
  int end_loop = count;
  while (check_num < count) {
    for (int i = 0; i < count; i++) {
      if (poly_flag[i].second == true) {
        start_loop++;
      } else {
        break;
      }
    }
    int tmp = start_loop;
    SiteVec tmp_poly;
    tmp_poly.clear();
    sub_polygons.push_back(tmp_poly);
    std::vector<int> tmp_ind;
    sub_inds.push_back(tmp_ind);
    while (1) {
      if (poly_flag[tmp].second == false) {
        sub_polygons.back().push_back(erodingpoly[tmp]);
        sub_inds.back().push_back(tmp);
        poly_flag[tmp].second = true;
        check_num++;
      }
      if (cross_ind[tmp].size() == 1) {
        end_loop = cross_ind[tmp][0];
      } else {
        end_loop = cross_ind[tmp][1];
        int next_tmp = tmp == count - 1 ? 0 : tmp + 1;
        int next_end = end_loop == count - 1 ? 0 : end_loop + 1;
        Site cross_pt;
        math::cgl::GetCrossPtOfLines(erodingpoly[tmp], erodingpoly[next_tmp],
                                     erodingpoly[end_loop],
                                     erodingpoly[next_end], cross_pt);
        sub_polygons.back().push_back(cross_pt);
        sub_inds.back().push_back(1000);
        end_loop = next_end;
      }
      if (end_loop == start_loop) {
        break;
      } else {
        tmp = end_loop;
      }
    }
  }
  // 4. select an optimal polygon
  int max_size = std::numeric_limits<int>::min();
  int select_index = 0;
  int correct_num = 0;
  for (std::size_t i = 0; i < sub_polygons.size(); i++) {
    if (math::cgl::IfAutoclockwise(sub_polygons[i])) {
      correct_num++;
      int cur_size = sub_polygons[i].size();
      if (max_size < cur_size) {
        select_index = i;
        max_size = sub_polygons[i].size();
      }
    }
  }
  if (max_size < 3) return false;
  poly.clear();
  poly.insert(poly.end(), sub_polygons[select_index].begin(),
              sub_polygons[select_index].end());
  match.insert(match.end(), sub_inds[select_index].begin(),
               sub_inds[select_index].end());
  return true;
}

bool HybridRing::RingOffset(SiteVec &path) {
  /*FindNearOffset();
  std::vector<int> match;
  if (!PolyOffset(nearoffset_,
                  outer_edge_,
                  ext_outer_edge_,
                  true,
                  match))
    return false;
  std::vector<std::vector<int>> result;
  if (!PolyMatching(match, result)) return false;
  if (!PolyDelaunay(result, triangles_)) return false;
  SkeletonDilation();
  return true;*/

  int loop = 0;
  double offset;
  SiteVec orig_polygon, next_polygon;
  orig_polygon.insert(orig_polygon.end(), orig_outer_edge_.begin(),
                      orig_outer_edge_.end());
  while (1) {
    if (loop == 0) {
      offset = 0.8;
    } else {
      offset = 1.0;
    }
    if (!PathClipper(orig_polygon, next_polygon, offset)) break;
    if (PolygonCollision(next_polygon, orig_inner_edge_)) break;

    orig_polygon.clear();
    orig_polygon.insert(orig_polygon.end(), next_polygon.begin(),
                        next_polygon.end());
    path.insert(path.end(), next_polygon.begin(), next_polygon.end());
    loop++;
  }
  return true;
}

bool HybridRing::PathOptimizing(SiteVec &poly) {
  if (poly.size() < 3) return false;
  for (int i = 1; i < poly.size() - 1; i++) {
    // std::cout<< __LINE__ << "," << i << std::endl;
    double l_dis;
    if (i == 1) {
      l_dis = (poly[i] - poly[i - 1]).mold();
    } else {
      l_dis = (poly[i] - final_path_.back()).mold();
    }
    double r_dis = (poly[i + 1] - poly[i]).mold();
    Site l_unit = (poly[i] - poly[i - 1]).direction();
    Site r_unit = (poly[i + 1] - poly[i]).direction();
    double cross = math::cgl::CrossProduct(l_unit, r_unit);
    double n_dot = math::cgl::DotProduct(l_unit, r_unit);
    double smooth_dis = std::fabs(kMinRadius / cross * (1.0 - n_dot));
    if (smooth_dis >= l_dis || smooth_dis >= r_dis) {
      SiteVec new_path;
      if (i == 1) {
        LineSampling(poly[0], poly[i], new_path);
      } else {
        LineSampling(final_path_.back(), poly[i], new_path);
      }
      final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
    } else {
      Site start_i, end_i, cent_i;
      if (std::fabs(cross) < 0.1) {
        SiteVec new_path;
        if (i == 1) {
          LineSampling(poly[0], poly[i], new_path);
        } else {
          LineSampling(final_path_.back(), poly[i], new_path);
        }
        final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
      } else if (cross > 0.0) {
        SiteVec new_path;
        start_i = poly[i] - l_unit * kMinRadius / cross * (1.0 - n_dot);
        end_i = poly[i] + r_unit * kMinRadius / cross * (1.0 - n_dot);
        cent_i = poly[i] + (r_unit - l_unit) * kMinRadius / cross;
        if (i == 1) {
          LineSampling(poly[0], start_i, new_path);
        } else {
          LineSampling(final_path_.back(), start_i, new_path);
        }
        ArcSampling(cent_i, start_i, end_i, cross, new_path);
        final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
      } else {
        SiteVec new_path;
        start_i = poly[i] + l_unit * kMinRadius / cross * (1.0 - n_dot);
        end_i = poly[i] - r_unit * kMinRadius / cross * (1.0 - n_dot);
        cent_i = poly[i] - (r_unit - l_unit) * kMinRadius / cross;
        if (i == 1) {
          LineSampling(poly[0], start_i, new_path);
        } else {
          LineSampling(final_path_.back(), start_i, new_path);
        }
        ArcSampling(cent_i, start_i, end_i, cross, new_path);
        final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
      }
    }
  }
  return true;
}

bool HybridRing::PathClipper(const SiteVec &poly, SiteVec &result,
                             const double offset) {
  const double scale = 100;
  const double inv_scale = 1.0 / scale;
  ClipperLib::ClipperOffset co(2.0);
  ClipperLib::Path path;
  ClipperLib::Paths solution;
  for (auto &pt : poly) {
    path.push_back(ClipperLib::IntPoint(pt.x * scale, pt.y * scale));
  }
  co.AddPath(path, ClipperLib::JoinType::jtMiter,
             ClipperLib::EndType::etClosedPolygon);
  co.Execute(solution, -offset * scale);
  // if there are >1 polygons, return false
  if (solution.size() != 1) return false;
  // remove the too small polygons
  if (ClipperLib::Area(solution[0]) < 10.0 * scale * scale) return false;
  result.clear();
  // fill result and remove too close points
  Site last_pt;
  double closest_pt_th = 1.2;
  for (auto &p : solution[0]) {
    Site cur_pt = Site(p.X * inv_scale, p.Y * inv_scale);
    if (result.empty()) {
      result.push_back(cur_pt);
      last_pt = cur_pt;
    } else if ((last_pt - cur_pt).mold() > closest_pt_th) {
      result.push_back(cur_pt);
      last_pt = cur_pt;
    }
  }
  return true;
}

bool HybridRing::PolygonCollision(const SiteVec &poly1, const SiteVec &poly2) {
  if (poly1.size() < 3 || poly2.size() < 3) return true;
  int next_i;
  int next_j;
  for (int i = 0; i < poly1.size(); ++i) {
    next_i = i == poly1.size() - 1 ? 0 : i + 1;
    if (i >= poly1.size() || next_i >= poly1.size()) {
      std::cout << i << "," << next_i << std::endl;
    }
    for (int j = 0; j < poly2.size(); ++j) {
      next_j = j == poly2.size() - 1 ? 0 : j + 1;
      if (next_j >= poly2.size() || j >= poly2.size()) {
        std::cout << j << "," << next_j << std::endl;
      }
      if (math::cgl::CheckCrossOfSegments(poly1[i], poly1[next_i], poly2[j],
                                          poly2[next_j]))
        return true;
    }
  }
  return false;
}

bool HybridRing::FindNearOffset() {
  double min_dis = std::numeric_limits<double>::max();
  int near_ind = 0;
  int near_out_ind = 0;
  int outer_size = outer_edge_.size();
  int inner_size = inner_edge_.size();
  for (int i = 0; i < inner_size; i++) {
    for (int j = 0; j < outer_size; j++) {
      int next_j = j == outer_size - 1 ? 0 : j + 1;
      double dis = math::cgl::GetDisFromPtToLine(inner_edge_[i], outer_edge_[j],
                                                 outer_edge_[next_j]);
      if (min_dis > dis) {
        min_dis = dis;
        near_ind = i;
        near_out_ind = next_j;
      }
    }
  }
  nearoffset_ = min_dis;
  nearest_ind_ = near_ind;
  nearest_out_ind_ = near_out_ind;
  // std::cout << "inner_size:" << inner_edge_.size() << std::endl;
  // std::cout << "outer_size:" << outer_edge_.size() << std::endl;
  for (int i = 0; i < near_ind; i++) {
    inner_edge_.push_back(inner_edge_[i]);
  }
  for (int i = 0; i < near_ind; i++) {
    inner_edge_.erase(inner_edge_.begin());
  }
  for (int i = 0; i < near_out_ind; i++) {
    outer_edge_.push_back(outer_edge_[i]);
  }
  for (int i = 0; i < near_out_ind; i++) {
    outer_edge_.erase(outer_edge_.begin());
  }
  nearest_ind_ = 0;
  nearest_out_ind_ = 0;
  // std::cout << "inner_size:" << inner_edge_.size() << std::endl;
  // std::cout << "outer_size:" << outer_edge_.size() << std::endl;
  // std::cout << nearest_ind_ << nearest_out_ind_ << std::endl;

  return true;
}

bool HybridRing::PolyMatching(const std::vector<int> &match,
                              std::vector<std::vector<int>> &result) {
  int ext_outer_size = ext_outer_edge_.size();
  int outer_size = outer_edge_.size();
  for (int i = 0; i < match.size(); i++) {
    // std::cout << "match" << i << ":" << match[i] << std::endl;
    std::vector<int> tmp;
    if (match[i] < 1000) {
      tmp.push_back(match[i]);
    } else {
      int before_i = i == 0 ? match.size() - 1 : i - 1;
      int next_i = i == match.size() - 1 ? 0 : i + 1;
      if (match[next_i] == 1000 || match[before_i] == 1000) return false;
      if (match[next_i] > match[before_i]) {
        for (int j = match[before_i] + 1; j < match[next_i]; j++) {
          tmp.push_back(j);
        }
      } else {
        for (int j = match[before_i] + 1; j < outer_edge_.size(); j++) {
          tmp.push_back(j);
        }
        for (int j = 0; j < match[next_i]; j++) {
          tmp.push_back(j);
        }
      }
    }
    result.push_back(tmp);
  }
  // for (const auto &p : result) {
  //   for (const auto &t : p) {
  //     std::cout << t << ",";
  //   }
  //   std::cout << std::endl;
  // }
  return true;
}

bool HybridRing::PolyDelaunay(const std::vector<std::vector<int>> &match,
                              std::vector<std::pair<bool, SiteVec>> &result) {
  result.clear();
  std::vector<std::pair<Site, Site>> tmp_result;
  for (int i = 0; i < match.size(); i++) {
    for (int j = 0; j < match[i].size(); j++) {
      tmp_result.push_back(
          std::make_pair(ext_outer_edge_[i], outer_edge_[match[i][j]]));
    }
  }
  for (int i = 0; i < tmp_result.size(); i++) {
    int next_i = i == tmp_result.size() - 1 ? 0 : i + 1;
    double dis = (tmp_result[i].first - tmp_result[next_i].first).mold();
    if (dis < 1e-3) {
      SiteVec tmp;
      tmp.push_back(tmp_result[i].first);
      tmp.push_back(tmp_result[i].second);
      tmp.push_back(tmp_result[next_i].second);
      result.push_back(std::make_pair(true, tmp));
    } else {
      SiteVec tmp1, tmp2;
      tmp1.push_back(tmp_result[i].first);
      tmp1.push_back(tmp_result[i].second);
      tmp1.push_back(tmp_result[next_i].second);
      tmp2.push_back(tmp_result[next_i].second);
      tmp2.push_back(tmp_result[i].first);
      tmp2.push_back(tmp_result[next_i].first);
      result.push_back(std::make_pair(true, tmp1));
      result.push_back(std::make_pair(false, tmp2));
    }
  }
  return true;
}

bool HybridRing::SkeletonDilation() {
  int seg_num = ceil(nearoffset_ / kVehicleWidth);
  for (int i = 0; i < seg_num; i++) {
    SiteVec tmp;
    ringpoly_.push_back(tmp);
    for (int j = 0; j < triangles_.size(); j++) {
      if (triangles_[j].first == true) {
        // std::cout << "triangle_size:" << triangles_[j].second.size() <<
        // std::endl;
        Site p1;
        p1 = triangles_[j].second[1] +
             (triangles_[j].second[0] - triangles_[j].second[1]) * i / seg_num;
        // Site p2 = triangles_[j].first[2] +
        //           (triangles_[j].first[0] - triangles_[j].first[2]) *
        //            i / seg_num;
        ringpoly_.back().push_back(p1);
        // tmp.push_back(p2);
      } else {
        // std::cout << "triangle_size:" << triangles_[j].second.size() <<
        // std::endl;

        Site p1;
        p1 = triangles_[j].second[0] +
             (triangles_[j].second[1] - triangles_[j].second[0]) * i / seg_num;
        ringpoly_.back().push_back(p1);
      }
    }
    // std::cout << "size:" << ringpoly_.back().size() << std::endl;
  }
  return true;
}

bool HybridRing::PipeOffset() {
  // std::cout << "total" << inner_edge_.size() << ","
  //           << ext_outer_edge_.size() << std::endl;
  ReconstructPipe(ext_outer_edge_, inner_edge_, pipe_, inds_);
  TriangleGeneration(pipe_, pipe_triangles_, triatype_);
  // std::cout << "------------------" << std::endl;
  // for (const auto &p : pipe_triangles_) {
  //   std::cout << p[0].x << "," << p[0].y << ","
  //             << p[1].x << "," << p[1].y << ","
  //             << p[2].x << "," << p[2].y << ","
  //             << p[0].x << "," << p[0].y << std::endl;
  // }
  GetRelationMap(triatype_);
  std::vector<int> main_ind;
  double max_dis;
  if (!GetMainSkeleton(main_ind)) return false;
  if (!CenterPtFilter(main_ind, max_dis)) return false;
  if (!PipePtDilation(max_dis)) return false;
  return true;
}

bool HybridRing::ReconstructPipe(const SiteVec &outeredge,
                                 const SiteVec &inneredge, SiteVec &pipe,
                                 std::vector<int> &inds) {
  int outer_adapter_ind = outeredge.size();
  int outer_num = outeredge.size();
  std::pair<int, int> outer_break_inds;
  for (int i = 0; i < outer_num; i++) {
    int next_i = i == outer_num - 1 ? 0 : i + 1;
    double dis = math::cgl::GetDisFromPtToLine(inneredge[nearest_ind_],
                                               outeredge[i], outeredge[next_i]);
    if (dis < 1e-3) {
      outer_break_inds.first = i;
      outer_break_inds.second = next_i;
      break;
    }
  }
  int inner_num = inneredge.size();
  pipe.clear();
  for (int i = nearest_ind_; i < inner_num; i++) {
    pipe.push_back(inneredge[i]);
  }
  for (int i = 0; i <= nearest_ind_; i++) {
    pipe.push_back(inneredge[i]);
  }
  inds.push_back(0);
  inds.push_back(pipe.size() - 1);
  inds.push_back(pipe.size());
  SiteVec outerpipe;
  // std::cout << "filt" << outer_break_inds.first
  //           << "," << outer_break_inds.second << std::endl;
  if (outer_break_inds.second == 0) {
    pipe.insert(pipe.end(), outeredge.rbegin(), outeredge.rend());
  } else {
    for (int i = outer_break_inds.second; i < outer_num; i++)
      outerpipe.push_back(outeredge[i]);
    for (int i = 0; i < outer_break_inds.first; i++)
      outerpipe.push_back(outeredge[i]);
    pipe.insert(pipe.end(), outerpipe.rbegin(), outerpipe.rend());
  }
  inds.push_back(pipe.size() - 1);
  return true;
}

bool HybridRing::TriangleGeneration(const SiteVec &poly,
                                    std::vector<SiteVec> &tria,
                                    std::vector<TriaType> &triatype) {
  std::vector<delaunay::Point<float>> points;
  for (const auto &p : poly) {
    delaunay::Point<float> tmp;
    tmp.x = p.x;
    tmp.y = p.y;
    points.push_back(tmp);
  }
  const auto triangulation = delaunay::triangulate(points);
  if (triangulation.triangles.size() < 2) return false;
  for (auto const &e : triangulation.triangles) {
    SiteVec temp;
    temp.push_back(Site(e.p0.x, e.p0.y));
    temp.push_back(Site(e.p1.x, e.p1.y));
    temp.push_back(Site(e.p2.x, e.p2.y));
    Site cent_pt = math::cgl::IncenterPt(temp[0], temp[1], temp[2]);
    if (!math::cgl::IsInside(cent_pt, poly)) continue;
    if (!math::cgl::IfAutoclockwise(temp))
      std::reverse(temp.begin(), temp.end());
    TriaType temptype = GetTriaType(temp, poly);
    tria.push_back(temp);
    triatype.push_back(temptype);
  }
  return true;
}

TriaType HybridRing::GetTriaType(const SiteVec &tria, const SiteVec &poly) {
  TriaType cur_type;
  int start_ind = -1;
  int poly_size = poly.size();
  std::vector<int> p;
  for (std::size_t t = 0; t < tria.size(); t++) {
    for (int i = 0; i < poly_size; i++) {
      if (std::fabs(tria[t].x - poly[i].x) < 1e-2 &&
          std::fabs(tria[t].y - poly[i].y) < 1e-2) {
        p.push_back(i);
        if (i == 0) start_ind = t;
        break;
      }
    }
  }
  if (start_ind > -1) {
    if (!(p[(start_ind + 1) % 3] == poly_size - 1 ||
          p[(start_ind + 2) % 3] == poly_size - 1)) {
      p[start_ind] = inner_edge_.size();
    }
  }
  cur_type.ind.insert(cur_type.ind.end(), p.begin(), p.end());
  int p0, p1, p2;
  p0 = 0;
  p1 = 0;
  p2 = 0;
  if (std::abs(p[0] - p[1]) == 1 || std::abs(p[0] - p[1]) == poly_size - 1) {
    p0 = 1;
  }
  if (std::abs(p[1] - p[2]) == 1 || std::abs(p[1] - p[2]) == poly_size - 1) {
    p1 = 1;
  }
  if (std::abs(p[2] - p[0]) == 1 || std::abs(p[2] - p[0]) == poly_size - 1) {
    p2 = 1;
  }
  cur_type.type = p0 + p1 + p2;
  cur_type.relation.push_back(p0);
  cur_type.relation.push_back(p1);
  cur_type.relation.push_back(p2);
  return cur_type;
}

bool HybridRing::GetRelationMap(const std::vector<TriaType> &type) {
  for (std::size_t i = 0; i < type.size(); i++) {
    std::vector<int> tmp;
    tmp.push_back(i);
    relation_map_.push_back(tmp);
  }
  for (std::size_t i = 0; i < type.size(); i++) {
    for (std::size_t j = 0; j < type.size(); j++) {
      if (i == j) continue;
      std::vector<int> common_edges;
      if (IsCommonEdge(type[i].ind, type[j].ind, common_edges)) {
        relation_map_[i].push_back(j);
      }
    }
  }
  return true;
}

bool HybridRing::IsCommonEdge(const std::vector<int> &p1,
                              const std::vector<int> &p2,
                              std::vector<int> &common) {
  common.clear();
  int t1, t2, t3, t4;
  bool common1 = false;
  bool common2 = false;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (p1[i] == p2[j]) {
        t1 = i;
        t3 = j;
        common1 = true;
        break;
      }
      if (common1) break;
    }
  }
  if (!common1) return false;
  for (int i = 0; i < 3; i++) {
    if (i == t1) continue;
    for (int j = 0; j < 3; j++) {
      if (j == t3) continue;
      if (p1[i] == p2[j]) {
        t2 = i;
        t4 = j;
        common2 = true;
        break;
      }
    }
    if (common2) break;
  }
  if (!common2) return false;
  common.push_back(t1);
  common.push_back(t2);
  common.push_back(t3);
  common.push_back(t4);
  return true;
}

bool HybridRing::GetMainSkeleton(std::vector<int> &p) {
  for (std::size_t i = 0; i < relation_map_.size(); i++) {
    visited_.push_back(0);
    father_.push_back(-1);
  }
  max_dep_ = -1;
  DeepFirstSearch(0, 0);
  for (std::size_t i = 0; i < relation_map_.size(); i++) {
    visited_[i] = 0;
    father_[i] = -1;
  }
  max_dep_ = -1;
  DeepFirstSearch(last_ind_, 0);
  int temp = last_ind_;
  while (temp != -1) {
    p.push_back(temp);
    temp = father_[temp];
  }
  if (p.size() < 2) return false;

  if (triatype_[p.front()].type != 2 || triatype_[p.back()].type != 2)
    return false;
  bool order = false;
  for (const auto &t : triatype_[p.front()].ind) {
    if (t == 0) {
      order = true;
      break;
    }
  }
  if (!order) std::reverse(p.begin(), p.end());
  return true;
}

void HybridRing::DeepFirstSearch(int ind, int dep) {
  visited_[ind] = 1;
  if (dep > max_dep_) {
    max_dep_ = dep;
    last_ind_ = ind;
  }
  for (int i = 1; i < relation_map_[ind].size(); i++) {
    if (visited_[relation_map_[ind][i]] == 0) {
      father_[relation_map_[ind][i]] = ind;
      DeepFirstSearch(relation_map_[ind][i], dep + 1);
    }
  }
  return;
}

bool HybridRing::CenterPtFilter(const std::vector<int> &ind, double &max_dis) {
  max_dis = 0.0;
  for (std::size_t i = 0; i < ind.size(); i++) {
    if (i == 0) {
      if (triatype_[ind[i]].type != 2) return false;
      int start_ind = 0;
      for (int j = 0; j < 3; j++) {
        if (triatype_[ind[i]].relation[j] == 0) {
          start_ind = (j + 2) % 3;
          break;
        }
      }
      bool check_result = false;
      if (triatype_[ind[i]].ind[start_ind] == inds_[0] ||
          triatype_[ind[i]].ind[start_ind] == inds_[3]) {
        check_result = true;
      }
      if (check_result) {
        pipe_main_pt_.push_back((pipe_[inds_[0]] + pipe_[inds_[3]]) * 0.5);
      } else {
        pipe_main_pt_.push_back((pipe_[inds_[1]] + pipe_[inds_[2]]) * 0.5);
      }
      Site middle = (pipe_[triatype_[ind[i]].ind[(start_ind + 1) % 3]] +
                     pipe_[triatype_[ind[i]].ind[(start_ind + 2) % 3]]) *
                    0.5;
      pipe_main_pt_.push_back(middle);
      SiteVec lpt, rpt;
      for (int j = 0; j < 3; j++) {
        if (math::cgl::CheckLeftOfEdge(
                pipe_triangles_[ind[i]][j],
                pipe_main_pt_[pipe_main_pt_.size() - 2],
                pipe_main_pt_[pipe_main_pt_.size() - 1])) {
          lpt.push_back(pipe_triangles_[ind[i]][j]);
        } else {
          rpt.push_back(pipe_triangles_[ind[i]][j]);
        }
      }
      lpts_.push_back(lpt);
      rpts_.push_back(rpt);
    } else if (i == ind.size() - 1) {
      if (triatype_[ind[i]].type != 2) return false;
      int start_ind = 0;
      for (int j = 0; j < 3; j++) {
        if (triatype_[ind[i]].relation[j] == 0) {
          start_ind = (j + 2) % 3;
          break;
        }
      }
      bool check_result = false;
      if (triatype_[ind[i]].ind[start_ind] == inds_[0] ||
          triatype_[ind[i]].ind[start_ind] == inds_[3]) {
        check_result = true;
      }
      pipe_main_pt_.push_back(pipe_main_pt_.back());
      if (check_result) {
        pipe_main_pt_.push_back((pipe_[inds_[0]] + pipe_[inds_[3]]) * 0.5);
      } else {
        pipe_main_pt_.push_back((pipe_[inds_[1]] + pipe_[inds_[2]]) * 0.5);
      }
      SiteVec lpt, rpt;
      for (int j = 0; j < 3; j++) {
        if (math::cgl::CheckLeftOfEdge(
                pipe_triangles_[ind[i]][j],
                pipe_main_pt_[pipe_main_pt_.size() - 2],
                pipe_main_pt_[pipe_main_pt_.size() - 1])) {
          lpt.push_back(pipe_triangles_[ind[i]][j]);
        } else {
          rpt.push_back(pipe_triangles_[ind[i]][j]);
        }
      }
      lpts_.push_back(lpt);
      rpts_.push_back(rpt);
    } else {
      if (triatype_[ind[i]].type > 1) return false;
      if (triatype_[ind[i]].type == 1) {
        // std::cout << "middle is 1" << std::endl;
        int end_ind = 0;
        for (int j = 0; j < 3; j++) {
          if (triatype_[ind[i]].relation[j] == 1) {
            end_ind = j;
            break;
          }
        }
        Site middle1 = (pipe_[triatype_[ind[i]].ind[(end_ind + 1) % 3]] +
                        pipe_[triatype_[ind[i]].ind[(end_ind + 2) % 3]]) *
                       0.5;
        Site middle2 = (pipe_[triatype_[ind[i]].ind[(end_ind) % 3]] +
                        pipe_[triatype_[ind[i]].ind[(end_ind + 2) % 3]]) *
                       0.5;
        pipe_main_pt_.push_back(pipe_main_pt_.back());
        if ((pipe_main_pt_.back() - middle1).mold() < 1e-1) {
          pipe_main_pt_.push_back(middle2);
        } else {
          pipe_main_pt_.push_back(middle1);
        }
      } else {
        Site middle1, middle2;
        std::vector<int> common;
        if (IsCommonEdge(triatype_[ind[i]].ind, triatype_[ind[i - 1]].ind,
                         common)) {
          middle1 = (pipe_[common[0]] + pipe_[common[1]]) * 0.5;
        } else {
          return false;
        }
        if (IsCommonEdge(triatype_[ind[i]].ind, triatype_[ind[i + 1]].ind,
                         common)) {
          middle2 = (pipe_[common[0]] + pipe_[common[1]]) * 0.5;
        } else {
          return false;
        }
        pipe_main_pt_.push_back(pipe_main_pt_.back());
        if ((pipe_main_pt_.back() - middle1).mold() < 1e-1) {
          pipe_main_pt_.push_back(middle2);
        } else {
          pipe_main_pt_.push_back(middle1);
        }
      }
      SiteVec lpt, rpt;
      for (int j = 0; j < 3; j++) {
        if (math::cgl::CheckLeftOfEdge(
                pipe_triangles_[ind[i]][j],
                pipe_main_pt_[pipe_main_pt_.size() - 2],
                pipe_main_pt_[pipe_main_pt_.size() - 1])) {
          lpt.push_back(pipe_triangles_[ind[i]][j]);
        } else {
          rpt.push_back(pipe_triangles_[ind[i]][j]);
        }
      }
      lpts_.push_back(lpt);
      rpts_.push_back(rpt);
    }
    double dis = math::cgl::GetDisFromPtToLine(
        pipe_triangles_[ind[i]][0], pipe_main_pt_[pipe_main_pt_.size() - 2],
        pipe_main_pt_[pipe_main_pt_.size() - 1]);
    if (max_dis < dis) max_dis = dis;
  }
  // std::cout << "get main pt" << std::endl;
  // for (const auto &p : pipe_main_pt_) {
  //   std::cout << p.x << "," << p.y << std::endl;
  // }
  // for (const auto &p : ind) {
  //   std::cout << p << std::endl;
  // }
  // std::cout << "get main pt" << std::endl;
  return true;
}

bool HybridRing::PipePtDilation(const double dis) {
  int seg = ceil(dis / kVehicleWidth);
  if (lpts_.size() != rpts_.size()) return false;
  if (seg < 2) return true;
  // std::cout << "seg" << lpts_.size() << std::endl;
  for (int ind = 1; ind < seg; ind++) {
    SiteVec l, r;
    if (!SingleDilation(ind, seg, lpts_, l)) return false;
    if (!SingleDilation(ind, seg, rpts_, r)) return false;
    lsegs_.push_back(l);
    rsegs_.push_back(r);
  }
  // std::cout << "pipe edge_size:" << lsegs_.size() << std::endl;
  // for (int i = 0; i < lsegs_.size(); i++) {
  //   for (int j = 0; j < lsegs_[i].size()/2; j++) {
  //     std::cout << lsegs_[i][j].x << "," << lsegs_[i][j].y << std::endl;
  //   }
  // }
  // for (int i = 0; i < rsegs_.size(); i++) {
  //   for (int j = 0; j < rsegs_[i].size(); j++) {
  //     std::cout << rsegs_[i][j].x << "," << rsegs_[i][j].y << std::endl;
  //   }
  // }
  // std::cout << "pipe edge_size:" << lsegs_.size() << std::endl;

  return true;
}

bool HybridRing::SingleDilation(int ind, int seg,
                                const std::vector<SiteVec> &pts,
                                SiteVec &edge) {
  for (std::size_t i = 0; i < pts.size(); i++) {
    if (pts[i].size() == 1) {
      Site p1 =
          (pts[i][0] - pipe_main_pt_[i * 2]) / seg * ind + pipe_main_pt_[i * 2];
      Site p2 = (pts[i][0] - pipe_main_pt_[i * 2 + 1]) / seg * ind +
                pipe_main_pt_[i * 2 + 1];
      edge.push_back(p1);
      edge.push_back(p2);
    } else if (pts[i].size() == 2) {
      Site angle1, angle2;
      angle1 = pipe_main_pt_[i * 2] - pipe_main_pt_[i * 2 + 1];
      angle2 = pts[i][0] - pts[i][1];
      // std::cout << "pipe_main_pt_0:" << pipe_main_pt_[i*2].x << ","
      //           << pipe_main_pt_[i*2].y << std::endl;
      // std::cout << "pipe_main_pt_1:" << pipe_main_pt_[i*2+1].x << ","
      //           << pipe_main_pt_[i*2+1].y << std::endl;
      // std::cout << "pts0:" << pts[i][0].x << "," << pts[i][0].y << std::endl;
      // std::cout << "pts11:" << pts[i][1].x << "," << pts[i][1].y <<
      // std::endl; std::cout << "angle0:" << angle1.x << "," << angle1.y <<
      // std::endl; std::cout << "angle0:" << angle2.x << "," << angle2.y <<
      // std::endl;
      if (math::cgl::SameDirection(angle1, angle2)) {
        Site p1 = (pts[i][0] - pipe_main_pt_[i * 2]) / seg * ind +
                  pipe_main_pt_[i * 2];
        Site p2 = (pts[i][1] - pipe_main_pt_[i * 2 + 1]) / seg * ind +
                  pipe_main_pt_[i * 2 + 1];
        edge.push_back(p1);
        edge.push_back(p2);
      } else {
        Site p1 = (pts[i][1] - pipe_main_pt_[i * 2]) / seg * ind +
                  pipe_main_pt_[i * 2];
        Site p2 = (pts[i][0] - pipe_main_pt_[i * 2 + 1]) / seg * ind +
                  pipe_main_pt_[i * 2 + 1];
        edge.push_back(p1);
        edge.push_back(p2);
      }
    } else {
      return false;
    }
  }
  return true;
}

bool HybridRing::PipeWidthOffset() {
  // SiteVec width_pipe;
  // std::vector<int> width_inds;
  // ReconstructPipe(ext_outer_edge_, inner_edge_, width_pipe, width_inds);
  // std::reverse(width_pipe.begin(), width_pipe.end());
  // for (std::size_t i = 0; i < width_inds.size(); i++) {
  //   width_inds[i] = width_pipe.size()-1-width_inds[i];
  // }
  // std::reverse(width_inds.begin(), width_inds.end());
  // std::vector<SiteVec> pipevec;
  // while (1) {
  //   std::cout << __LINE__ << std::endl;
  //   SiteVec erodepoly;
  //   if (!PipeEroding(width_pipe, width_inds, kVehicleWidth, erodepoly))
  //   return false; int aim_ind = -1; double aim_dis =
  //   std::numeric_limits<double>::min(); for (std::size_t i = 0; i  <
  //   erodepoly.size()-1; i++) {
  //     int ind;
  //     double dis;
  //     int result = math::cgl::CheckCrossSeg2Poly(erodepoly[i],
  //                                               erodepoly[i+1],
  //                                               inner_edge_,
  //                                               ind,
  //                                               dis);
  //     if (result < 0) return false;
  //     if (result == 0) continue;
  //     if (dis > kVehicleWidth) return false;
  //     if (aim_dis < dis) {
  //       aim_dis = dis;
  //       aim_ind = ind;
  //     }
  //   }
  //   std::cout << "enter this loop" << std::endl;
  //   if (aim_ind >= 0) {
  //     SiteVec finalpoly;
  //     std::cout << pipe_.size() << std::endl;
  //     if (!PipeEroding(width_pipe, width_inds,
  //                     kVehicleWidth-aim_dis, finalpoly)) return false;
  //     pipevec.push_back(finalpoly);
  //     break;
  //   } else {
  //     pipevec.push_back(erodepoly);
  //     for (const auto &p : erodepoly) {
  //       std::cout << "erodepoly" << p.x << "," << p.y << std::endl;
  //     }
  //     width_pipe.clear();
  //     width_inds.clear();
  //     ReconstructPipe(erodepoly, inner_edge_, width_pipe, width_inds);
  //     std::reverse(width_pipe.begin(), width_pipe.end());
  //     for (std::size_t i = 0; i < width_inds.size(); i++) {
  //       width_inds[i] = width_pipe.size()-1-width_inds[i];
  //     }
  //     std::reverse(width_inds.begin(), width_inds.end());
  //   }
  // }
  // for (const auto &p : pipevec) {
  //   for (const auto &t : p) {
  //     std::cout << "pipevec:" << t.x << "," << t.y << std::endl;
  //   }
  // }

  SiteVec width_pipe;
  std::vector<int> width_inds;
  ReconstructPipe(ext_outer_edge_, inner_edge_, width_pipe, width_inds);
  std::reverse(width_pipe.begin(), width_pipe.end());
  for (std::size_t i = 0; i < width_inds.size(); i++) {
    width_inds[i] = width_pipe.size() - 1 - width_inds[i];
  }
  std::reverse(width_inds.begin(), width_inds.end());
  if (nearest_ind_ != 0) {
    for (int i = 0; i < nearest_ind_; i++) {
      Site tmp = inner_edge_[i];
      inner_edge_.push_back(tmp);
    }
    auto itr = inner_edge_.begin();
    for (int i = 0; i < nearest_ind_; i++) {
      itr = std::next(itr);
    }
    inner_edge_.erase(inner_edge_.begin(), itr);
  }
  std::queue<PipeSet> pipe_queue;
  std::vector<SiteVec> ignore_tria;
  PipeSet start_pipe;
  start_pipe.state = 0;
  start_pipe.start = 0;
  start_pipe.end = inner_edge_.size();
  start_pipe.pipe.insert(start_pipe.pipe.end(), width_pipe.begin(),
                         width_pipe.end());
  start_pipe.inds.insert(start_pipe.inds.end(), width_inds.begin(),
                         width_inds.end());
  pipe_queue.push(start_pipe);
  while (!pipe_queue.empty()) {
    auto temp = pipe_queue.front();
    pipe_queue.pop();
    for (const auto &p : temp.pipe) {
      std::cout << "pop:" << p.x << "," << p.y << std::endl;
    }
    while (1) {
      SiteVec erodepoly;
      std::cout << temp.pipe.size() << std::endl;
      std::cout << temp.inds[0] << "," << temp.inds[1] << "," << temp.inds[2]
                << "," << temp.inds[3] << std::endl;
      if (!PipeEroding(temp.pipe, temp.inds, kVehicleWidth, erodepoly))
        return false;
      std::cout << "erode finish" << std::endl;
      int aim_ind = -1;
      int out_aim_ind = -1;
      double aim_dis = std::numeric_limits<double>::min();
      for (std::size_t i = 0; i < erodepoly.size() - 1; i++) {
        int ind;
        double dis;
        int result = CheckCrossSeg2Inner(erodepoly[i], erodepoly[i + 1],
                                         temp.start, temp.end, ind, dis);
        if (result < 0) return false;
        std::cout << "check result is:" << result << std::endl;
        if (result == 0) continue;
        if (dis > kVehicleWidth) return false;
        if (aim_dis < dis) {
          aim_dis = dis;
          aim_ind = ind;
          out_aim_ind = i;
        }
      }
      if (aim_ind >= 0) {
        // update state
        std::cout << "insect interruption" << aim_ind << std::endl;
        for (const auto &p : erodepoly) {
          std::cout << "the bugpoly is:" << p.x << "," << p.y << std::endl;
        }
        SiteVec finalpoly;
        std::vector<int> finalinds;
        std::cout << temp.pipe.size() << std::endl;
        std::cout << temp.inds[0] << "," << temp.inds[1] << "," << temp.inds[2]
                  << "," << temp.inds[3] << std::endl;
        if (!PipeEroding(temp.pipe, temp.inds, kVehicleWidth - aim_dis,
                         finalpoly))
          return false;
        if (finalpoly.size() != erodepoly.size()) {
          aim_ind = -1;
          out_aim_ind = -1;
          aim_dis = std::numeric_limits<double>::min();
          for (std::size_t i = 0; i < finalpoly.size() - 1; i++) {
            int ind;
            double dis;
            int result = CheckCrossSeg2Inner(finalpoly[i], finalpoly[i + 1],
                                             temp.start, temp.end, ind, dis);
            if (result < 0) return false;
            std::cout << "check result is:" << result << std::endl;
            if (result == 0) continue;
            if (dis > kVehicleWidth) return false;
            if (aim_dis < dis) {
              aim_dis = dis;
              aim_ind = ind;
              out_aim_ind = i;
            }
          }
        }

        for (const auto &p : finalpoly) {
          std::cout << "erodepoly" << p.x << "," << p.y << std::endl;
        }
        std::cout << "insect interruption completion" << finalpoly.size()
                  << std::endl;
        if (aim_ind == temp.start || aim_ind == temp.end - 1) {
          int new_state = -1;
          if (temp.end == inner_edge_.size()) {
            if (aim_ind == 0) {
              int out2end = erodepoly.size() - out_aim_ind;
              if (out2end > out_aim_ind) {
                new_state = 0;
              } else {
                new_state = 1;
              }
            } else if (aim_ind == inner_edge_.size() - 1) {
              new_state = 2;
            } else {
              new_state = 3;
            }
          } else {
            new_state = 4;
          }
          std::cout << __LINE__ << "," << new_state << std::endl;
          std::cout << temp.start << "," << temp.end << std::endl;
          std::cout << aim_ind << "," << out_aim_ind << std::endl;
          if (aim_ind == temp.start) finalpoly.erase(finalpoly.begin());
          if (aim_ind == temp.end - 1) finalpoly.pop_back();
          finalinds.push_back(0);
          finalinds.push_back(finalpoly.size() - 1);
          finalinds.push_back(finalinds.back() + 1);
          if (new_state < 2) {
            finalpoly.push_back(inner_edge_[0]);
          }
          // if (temp.start == 0 && temp.end == inner_edge_.size()) {
          //   finalpoly.push_back(inner_edge_[0]);
          // }
          for (int i = temp.end - 1; i >= temp.start; i--) {
            finalpoly.push_back(inner_edge_[i]);
          }
          finalinds.push_back(finalpoly.size() - 1);
          PipeSet newpipe;
          newpipe.state = new_state;
          newpipe.start = temp.start;
          newpipe.end = temp.end;
          newpipe.pipe.insert(newpipe.pipe.end(), finalpoly.begin(),
                              finalpoly.end());
          newpipe.inds.insert(newpipe.inds.end(), finalinds.begin(),
                              finalinds.end());
          if (newpipe.pipe.size() > 3) {
            pipe_queue.push(newpipe);
            for (const auto &p : newpipe.pipe) {
              std::cout << "ok_poly:" << p.x << "," << p.y << std::endl;
            }
            SiteVec test;
            for (int i = newpipe.inds[0]; i <= newpipe.inds[1]; i++) {
              test.push_back(newpipe.pipe[i]);
            }
            total_pipe_.push_back(test);
          }
        } else {
          std::cout << __LINE__ << "," << temp.state << std::endl;
          std::cout << temp.start << "," << temp.end << std::endl;
          std::cout << aim_ind << "," << out_aim_ind << std::endl;
          SiteVec leftpoly, rightpoly;
          std::vector<int> leftind, rightind;
          for (int i = 0; i <= out_aim_ind; i++) {
            leftpoly.push_back(finalpoly[i]);
          }
          leftind.push_back(0);
          leftind.push_back(leftpoly.size() - 1);
          leftind.push_back(leftind.back() + 1);
          for (int i = aim_ind; i >= temp.start; i--) {
            leftpoly.push_back(inner_edge_[i]);
          }
          leftind.push_back(leftpoly.size() - 1);
          for (int i = out_aim_ind + 1; i < finalpoly.size(); i++) {
            rightpoly.push_back(finalpoly[i]);
          }
          rightind.push_back(0);
          rightind.push_back(rightpoly.size() - 1);
          rightind.push_back(rightind.back() + 1);
          if (temp.state < 2) {
            rightpoly.push_back(inner_edge_[0]);
          }
          // if (temp.end == inner_edge_.size()) {
          //   rightpoly.push_back(inner_edge_[0]);
          // }
          for (int i = temp.end - 1; i >= aim_ind; i--) {
            rightpoly.push_back(inner_edge_[i]);
          }
          rightind.push_back(rightpoly.size() - 1);
          if (leftpoly.size() > 3) {
            PipeSet newpipe;
            newpipe.state = 4;
            newpipe.start = temp.start;
            newpipe.end = aim_ind + 1;
            newpipe.pipe.insert(newpipe.pipe.end(), leftpoly.begin(),
                                leftpoly.end());
            newpipe.inds.insert(newpipe.inds.end(), leftind.begin(),
                                leftind.end());
            for (const auto &p : newpipe.pipe) {
              std::cout << "left_poly:" << p.x << "," << p.y << std::endl;
            }
            pipe_queue.push(newpipe);
            // total_pipe_.push_back(newpipe.pipe);
            SiteVec test;
            for (int i = newpipe.inds[0]; i <= newpipe.inds[1]; i++) {
              test.push_back(newpipe.pipe[i]);
            }
            total_pipe_.push_back(test);
          }
          if (rightpoly.size() > 3) {
            PipeSet newpipe;
            newpipe.state = temp.state;
            newpipe.start = aim_ind;
            newpipe.end = temp.end;
            newpipe.pipe.insert(newpipe.pipe.end(), rightpoly.begin(),
                                rightpoly.end());
            newpipe.inds.insert(newpipe.inds.end(), rightind.begin(),
                                rightind.end());
            pipe_queue.push(newpipe);
            // total_pipe_.push_back(newpipe.pipe);
            for (const auto &p : newpipe.pipe) {
              std::cout << "right_poly:" << p.x << "," << p.y << std::endl;
            }
            SiteVec test;
            for (int i = newpipe.inds[0]; i <= newpipe.inds[1]; i++) {
              test.push_back(newpipe.pipe[i]);
            }
            total_pipe_.push_back(test);
          }
        }
        break;
      } else {
        std::cout << "normal eroding" << temp.start << "," << temp.end
                  << std::endl;
        temp.pipe.clear();
        temp.pipe.insert(temp.pipe.end(), erodepoly.begin(), erodepoly.end());
        // if (temp.end == inner_edge_.size()) {
        //   temp.pipe.push_back(inner_edge_[0]);
        // }
        if (temp.state < 2) {
          temp.pipe.push_back(inner_edge_[0]);
        }
        for (int i = temp.end - 1; i >= temp.start; i--) {
          temp.pipe.push_back(inner_edge_[i]);
        }
        // else {
        //   for (int i = temp.end; i >= temp.start; i--) {
        //     temp.pipe.push_back(inner_edge_[i]);
        //   }
        // }
        // total_pipe_.push_back(temp.pipe);
        SiteVec test;
        for (int i = temp.inds[0]; i <= temp.inds[1]; i++) {
          test.push_back(temp.pipe[i]);
        }
        total_pipe_.push_back(test);
        std::cout << "normal size:" << temp.pipe.size() << std::endl;
        for (const auto &p : erodepoly) {
          std::cout << "erodepoly" << p.x << "," << p.y << std::endl;
        }
      }
    }
  }
  return true;
}

bool HybridRing::PipeEroding(const SiteVec &pipe, const std::vector<int> &inds,
                             double width, SiteVec &sub_poly) {
  SiteVec n_edgeset;
  std::cout << "ind:" << inds[0] << "," << inds[1] << "," << pipe.size()
            << std::endl;
  for (int i = inds[0]; i < inds[1]; i++) {
    Site temp = pipe[i + 1] - pipe[i];
    n_edgeset.push_back(temp.direction());
  }
  SiteVec erodingpoly;
  std::vector<std::pair<Site, bool>> poly_flag;
  if (inds[1] > inds[0] + 1) {
    for (int i = inds[0] + 1; i < inds[1]; i++) {
      int start = i - 1;
      int end = i;
      double cross = math::cgl::CrossProduct(n_edgeset[start], n_edgeset[end]);
      Site eroding_pt;
      eroding_pt =
          (n_edgeset[end] - n_edgeset[start]) * width / cross + pipe[end];
      erodingpoly.push_back(eroding_pt);
      poly_flag.push_back(std::make_pair(eroding_pt, false));
    }
    if (erodingpoly.size() < 1) return false;
    // get the first eroding pt and the end eroding pt
    Site p_front, p_back;
    // std::cout << pipe[inds[1]].x << "," << pipe[inds[1]].y << std::endl;
    // std::cout << pipe[inds[2]].x << "," << pipe[inds[2]].y << std::endl;
    // std::cout << n_edgeset.back().x << "," << n_edgeset.back().y <<
    // std::endl; std::cout << "well done" << std::endl;
    if (!math::cgl::GetCrossPtOfLines(erodingpoly.front() + n_edgeset.front(),
                                      erodingpoly.front(), pipe[inds[0]],
                                      pipe[inds[3]], p_front)) {
      std::cout << __LINE__ << std::endl;
      return false;
    }
    if (!math::cgl::GetCrossPtOfLines(erodingpoly.back() + n_edgeset.back(),
                                      erodingpoly.back(), pipe[inds[1]],
                                      pipe[inds[2]], p_back)) {
      std::cout << __LINE__ << std::endl;
      return false;
    }
    erodingpoly.insert(erodingpoly.begin(), p_front);
    poly_flag.insert(poly_flag.begin(), std::make_pair(p_front, false));
    erodingpoly.insert(erodingpoly.end(), p_back);
    poly_flag.insert(poly_flag.end(), std::make_pair(p_back, false));
  } else {
    Site direction = (pipe[inds[1]] - pipe[inds[0]]).direction();
    Site left_d(-direction.y, direction.x);
    Site l1 = pipe[inds[0]] + left_d * width;
    Site l2 = pipe[inds[1]] + left_d * width;
    sub_poly.push_back(l1);
    sub_poly.push_back(l2);
    return true;
  }

  for (const auto &p : erodingpoly) {
    std::cout << "normal erodingpoly:" << p.x << "," << p.y << std::endl;
  }
  std::vector<std::vector<int>> cross_ind;
  for (int i = 0; i < erodingpoly.size(); i++) {
    std::vector<int> temp;
    temp.push_back(i);
    cross_ind.push_back(temp);
  }
  for (int i = 0; i < erodingpoly.size() - 1; i++) {
    for (int j = i + 1; j < erodingpoly.size() - 1; j++) {
      if (math::cgl::CheckCrossOfSegments(erodingpoly[i], erodingpoly[i + 1],
                                          erodingpoly[j], erodingpoly[j + 1])) {
        cross_ind[i].push_back(j);
        cross_ind[j].push_back(i);
      }
    }
  }
  for (std::size_t i = 0; i < cross_ind.size(); i++) {
    std::cout << "cross:";
    for (std::size_t j = 0; j < cross_ind[i].size(); j++) {
      std::cout << cross_ind[i][j] << ",";
    }
    std::cout << std::endl;
  }
  // SiteVec sub_poly;
  std::vector<int> sub_ind;
  std::vector<int> ignore_flag;
  int start_i = 0;
  int step_count = 0;
  ignore_flag.push_back(0);
  while (start_i < erodingpoly.size() - 1) {
    sub_ind.push_back(start_i);
    step_count++;
    for (const auto &p : sub_ind) {
      std::cout << "sub_ind:" << p << std::endl;
    }
    for (const auto &p : ignore_flag) {
      std::cout << "sub_ignore:" << p << std::endl;
    }
    if (start_i == 0) {
      if (cross_ind[start_i].size() <= 2) {
        start_i++;
        if (start_i == erodingpoly.size() - 2) {
          ignore_flag.push_back(0);
          break;
        }
        if (!math::cgl::SameDirection(
                erodingpoly[start_i] - erodingpoly[start_i + 1],
                pipe[start_i] - pipe[start_i + 1])) {
          for (const auto &p : erodingpoly) {
            std::cout << "[diff direction]" << start_i << "," << p.x << ","
                      << p.y << std::endl;
          }
          std::cout << __LINE__ << std::endl;
          return false;
        }
        ignore_flag.push_back(0);
      } else {
        for (int i = 1; i < cross_ind[start_i].size(); i++) {
          int temp = cross_ind[start_i][i];
          if (math::cgl::SameDirection(
                  erodingpoly[temp] - erodingpoly[temp + 1],
                  pipe[temp] - pipe[temp + 1])) {
            start_i = temp;
            ignore_flag.push_back(1);
            std::cout << "ind 0 is" << start_i << std::endl;
            break;
          }
        }
      }
    } else if (cross_ind[start_i].size() <= 3) {
      start_i++;
      if (start_i >= erodingpoly.size() - 2) {
        ignore_flag.push_back(0);
        break;
      }
      if (!math::cgl::SameDirection(
              erodingpoly[start_i] - erodingpoly[start_i + 1],
              pipe[start_i] - pipe[start_i + 1])) {
        for (const auto &p : erodingpoly) {
          std::cout << "[diff direction]" << start_i << "," << p.x << "," << p.y
                    << std::endl;
        }
        std::cout << __LINE__ << "start" << start_i << std::endl;
        return false;
      }
      ignore_flag.push_back(0);
    } else {
      int new_ind = -1;
      for (int i = 1; i < cross_ind[start_i].size(); i++) {
        int temp = cross_ind[start_i][i];
        if (temp < start_i) continue;
        if (std::abs(temp - start_i) != 1) {
          if (temp == erodingpoly.size() - 2) {
            new_ind = temp;
            ignore_flag.push_back(1);
            break;
          }
          if (math::cgl::SameDirection(
                  erodingpoly[temp] - erodingpoly[temp + 1],
                  pipe[temp] - pipe[temp + 1])) {
            new_ind = temp;
            ignore_flag.push_back(1);
            break;
          }
        } else {
          if (temp == erodingpoly.size() - 2) {
            new_ind = temp;
            ignore_flag.push_back(0);
            break;
          }
          if (math::cgl::SameDirection(
                  erodingpoly[temp] - erodingpoly[temp + 1],
                  pipe[temp] - pipe[temp + 1])) {
            new_ind = temp;
            ignore_flag.push_back(0);
            break;
          }
        }
      }
      if (new_ind < 0) {
        std::cout << __LINE__ << std::endl;
        return false;
      }
      start_i = new_ind;
    }
    if (step_count > erodingpoly.size()) {
      std::cout << __LINE__ << std::endl;
      return false;
    }
  }
  std::cout << __LINE__ << std::endl;
  sub_ind.push_back(start_i);
  if (ignore_flag.size() != sub_ind.size()) return false;
  std::cout << __LINE__ << sub_ind.size() << std::endl;
  for (const auto &p : sub_ind) {
    std::cout << "sub_ind:" << p << std::endl;
  }
  for (const auto &p : ignore_flag) {
    std::cout << "sub_ignore:" << p << std::endl;
  }
  // ignore_flag.push_back(0);
  // sub_ind.push_back(erodingpoly.size()-1);
  for (std::size_t i = 0; i < sub_ind.size(); i++) {
    if (ignore_flag[i] == 0) {
      sub_poly.push_back(erodingpoly[sub_ind[i]]);
    } else {
      Site pt;
      if (!math::cgl::GetCrossPtOfSegments(
              erodingpoly[sub_ind[i - 1]], erodingpoly[sub_ind[i - 1] + 1],
              erodingpoly[sub_ind[i]], erodingpoly[sub_ind[i] + 1], pt))
        return false;
      sub_poly.push_back(pt);
    }
  }
  if ((sub_poly.back() - erodingpoly.back()).mold() > 1e-2)
    sub_poly.push_back(erodingpoly.back());
  std::cout << __LINE__ << "," << sub_poly.size() << "," << erodingpoly.size()
            << std::endl;
  return true;
}

bool HybridRing::PipeWidthEroding(const SiteVec &pipe,
                                  const std::vector<int> &inds, double width,
                                  SiteVec &sub_poly) {
  SiteVec n_edgeset;
  // std::cout << "ind:"<< inds[0] << "," << inds[1] << std::endl;
  for (int i = inds[0]; i < inds[1]; i++) {
    Site temp = pipe[i + 1] - pipe[i];
    n_edgeset.push_back(temp.direction());
  }
  SiteVec erodingpoly;
  std::vector<std::pair<Site, bool>> poly_flag;
  for (int i = inds[0] + 1; i < inds[1]; i++) {
    int start = i - 1;
    int end = i;
    double cross = math::cgl::CrossProduct(n_edgeset[start], n_edgeset[end]);
    Site eroding_pt;
    eroding_pt =
        (n_edgeset[end] - n_edgeset[start]) * width / cross + pipe[end];
    erodingpoly.push_back(eroding_pt);
    poly_flag.push_back(std::make_pair(eroding_pt, false));
  }
  // get the first eroding pt and the end eroding pt
  Site p_front, p_back;
  // std::cout << pipe[inds[1]].x << "," << pipe[inds[1]].y << std::endl;
  // std::cout << pipe[inds[2]].x << "," << pipe[inds[2]].y << std::endl;
  // std::cout << n_edgeset.back().x << "," << n_edgeset.back().y << std::endl;
  // std::cout << "well done" << std::endl;
  if (!math::cgl::GetCrossPtOfLines(erodingpoly.front() + n_edgeset.front(),
                                    erodingpoly.front(), pipe[inds[0]],
                                    pipe[inds[3]], p_front)) {
    return false;
  }
  if (!math::cgl::GetCrossPtOfLines(erodingpoly.back() + n_edgeset.back(),
                                    erodingpoly.back(), pipe[inds[1]],
                                    pipe[inds[2]], p_back)) {
    return false;
  }
  erodingpoly.insert(erodingpoly.begin(), p_front);
  poly_flag.insert(poly_flag.begin(), std::make_pair(p_front, false));
  erodingpoly.insert(erodingpoly.end(), p_back);
  poly_flag.insert(poly_flag.end(), std::make_pair(p_back, false));

  std::vector<std::vector<int>> cross_ind;
  for (int i = 0; i < erodingpoly.size(); i++) {
    std::vector<int> temp;
    temp.push_back(i);
    cross_ind.push_back(temp);
  }
  for (int i = 0; i < erodingpoly.size() - 1; i++) {
    for (int j = i + 1; j < erodingpoly.size() - 1; j++) {
      if (math::cgl::CheckCrossOfSegments(erodingpoly[i], erodingpoly[i + 1],
                                          erodingpoly[j], erodingpoly[j + 1])) {
        cross_ind[i].push_back(j);
        cross_ind[j].push_back(i);
      }
    }
  }
  // SiteVec sub_poly;
  std::vector<int> sub_ind;
  std::vector<int> ignore_flag;
  int start_i = 0;
  int step_count = 0;
  ignore_flag.push_back(0);
  while (start_i < erodingpoly.size() - 1) {
    sub_ind.push_back(start_i);
    step_count++;
    if (cross_ind[start_i].size() <= 3) {
      start_i++;
      if (start_i == erodingpoly.size() - 2) {
        ignore_flag.push_back(0);
        break;
      }
      if (!math::cgl::SameDirection(
              erodingpoly[start_i] - erodingpoly[start_i + 1],
              pipe[start_i] - pipe[start_i + 1])) {
        std::cout << __LINE__ << std::endl;
        return false;
      }
      ignore_flag.push_back(0);
    } else {
      int new_ind = -1;
      for (int i = 1; i < cross_ind[start_i].size(); i++) {
        int temp = cross_ind[start_i][i];
        if (temp < start_i) continue;
        if (std::abs(temp - start_i) != 1) {
          if (temp == erodingpoly.size() - 2) {
            new_ind = temp;
            ignore_flag.push_back(1);
            break;
          }
          if (math::cgl::SameDirection(
                  erodingpoly[temp] - erodingpoly[temp + 1],
                  pipe[temp] - pipe[temp + 1])) {
            new_ind = temp;
            ignore_flag.push_back(1);
            break;
          }
        } else {
          if (temp == erodingpoly.size() - 2) {
            new_ind = temp;
            ignore_flag.push_back(0);
            break;
          }
          if (math::cgl::SameDirection(
                  erodingpoly[temp] - erodingpoly[temp + 1],
                  pipe[temp] - pipe[temp + 1])) {
            new_ind = temp;
            ignore_flag.push_back(0);
            break;
          }
        }
      }
      if (new_ind < 0) {
        // std::cout << __LINE__ << std::endl;
        return false;
      }
      start_i = new_ind;
    }
    // std::cout << __LINE__  << std::endl;
    if (step_count > erodingpoly.size()) return false;
  }
  // std::cout << __LINE__  << std::endl;
  sub_ind.push_back(start_i);
  if (ignore_flag.size() != sub_ind.size()) return false;
  // std::cout << __LINE__  << sub_ind.size() << std::endl;
  // ignore_flag.push_back(0);
  // sub_ind.push_back(erodingpoly.size()-1);
  for (std::size_t i = 0; i < sub_ind.size(); i++) {
    if (ignore_flag[i] == 0) {
      sub_poly.push_back(erodingpoly[sub_ind[i]]);
    } else {
      Site pt;
      if (!math::cgl::GetCrossPtOfSegments(
              erodingpoly[sub_ind[i - 1]], erodingpoly[sub_ind[i - 1] + 1],
              erodingpoly[sub_ind[i]], erodingpoly[sub_ind[i] + 1], pt))
        return false;
      sub_poly.push_back(pt);
    }
  }
  if ((sub_poly.back() - erodingpoly.back()).mold() > 1e-2)
    sub_poly.push_back(erodingpoly.back());
  std::cout << __LINE__ << sub_poly.size() << std::endl;
  return true;
}

int HybridRing::CheckCrossSeg2Inner(const Site &p1, const Site &p2,
                                    const int start, const int end, int &ind,
                                    double &dis) {
  if (end <= start) {
    std::cout << "the end is smaller than the start" << std::endl;
    return -1;
  }
  bool cross = false;
  for (int i = start; i < end; i++) {
    int next_i;
    if (i == inner_edge_.size() - 1) {
      next_i = 0;
    } else {
      next_i = i + 1;
    }
    if (math::cgl::CheckCrossOfSegments(p1, p2, inner_edge_[i],
                                        inner_edge_[next_i])) {
      cross = true;
      break;
    }
  }
  if (!cross) return 0;
  bool direction = false;
  double min_dis = std::numeric_limits<double>::max();
  for (int i = start; i < end; i++) {
    // std::cout << "cur" << i << std::endl;
    // std::cout << inner_edge_[i].x << "," << inner_edge_[i].y << std::endl;
    // std::cout << p1.x << "," << p1.y << std::endl;
    // std::cout << p2.x << "," << p2.y << std::endl;
    if (!math::cgl::CheckLeftOfEdge(inner_edge_[i], p1, p2)) {
      direction = true;
      double t_dis = math::cgl::GetDisFromPtToLine(inner_edge_[i], p1, p2);
      // std::cout << "dis" <<  i << "[]" << t_dis << std::endl;
      int t_ind = i;
      if (min_dis > t_dis) {
        dis = t_dis;
        min_dis = t_dis;
        ind = t_ind;
      }
    }
  }
  if (!direction) {
    std::cout << "direction is error" << std::endl;
    return -1;
  }
  return 1;
}

bool HybridRing::PathConnection() {
  SiteVec poly;
  for (const auto &p : ringpoly_) {
    poly.insert(poly.end(), p.begin(), p.end());
  }
  for (std::size_t i = 0; i < lsegs_.size(); i++) {
    for (std::size_t j = 0; j < lsegs_[i].size() / 2; j++) {
      poly.push_back(lsegs_[i][j * 2]);
    }
    if ((poly.back() - lsegs_[i].back()).mold() > 1e-2) {
      poly.push_back(lsegs_[i].back());
    }
  }
  for (std::size_t i = 0; i < pipe_main_pt_.size() / 2; i++) {
    poly.push_back(pipe_main_pt_[i * 2]);
  }
  poly.push_back(pipe_main_pt_.back());
  for (std::size_t i = 0; i < rsegs_.size(); i++) {
    for (std::size_t j = 0; j < rsegs_[i].size() / 2; j++) {
      poly.push_back(rsegs_[i][j * 2]);
    }
    if ((poly.back() - rsegs_[i].back()).mold() > 1e-2) {
      poly.push_back(rsegs_[i].back());
    }
  }
  poly.erase(std::unique(poly.begin(), poly.end()), poly.end());
  if (poly.size() < 3) return false;
  for (int i = 1; i < poly.size() - 1; i++) {
    // std::cout<< __LINE__ << "," << i << std::endl;
    double l_dis;
    if (i == 1) {
      l_dis = (poly[i] - poly[i - 1]).mold();
    } else {
      l_dis = (poly[i] - final_path_.back()).mold();
    }
    double r_dis = (poly[i + 1] - poly[i]).mold();
    Site l_unit = (poly[i] - poly[i - 1]).direction();
    Site r_unit = (poly[i + 1] - poly[i]).direction();
    double cross = math::cgl::CrossProduct(l_unit, r_unit);
    double n_dot = math::cgl::DotProduct(l_unit, r_unit);
    double smooth_dis = std::fabs(kMinRadius / cross * (1.0 - n_dot));
    // std::cout << __LINE__ << std::endl;
    if (smooth_dis >= l_dis || smooth_dis >= r_dis) {
      SiteVec new_path;
      if (i == 1) {
        LineSampling(poly[0], poly[i], new_path);
      } else {
        LineSampling(final_path_.back(), poly[i], new_path);
        // std::cout << __LINE__ << std::endl;
      }
      final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
    } else {
      // std::cout << __LINE__ << i << "," << cross << std::endl;
      Site start_i, end_i, cent_i;
      // start_i = poly[i] - l_unit * kMinRadius * (1.0-n_dot) /
      // std::fabs(cross); end_i = poly[i] + r_unit * kMinRadius * (1.0-n_dot) /
      // std::fabs(cross); cent_i = poly[i] + (r_unit - l_unit) * kMinRadius /
      // std::fabs(cross);
      if (std::fabs(cross) < 0.1) {
        SiteVec new_path;
        if (i == 1) {
          // std::cout << __LINE__ << std::endl;
          LineSampling(poly[0], poly[i], new_path);
          // std::cout << __LINE__ << std::endl;
        } else {
          // std::cout << __LINE__ << "," << final_path_.size()<< std::endl;
          LineSampling(final_path_.back(), poly[i], new_path);
          // std::cout << __LINE__ << std::endl;
        }
        final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
      } else if (cross > 0.0) {
        // std::cout << __LINE__ << std::endl;
        SiteVec new_path;
        start_i = poly[i] - l_unit * kMinRadius / cross * (1.0 - n_dot);
        end_i = poly[i] + r_unit * kMinRadius / cross * (1.0 - n_dot);
        cent_i = poly[i] + (r_unit - l_unit) * kMinRadius / cross;
        ArcSampling(cent_i, start_i, end_i, cross, new_path);
        final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
      } else {
        // std::cout << __LINE__ << std::endl;
        SiteVec new_path;
        start_i = poly[i] + l_unit * kMinRadius / cross * (1.0 - n_dot);
        end_i = poly[i] - r_unit * kMinRadius / cross * (1.0 - n_dot);
        cent_i = poly[i] - (r_unit - l_unit) * kMinRadius / cross;
        ArcSampling(cent_i, start_i, end_i, cross, new_path);
        final_path_.insert(final_path_.end(), new_path.begin(), new_path.end());
      }
    }
    // std::cout << __LINE__ << std::endl;
  }
  return true;
}

bool HybridRing::LineSampling(const geometry::Site &start,
                              const geometry::Site &end,
                              std::vector<geometry::Site> &sample) {
  double dis = (start - end).mold();
  Site dir = end - start;
  Site n_sample = dir.direction();
  double sample_line = 0.0;
  // while (sample_line < dis) {
  //  // std::cout << dis << "," << sample_line << "," << sample.size()<<
  //  std::endl; geometry::Site temp; temp.x = start.x + n_sample.x *
  //  sample_line; temp.y = start.y + n_sample.y * sample_line;
  //  sample.push_back(temp);
  //  sample_line += kSampleDis;
  //}
  if (dis < 2.0) {
    Site temp = start + n_sample * 0.5 * dis;
    // sample.push_back(start);
    sample.push_back(temp);
  } else {
    Site p1 = start + n_sample * 0.8;
    Site p2 = start + n_sample * 0.5 * dis;
    Site p3 = start + n_sample * (dis - 0.8);
    // sample.push_back(start);
    sample.push_back(p1);
    sample.push_back(p2);
    sample.push_back(p3);
  }
  // std::cout << "line over" << std::endl;
  return true;
}

bool HybridRing::ArcSampling(const geometry::Site &cent,
                             const geometry::Site &start,
                             const geometry::Site &end, const double cross,
                             std::vector<geometry::Site> &sample) {
  bool autoclockwise = cross > 0 ? true : false;
  // double rad_range = std::abs(acos(cross));
  double angle =
      ((start - cent).x * (end - cent).x + (start - cent).y * (end - cent).y) /
      ((start - cent).mold() * (end - cent).mold());
  double rad_range = std::abs(acos(angle));
  double start_rad = asin((start.y - cent.y) / (start - cent).mold());
  if (start.x - cent.x < 0 && start.y - cent.y > 0) {
    start_rad = M_PI - start_rad;
  } else if (start.x - cent.x < 0 && start.y - cent.y < 0) {
    start_rad = -start_rad - M_PI;
  }
  double single = 0.3 / kMinRadius;
  double sample_rad = 0.0;
  double dis_mold = (cent - start).mold();
  double dis_mold1 = (cent - end).mold();
  // std::cout << "dis_mold:" << dis_mold << "," << dis_mold1 << std::endl;
  // std::cout << "rad_range:" << rad_range << std::endl;
  // std::cout << "start_rad:" << start_rad << std::endl;
  // std::cout << "start:" << start.x << "," << start.y << std::endl;
  // std::cout << "end:" << end.x << "," << end.y << std::endl;
  // std::cout << "cent:" << cent.x << "," << cent.y << std::endl;
  while (sample_rad < rad_range) {
    geometry::Site temp;
    if (autoclockwise) {
      double cur_rad = start_rad + sample_rad;
      cur_rad = cur_rad > M_PI ? cur_rad - 2 * M_PI : cur_rad;
      double x = kMinRadius * cos(cur_rad);
      double y = kMinRadius * sin(cur_rad);
      temp.x = cent.x + x;
      temp.y = cent.y + y;
    } else {
      double cur_rad = start_rad - sample_rad;
      // std::cout << "cur_rad" << cur_rad << std::endl;
      cur_rad = cur_rad < -M_PI ? cur_rad + 2 * M_PI : cur_rad;
      double x = kMinRadius * cos(cur_rad);
      double y = kMinRadius * sin(cur_rad);
      temp.x = cent.x + x;
      temp.y = cent.y + y;
      // std::cout << "[circle]" << temp.x << "," << temp.y << std::endl;
    }
    sample.push_back(temp);
    sample_rad += single;
  }
  return true;
}

bool HybridRing::ArcSampling(const geometry::Site &cent,
                             const geometry::Site &start,
                             const geometry::Site &end, const double cross,
                             const double radius,
                             std::vector<geometry::Site> &sample) {
  bool autoclockwise = cross > 0 ? true : false;
  // double rad_range = std::abs(acos(cross));
  double angle =
      ((start - cent).x * (end - cent).x + (start - cent).y * (end - cent).y) /
      ((start - cent).mold() * (end - cent).mold());
  double rad_range = std::abs(acos(angle));
  double start_rad = asin((start.y - cent.y) / (start - cent).mold());
  if (start.x - cent.x < 0 && start.y - cent.y > 0) {
    start_rad = M_PI - start_rad;
  } else if (start.x - cent.x < 0 && start.y - cent.y < 0) {
    start_rad = -start_rad - M_PI;
  }
  double single = kSampleDis / radius;
  double sample_rad = 0.5 * rad_range;
  // std::cout << "dis_mold:" << dis_mold << "," << dis_mold1 << std::endl;
  // std::cout << "rad_range:" << rad_range << std::endl;
  // std::cout << "start_rad:" << start_rad << std::endl;
  // std::cout << "start:" << start.x << "," << start.y << std::endl;
  // std::cout << "end:" << end.x << "," << end.y << std::endl;
  // std::cout << "cent:" << cent.x << "," << cent.y << std::endl;
  geometry::Site temp;
  if (autoclockwise) {
    double cur_rad = start_rad + sample_rad;
    cur_rad = cur_rad > M_PI ? cur_rad - 2 * M_PI : cur_rad;
    double x = radius * cos(cur_rad);
    double y = radius * sin(cur_rad);
    temp.x = cent.x + x;
    temp.y = cent.y + y;
  } else {
    double cur_rad = start_rad - sample_rad;
    // std::cout << "cur_rad" << cur_rad << std::endl;
    cur_rad = cur_rad < -M_PI ? cur_rad + 2 * M_PI : cur_rad;
    double x = kMinRadius * cos(cur_rad);
    double y = kMinRadius * sin(cur_rad);
    temp.x = cent.x + x;
    temp.y = cent.y + y;
    // std::cout << "[circle]" << temp.x << "," << temp.y << std::endl;
  }
  sample.push_back(temp);
  sample.push_back(end);
  return true;
}

bool HybridRing::BSplineOptiming(const SiteVec &poly) {
  if (poly.size() < 3) return false;
  final_path_.push_back(poly[0]);

  for (int i = 1; i < poly.size() - 1; i++) {
    Site l_unit = (poly[i] - poly[i - 1]).direction();
    Site r_unit = (poly[i + 1] - poly[i]).direction();
    Site last_unit(0, 0);
    double cos_theta0 = 1.0, cos_theta1 = 1.0;
    cos_theta1 = math::cgl::DotProduct(l_unit, r_unit);

    if (i >= 2) {
      last_unit = (poly[i - 1] - poly[i - 2]).direction();
      cos_theta0 = math::cgl::DotProduct(last_unit, l_unit);
    }

    // std::cout << "----------------------" << std::endl;
    // std::cout << "l_dis:" << l_dis << std::endl;
    // std::cout << "r_dis:" << r_dis << std::endl;
    // std::cout << "cross:" << cross << std::endl;
    // std::cout << "n_dot:" << n_dot << std::endl;
    // std::cout << "smooth_dis:" << smooth_dis << std::endl;

    // set b spline params by cos theta
    std::vector<double> param;
    param.reserve(4);
    double b0_pos_ = 0.15 + (1 - std::fabs(cos_theta0)) * 0.05;
    double b0_para_ = 0.05 + (1 - std::fabs(cos_theta0)) * 0.02;
    double b1_pos_ = 0.85 - (1 - std::fabs(cos_theta1)) * 0.05;
    double b1_para_ = 0.05 + (1 - std::fabs(cos_theta1)) * 0.02;
    param.push_back(b0_pos_);
    param.push_back(b0_para_);
    param.push_back(b1_pos_);
    param.push_back(b1_para_);

    SiteVec b_samples;
    BSampling(poly[i - 1], poly[i], r_unit, last_unit, param, b_samples);
    final_path_.insert(final_path_.end(), b_samples.begin(), b_samples.end());
  }

  SiteVec new_path1;
  BLineSampling(final_path_.back(), poly.back(), new_path1);
  final_path_.insert(final_path_.end(), new_path1.begin(), new_path1.end());
  return true;
}

bool HybridRing::BSampling(const Site &start, const Site &end,
                           const Site &next_direction,
                           const Site &last_direction,
                           const std::vector<double> &param,
                           std::vector<Site> &sample) {
  Site dir = end - start;
  double dis = dir.mold();
  Site n_sample = dir.direction();

  // ���
  bool short_lane = false;
  if (dis < 5.0) short_lane = true;
  double d1 = param[0] * dis;
  double d2 = param[2] * dis;
  double offset_dis = 0.05 * dis > 3.0 ? 0.05 * dis : 3.0;
  if (d1 > offset_dis) d1 = offset_dis;
  if (d2 < (dis - offset_dis)) d2 = dis - offset_dis;
  // �������̫������d1��d2���������м�
  // TODO �������Ƕ�̫С���ʵ���d1��d2���������м�
  double cos_ang1 = math::cgl::DotProduct(n_sample, last_direction);
  double cos_ang2 = math::cgl::DotProduct(n_sample, next_direction);

  if (cos_ang1 < 0.0) {
    d1 += 2.0 + 8.0 * (0.0 - cos_ang1);
    if (d1 > (dis * 0.4)) {
      short_lane = true;
    }
  }

  if (cos_ang2 < 0.0) {
    d2 -= 2.0 + 8.0 * (0.0 - cos_ang2);
    if (d2 < (dis * 0.6)) {
      short_lane = true;
    }
  }
  geometry::Site pt1 = start + n_sample * d1 - last_direction * 0.5 * param[1];
  geometry::Site pt2 = start + n_sample * 0.5 * dis;
  geometry::Site pt3 = start + n_sample * d2 + next_direction * 0.5 * param[3];

  if (short_lane) {
    sample = SiteVec{pt2};
  } else {
    sample = SiteVec{pt1, pt2, pt3};
  }
  return true;
}

// sparse line sampling
bool HybridRing::BLineSampling(const geometry::Site &start,
                               const geometry::Site &end,
                               std::vector<geometry::Site> &sample) {
  sample.clear();
  double dis = (start - end).mold();
  Site dir = end - start;
  Site n_sample = dir.direction();
  double sample_line = 0.0;
  std::vector<double> longsample_dis;

  // longsample_dis.push_back(0);
  longsample_dis.push_back(dis * 0.33);
  longsample_dis.push_back(dis * 0.66);
  // longsample_dis.push_back(dis);

  // longsample_dis.push_back(0.0);
  // longsample_dis.push_back(0.8);
  // longsample_dis.push_back(-0.8);

  if (dis < 2.0) {
    // ����С��2�ף�ÿ��0.4�״�һ����
    while (sample_line < dis) {
      geometry::Site temp;
      temp.x = start.x + n_sample.x * sample_line;
      temp.y = start.y + n_sample.y * sample_line;
      sample.push_back(temp);
      sample_line += 0.4;
    }
  } else {
    // ����㡢���ǰ0.8���յ�ǰ0.8�����
    // longsample_dis[2] = dis + longsample_dis[2];

    for (std::size_t i = 0; i < longsample_dis.size(); i++) {
      geometry::Site temp;
      temp.x = start.x + n_sample.x * longsample_dis[i];
      temp.y = start.y + n_sample.y * longsample_dis[i];
      sample.push_back(temp);
    }
  }
  return true;
}

}  // namespace coverage
