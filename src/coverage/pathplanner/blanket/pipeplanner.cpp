#include "pathplanner/blanket/pipeplanner.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "geometry/dubins.h"
#include "math/cgl.h"
#include "optimizer/pure_pursuit.h"
#include "pathplanner/blanket/delaunay.h"

namespace coverage {

PipePlanner::PipePlanner() {
  state_ = false;
  max_distance_ = std::numeric_limits<double>::max();
}

bool PipePlanner::Interface(const std::vector<double> &x,
                            const std::vector<double> &y,
                            const std::vector<double> &x_edge,
                            const std::vector<double> &y_edge) {
  if (!EdgeSizeValid(x, y) || !EdgeSizeValid(x_edge, y_edge)) {
    state_ = false;
    return false;
  }
  EdgeReconstruct(x, y);
  state_ = true;
  edge1_.push_back(Site(x_edge[0] - x_offset_, y_edge[0] - y_offset_));
  edge1_.push_back(Site(x_edge[1] - x_offset_, y_edge[1] - y_offset_));
  edge2_.push_back(Site(x_edge[2] - x_offset_, y_edge[2] - y_offset_));
  edge2_.push_back(Site(x_edge[3] - x_offset_, y_edge[3] - y_offset_));
  if (!IfAutoclockwise(orig_edge_)) {
    std::reverse(orig_edge_.begin(), orig_edge_.end());
  }
  // for (auto &p : orig_edge_) {
  //   std::cout << "[thin]" << p.x << "," << p.y << std::endl;
  // }
  // for (auto &p : edge1_) {
  //   std::cout << "[edge1_]" << p.x << "," << p.y << std::endl;
  // }
  // for (auto &p : edge2_) {
  //   std::cout << "[edge2_]" << p.x << "," << p.y << std::endl;
  // }
  if (!EdgeEroding()) {
    // std::cout << __LINE__ << std::endl;
    state_ = false;
    return false;
  }
  if (!EdgeThinning()) {
    // std::cout << __LINE__ << std::endl;
    state_ = false;
    return false;
  }
  return true;
}

bool PipePlanner::Planning() {
  // std::cout << __LINE__ << std::endl;
  if (!state_) {
    // std::cout << __LINE__ << std::endl;
    return false;
  }
  // std::cout << __LINE__ << std::endl;
  if (!TriangleGeneration()) {
    // std::cout << __LINE__ << std::endl;
    return false;
  }
  // std::cout << __LINE__ << std::endl;
  if (!SkeletonGeneration()) {
    // std::cout << __LINE__ << std::endl;
    return false;
  }
  // std::cout << __LINE__ << std::endl;
  // if (!GenerateEdgeCurve()) return false;
  if (!GenerateFilterCurve()) {
    // std::cout << __LINE__ << std::endl;
    return false;
  }
  // std::cout << __LINE__ << std::endl;
  if (!PathConnect()) {
    // std::cout << __LINE__ << std::endl;
    return false;
  }
  // std::cout << __LINE__ << std::endl;
  if (!PathSmooth()) {
    // std::cout << __LINE__ << std::endl;
    return false;
  }
  Recovery(final_path_);
  return true;
}

bool PipePlanner::EdgeThinning() {
  double default_offset = kTurningRadius + 0.5;
  int edge1_ind1;
  int edge1_ind2;
  int edge2_ind1;
  int edge2_ind2;
  bool before1 = false;
  bool before2 = false;
  std::vector<std::pair<int, Site>> erase1;
  std::vector<std::pair<int, Site>> erase2;
  for (std::size_t i = 0; i < orig_edge_.size(); i++) {
    double dis = (orig_edge_[i] - edge1_[0]).mold();
    if (dis < 1e-2) {
      edge1_ind1 = i;
      int before = edge1_ind1 == 0 ? orig_edge_.size() - 1 : edge1_ind1 - 1;
      if ((orig_edge_[before] - edge1_[1]).mold() < 1e-2) {
        edge1_ind2 = before;
        before1 = true;
      } else {
        edge1_ind2 = edge1_ind1;
        edge1_ind1 = edge1_ind1 == orig_edge_.size() - 1 ? 0 : edge1_ind1 + 1;
      }
      break;
    }
  }
  for (std::size_t i = 0; i < orig_edge_.size(); i++) {
    double dis = (orig_edge_[i] - edge2_[0]).mold();
    if (dis < 1e-2) {
      edge2_ind1 = i;
      int before = edge2_ind1 == 0 ? orig_edge_.size() - 1 : edge2_ind1 - 1;
      if ((orig_edge_[before] - edge2_[1]).mold() < 1e-2) {
        edge2_ind2 = before;
        before2 = true;
      } else {
        edge2_ind2 = edge2_ind1;
        edge2_ind1 = edge2_ind1 == orig_edge_.size() - 1 ? 0 : edge2_ind1 + 1;
      }
      break;
    }
  }
  // std::cout << __LINE__<< std::endl;
  // std::cout << "edge ind" << edge1_ind1 << ","
  //                         << edge1_ind2 << ","
  //                         << edge2_ind1 << ","
  //                         << edge2_ind2 << std::endl;
  for (int i = 0; i < orig_edge_.size() - 2; i++) {
    int cur = edge1_ind2 - i;
    if (cur < 0) {
      cur = orig_edge_.size() + cur;
    }
    double dis = GetDisFromPtToLine(orig_edge_[cur], orig_edge_[edge1_ind1],
                                    orig_edge_[edge1_ind2]);
    if (dis > default_offset) {
      int before_cur = cur == orig_edge_.size() - 1 ? 0 : cur + 1;
      double dis_bef =
          GetDisFromPtToLine(orig_edge_[before_cur], orig_edge_[edge1_ind1],
                             orig_edge_[edge1_ind2]);
      double ratio = 1.0 * (default_offset - dis_bef) / (dis - dis_bef);
      Site pt = orig_edge_[before_cur] +
                (orig_edge_[cur] - orig_edge_[before_cur]) * ratio;
      erase1.push_back(std::make_pair(cur, pt));
      // std::cout << "---------jf-------"
      //           << edge1_ind1 << "," << edge1_ind2 << ","
      //           << cur << "," << before_cur << ","
      //           << dis << "," << dis_bef << ","
      //           << "default:" << default_offset << ","
      //           << orig_edge_[edge1_ind1].x << ","
      //           << orig_edge_[edge1_ind1].y << ","
      //           << orig_edge_[edge1_ind2].x << ","
      //           << orig_edge_[edge1_ind2].y << ","
      //           << orig_edge_[cur].x << ","
      //           << orig_edge_[cur].y << ","
      //           << orig_edge_[before_cur].x << ","
      //           << orig_edge_[before_cur].y << ","
      //           << pt.x << "," << pt.y << std::endl;
      break;
    }
  }
  // std::cout << __LINE__<< std::endl;
  for (int i = 0; i < orig_edge_.size() - 2; i++) {
    int cur = edge1_ind1 + i;
    if (cur > orig_edge_.size() - 1) {
      cur = cur - orig_edge_.size();
    }
    double dis = GetDisFromPtToLine(orig_edge_[cur], orig_edge_[edge1_ind1],
                                    orig_edge_[edge1_ind2]);
    if (dis > default_offset) {
      int before_cur = cur == 0 ? orig_edge_.size() - 1 : cur - 1;
      double dis_bef =
          GetDisFromPtToLine(orig_edge_[before_cur], orig_edge_[edge1_ind1],
                             orig_edge_[edge1_ind2]);
      double ratio = 1.0 * (default_offset - dis_bef) / (dis - dis_bef);
      Site pt = orig_edge_[before_cur] +
                (orig_edge_[cur] - orig_edge_[before_cur]) * ratio;
      erase1.push_back(std::make_pair(cur, pt));
      break;
    }
  }
  // std::cout << __LINE__<< std::endl;
  for (int i = 0; i < orig_edge_.size() - 2; i++) {
    int cur = edge2_ind2 - i;
    if (cur < 0) {
      cur = orig_edge_.size() + cur;
    }
    // double dis = (orig_edge_[cur]-orig_edge_[edge2_ind2]).mold();
    double dis = GetDisFromPtToLine(orig_edge_[cur], orig_edge_[edge2_ind1],
                                    orig_edge_[edge2_ind2]);
    if (dis > default_offset) {
      int before_cur = cur == orig_edge_.size() - 1 ? 0 : cur + 1;
      double dis_bef =
          GetDisFromPtToLine(orig_edge_[before_cur], orig_edge_[edge2_ind1],
                             orig_edge_[edge2_ind2]);
      double ratio = 1.0 * (default_offset - dis_bef) / (dis - dis_bef);
      Site pt = orig_edge_[before_cur] +
                (orig_edge_[cur] - orig_edge_[before_cur]) * ratio;
      erase2.push_back(std::make_pair(cur, pt));
      break;
    }
  }
  // std::cout << __LINE__<< std::endl;
  for (int i = 0; i < orig_edge_.size() - 2; i++) {
    int cur = edge2_ind1 + i;
    if (cur > orig_edge_.size() - 1) {
      cur = cur - orig_edge_.size();
    }
    double dis = GetDisFromPtToLine(orig_edge_[cur], orig_edge_[edge2_ind1],
                                    orig_edge_[edge2_ind2]);
    if (dis > default_offset) {
      int before_cur = cur == 0 ? orig_edge_.size() - 1 : cur - 1;
      double dis_bef =
          GetDisFromPtToLine(orig_edge_[before_cur], orig_edge_[edge2_ind1],
                             orig_edge_[edge2_ind2]);
      double ratio = 1.0 * (default_offset - dis_bef) / (dis - dis_bef);
      Site pt = orig_edge_[before_cur] +
                (orig_edge_[cur] - orig_edge_[before_cur]) * ratio;
      erase2.push_back(std::make_pair(cur, pt));
      break;
    }
  }
  // std::cout << __LINE__<< std::endl;
  // std::cout << "erase" << erase1[0].first << ","
  //                      << erase1[1].first << ","
  //                      << erase2[0].first << ","
  //                      << erase2[1].first << std::endl;
  SiteVec filter;
  if (erase2[0].first > erase1[1].first) {
    for (int i = erase1[1].first; i <= erase2[0].first; i++) {
      filter.push_back(orig_edge_[i]);
    }
  } else {
    for (int i = erase1[1].first; i < orig_edge_.size(); i++) {
      filter.push_back(orig_edge_[i]);
    }
    for (int i = 0; i <= erase2[0].first; i++) {
      filter.push_back(orig_edge_[i]);
    }
  }
  filter.push_back(erase2[0].second);
  filter.push_back(erase2[1].second);
  if (erase1[0].first > erase2[1].first) {
    for (int i = erase2[1].first; i <= erase1[0].first; i++) {
      filter.push_back(orig_edge_[i]);
    }
  } else {
    for (int i = erase2[1].first; i < orig_edge_.size(); i++) {
      filter.push_back(orig_edge_[i]);
    }
    for (int i = 0; i <= erase1[0].first; i++) {
      filter.push_back(orig_edge_[i]);
    }
  }
  filter.push_back(erase1[0].second);
  filter.push_back(erase1[1].second);
  orig_edge_.clear();
  edge1_.clear();
  edge2_.clear();
  orig_edge_.insert(orig_edge_.end(), filter.begin(), filter.end());
  edge1_.push_back(erase2[0].second);
  edge1_.push_back(erase2[1].second);
  edge2_.push_back(erase1[0].second);
  edge2_.push_back(erase1[1].second);
  // for (auto &p : orig_edge_) {
  //   std::cout << "[thin]" << p.x << "," << p.y << std::endl;
  // }
  // for (auto &p : edge1_) {
  //   std::cout << "[edge1_]" << p.x << "," << p.y << std::endl;
  // }
  // for (auto &p : edge2_) {
  //   std::cout << "[edge2_]" << p.x << "," << p.y << std::endl;
  // }
  return true;
}

bool PipePlanner::EdgeEroding() {
  if (orig_edge_.size() < 3) return false;
  // 1.edge set and nornalize it
  SiteVec edgeset, n_edgeset;
  int count = orig_edge_.size();
  for (int i = 0; i < count; i++) {
    int next = (i == count - 1) ? 0 : (i + 1);
    geometry::Site temp = orig_edge_[next] - orig_edge_[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  // 2.compute the eroding point
  SiteVec erodingpoly;
  std::vector<std::pair<geometry::Site, bool>> eroding_poly;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : (i - 1);
    int end = i;
    double cross_product = n_edgeset[start].x * n_edgeset[end].y -
                           n_edgeset[start].y * n_edgeset[end].x;
    // todo
    geometry::Site eroding_pt;
    eroding_pt =
        (n_edgeset[end] - n_edgeset[start]) * kCarWidth / cross_product +
        orig_edge_[end];
    erodingpoly.push_back(eroding_pt);
    eroding_poly.push_back(std::make_pair(eroding_pt, false));
  }
  int edge1_ind = FindNearInd(edge1_[0]);
  int edge2_ind = FindNearInd(edge1_[1]);
  int edge3_ind = FindNearInd(edge2_[0]);
  int edge4_ind = FindNearInd(edge2_[1]);
  // for (const auto &p : orig_edge_) {
  //   std::cout << p.x << "," << p.y << std::endl;
  // }
  orig_edge_.clear();
  orig_edge_.insert(orig_edge_.end(), erodingpoly.begin(), erodingpoly.end());
  edge1_.clear();
  edge2_.clear();
  edge1_.push_back(orig_edge_[edge1_ind]);
  edge1_.push_back(orig_edge_[edge2_ind]);
  edge2_.push_back(orig_edge_[edge3_ind]);
  edge2_.push_back(orig_edge_[edge4_ind]);
  // for (const auto &p : orig_edge_) {
  //   std::cout << p.x << "," << p.y << std::endl;
  // }
  // std::cout << edge1_[0].x << "," << edge1_[0].y << std::endl;
  // std::cout << edge1_[1].x << "," << edge1_[1].y << std::endl;
  // std::cout << edge2_[0].x << "," << edge2_[0].y << std::endl;
  // std::cout << edge2_[1].x << "," << edge2_[1].y << std::endl;
  return true;
  // 4. check if there is cross between the edges
  std::vector<int> cross_points;
  std::vector<geometry::Site> sub_polygon;
  std::vector<SiteVec> sub_polygons;
  int correct_num = 0;
  while (correct_num < count) {
    int start_i = 0;
    int end_i = count;
    for (int i = 0; i < count; i++) {
      if (eroding_poly[i].second == true) {
        start_i++;
      } else {
        break;
      }
    }
    int tmp_i = start_i;
    bool cur_cross = false;
    SiteVec tmp_poly;
    tmp_poly.clear();
    sub_polygons.push_back(tmp_poly);
    while (end_i != start_i) {
      if (!cur_cross) {
        sub_polygons.back().push_back(erodingpoly[tmp_i]);
        eroding_poly[tmp_i].second = true;
        correct_num++;
      }
      int next_i = (tmp_i == count - 1) ? 0 : tmp_i + 1;
      int before_i = (tmp_i == 0) ? count - 1 : tmp_i - 1;
      cur_cross = false;
      for (int j = tmp_i + 1; j < count; j++) {
        int next_j = (j == count - 1) ? 0 : (j + 1);
        if (j == before_i || j == next_i) continue;
        if (CheckCrossOfLines(erodingpoly[tmp_i], erodingpoly[next_i],
                              erodingpoly[j], erodingpoly[next_j])) {
          cur_cross = true;
          geometry::Site cross_pt =
              GetCrossPt(erodingpoly[tmp_i], erodingpoly[next_i],
                         erodingpoly[j], erodingpoly[next_j]);
          sub_polygon.push_back(cross_pt);
          sub_polygons.back().push_back(cross_pt);
          tmp_i = j;
          end_i = count;
          break;
        }
      }
      if (!cur_cross) {
        tmp_i++;
        tmp_i = tmp_i > count - 1 ? 0 : tmp_i;
        end_i = tmp_i;
      }
    }
  }
  // select an optimal polygon
  int max_size = std::numeric_limits<int>::min();
  int select_index = 0;
  for (std::size_t i = 0; i < sub_polygons.size(); i++) {
    if (IfAutoclockwise(sub_polygons[i])) {
      if (max_size < sub_polygons.size()) {
        select_index = i;
        max_size = sub_polygons.size();
      }
    }
  }
  orig_edge_.clear();
  orig_edge_.insert(orig_edge_.end(), sub_polygons[select_index].begin(),
                    sub_polygons[select_index].end());
  // for (auto &p : sub_polygons[select_index]) {
  //   std::cout << "[erode]" << p.x << "," << p.y << std::endl;
  // }
  return true;
}

int PipePlanner::FindNearInd(const Site &pt) {
  int near_ind = 0;
  double min_dis = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < orig_edge_.size(); i++) {
    double dis = (orig_edge_[i] - pt).mold();
    if (dis < min_dis) {
      min_dis = dis;
      near_ind = i;
    }
  }
  return near_ind;
}

bool PipePlanner::EdgeSizeValid(const std::vector<double> &x,
                                const std::vector<double> &y) {
  if (x.size() == 0 || y.size() == 0) return false;
  if (x.size() != y.size()) return false;
  if (x.size() < 4) return false;
  return true;
}

bool PipePlanner::EdgeReconstruct(const std::vector<double> &x,
                                  const std::vector<double> &y) {
  // 1. get the x_offset and y_offset
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
  // 2. reconstruct the edge
  for (std::size_t i = 0; i < x.size(); i++) {
    geometry::Site temp;
    temp.x = x[i] - x_offset_;
    temp.y = y[i] - y_offset_;
    orig_edge_.push_back(temp);
  }
  return true;
}

bool PipePlanner::TriangleGeneration() {
  std::vector<delaunay::Point<float>> points;
  for (const auto &p : orig_edge_) {
    delaunay::Point<float> tmp;
    tmp.x = p.x;
    tmp.y = p.y;
    points.push_back(tmp);
  }
  const auto triangulation = delaunay::triangulate(points);
  if (triangulation.triangles.size() < 2) return false;
  // add the basic info for each triangle
  // one triangle which have no edges on polygon is called 0-level triangle
  // one triangle which have an edge on polygon is called I-level triangle
  // one triangle which have two edges on polygon is called II-level triangle
  for (auto const &e : triangulation.triangles) {
    SiteVec temp;
    temp.push_back(Site(e.p0.x, e.p0.y));
    temp.push_back(Site(e.p1.x, e.p1.y));
    temp.push_back(Site(e.p2.x, e.p2.y));
    Site cent_pt = Incenter(temp[0], temp[1], temp[2]);
    if (!IsInSide(cent_pt, orig_edge_)) continue;
    std::cout << e.p0.x << "," << e.p0.y << "," << e.p1.x << "," << e.p1.y
              << "," << e.p2.x << "," << e.p2.y << "," << e.p0.x << ","
              << e.p0.y << std::endl;
    delau_triangle_.push_back(temp);
    // delau_triangle_center_.push_back(cent_pt);
    std::pair<int, std::vector<int>> temp_type;
    temp_type = GetPtType(temp);
    if (temp_type.first > 2) {
      std::cout << "there must be something wrong" << std::endl;
      return false;
    }
    delau_triangle_type_.push_back(temp_type);
  }
  return true;
}

bool PipePlanner::SkeletonGeneration() {
  for (std::size_t i = 0; i < delau_triangle_type_.size(); i++) {
    std::vector<int> tmp;
    tmp.push_back(i);
    relation_map_.push_back(tmp);
    // std::cout << "[]" << i << ","
    //           << delau_triangle_type_[i].first << ","
    //           << delau_triangle_type_[i].second[0] << ","
    //           << delau_triangle_type_[i].second[1] << ","
    //           << delau_triangle_type_[i].second[2] << ","
    //           << delau_triangle_type_[i].second[3] << ","
    //           << delau_triangle_type_[i].second[4] << ","
    //           << delau_triangle_type_[i].second[5] << std::endl;
  }
  for (std::size_t i = 0; i < delau_triangle_type_.size(); i++) {
    for (std::size_t j = 0; j < delau_triangle_type_.size(); j++) {
      if (i == j) continue;
      std::vector<int> common_edges;
      if (IsCommonEdge(delau_triangle_type_[i].second,
                       delau_triangle_type_[j].second, common_edges)) {
        relation_map_[i].push_back(j);
      }
    }
  }
  // for (std::size_t i = 0; i < relation_map_.size(); i++) {
  //   for (std::size_t j = 0; j < relation_map_[i].size(); j++) {
  //     std::cout << "relation_map_" << relation_map_[i][j] << ",";
  //   }
  //   std::cout << std::endl;
  // }
  if (!GetMainSkeleton(main_ind_)) return false;
  if (!CentralPtFilter(main_ind_)) return false;
  if (!SkeletonDilation()) return false;
  return true;
}

bool PipePlanner::IsInSide(const geometry::Site &pt,
                           const std::vector<Site> &polygon) {
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::min();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::min();
  for (const auto &p : polygon) {
    if (x_min > p.x) x_min = p.x;
    if (x_max < p.x) x_max = p.x;
    if (y_min > p.y) y_min = p.y;
    if (y_max < p.y) y_max = p.y;
  }
  if (pt.x < x_min || pt.x > x_max || pt.y < y_min || pt.y > y_max) {
    return false;
  }
  int counter = 0;
  int i;
  double xinters;
  geometry::Site p1;
  geometry::Site p2;
  int N = polygon.size();
  p1 = polygon[0];
  for (i = 1; i <= N; i++) {
    p2 = polygon[i % N];
    if (pt.y > std::min<double>(p1.y, p2.y)) {
      if (pt.y <= std::max<double>(p1.y, p2.y)) {
        if (pt.x <= std::max<double>(p1.x, p2.x)) {
          if (p1.y != p2.y) {
            xinters = (pt.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            if (p1.x == p2.x || pt.x <= xinters) counter++;
          }
        }
      }
    }
    p1 = p2;
  }
  if (counter % 2 == 0) return false;
  return true;
}

std::pair<int, std::vector<int>> PipePlanner::GetPtType(const SiteVec &poly) {
  std::vector<int> p;
  for (const auto &t : poly) {
    for (int i = 0; i < orig_edge_.size(); i++) {
      if (std::fabs(t.x - orig_edge_[i].x) < 1e-2 &&
          std::fabs(t.y - orig_edge_[i].y) < 1e-2) {
        p.push_back(i);
        break;
      }
    }
  }
  int p0, p1, p2, p_total;
  p0 = 0;
  p1 = 0;
  p2 = 0;
  if (std::abs(p[0] - p[1]) == 1 ||
      std::abs(p[0] - p[1]) == orig_edge_.size() - 1) {
    p0 = 1;
  }
  if (std::abs(p[1] - p[2]) == 1 ||
      std::abs(p[1] - p[2]) == orig_edge_.size() - 1) {
    p1 = 1;
  }
  if (std::abs(p[2] - p[0]) == 1 ||
      std::abs(p[2] - p[0]) == orig_edge_.size() - 1) {
    p2 = 1;
  }
  p_total = p0 + p1 + p2;
  std::vector<int> p_type;
  // the first three value represent the point index
  p_type.push_back(p[0]);
  p_type.push_back(p[1]);
  p_type.push_back(p[2]);
  // the last three value represent the common relation
  p_type.push_back(p0);
  p_type.push_back(p1);
  p_type.push_back(p2);
  return std::make_pair(p_total, p_type);
  // return 0;
}

bool PipePlanner::IsCommonEdge(const std::vector<int> &p1,
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
      // if ((p1[i] == p2[(j+1)%3] && p1[(i+1)%3] == p2[j]) ||
      //     (p1[(i+1)%3] == p2[(j+1)%3] && p1[i] == p2[j])) {
      //   common.push_back(i);
      //   common.push_back((i+1)%3);
      //   common.push_back((j+1)%3);
      //   common.push_back(j);
      //   return true;
      // }
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

void PipePlanner::DeepFirstSearch(int ind, int dep) {
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

bool PipePlanner::GetMainSkeleton(std::vector<int> &p) {
  for (std::size_t i = 0; i < relation_map_.size(); i++) {
    visited_.push_back(0);
    father_.push_back(-1);
  }
  max_dep_ = -1;
  DeepFirstSearch(0, 0);
  // std::cout << last_ind_ << "[]"<< relation_map_.size()<< std::endl;
  // for (std::size_t i = 0; i < father_.size(); i++) {
  //   std::cout << "[father]" << father_[i] << std::endl;
  // }
  for (std::size_t i = 0; i < relation_map_.size(); i++) {
    visited_[i] = 0;
    father_[i] = -1;
  }
  max_dep_ = -1;
  DeepFirstSearch(last_ind_, 0);
  int temp = last_ind_;
  // for (std::size_t i = 0; i < father_.size(); i++) {
  //   std::cout << "[father]" << father_[i] << std::endl;
  // }
  // std::cout << "[father size]" << last_ind_ << "," << father_.size() <<
  // std::endl;
  while (temp != -1) {
    // std::cout << temp << std::endl;
    p.push_back(temp);
    temp = father_[temp];
  }
  if (p.size() < 2) return false;
  // for (const auto &ii : p) {
  //   std::cout << "[type]" << ii << "," <<  delau_triangle_type_[ii].first <<
  //   std::endl;
  // }
  if (delau_triangle_type_[p.front()].first != 2 ||
      delau_triangle_type_[p.back()].first != 2)
    return false;
  return true;
}

bool PipePlanner::CentralPtFilter(const std::vector<int> ind) {
  // std::cout << "[ind_size]" << ind.size() << std::endl;
  bool check_result = false;
  for (std::size_t i = 0; i < ind.size(); i++) {
    if (i == 0) {
      // todo
      if (delau_triangle_type_[ind[i]].first != 2) return false;
      int start_ind = 0;
      for (int j = 0; j < 3; j++) {
        if (delau_triangle_type_[ind[i]].second[j + 3] == 0) {
          start_ind = (j + 2) % 3;
          break;
        }
      }
      for (const auto &p : edge1_) {
        double dis = (p - delau_triangle_[ind[i]][start_ind]).mold();
        if (dis < 1e-2) {
          check_result = true;
          break;
        }
      }
      if (check_result) {
        main_pt_.push_back((edge1_[0] + edge1_[1]) * 0.5);
      } else {
        main_pt_.push_back((edge2_[0] + edge2_[1]) * 0.5);
      }
      // main_pt_.push_back(delau_triangle_[ind[i]][start_ind]);
      Site middle_pt;
      middle_pt = (delau_triangle_[ind[i]][(start_ind + 1) % 3] +
                   delau_triangle_[ind[i]][(start_ind + 2) % 3]) *
                  0.5;
      main_pt_.push_back(middle_pt);
      SiteVec leftpt, rightpt;
      for (std::size_t j = 0; j < 3; j++) {
        // if (j == start_ind) continue;
        if (LeftOfLine(delau_triangle_[ind[i]][j],
                       main_pt_[main_pt_.size() - 2],
                       main_pt_[main_pt_.size() - 1])) {
          leftpt.push_back(delau_triangle_[ind[i]][j]);
        } else {
          rightpt.push_back(delau_triangle_[ind[i]][j]);
        }
        double dis = GetDisFromPtToLine(delau_triangle_[ind[i]][j],
                                        main_pt_[main_pt_.size() - 2],
                                        main_pt_[main_pt_.size() - 1]);
        if (dis < max_distance_) {
          std::cout << dis << "[]" << max_distance_ << std::endl;
          max_distance_ = dis;
        }
      }
      std::cout << "max_distance" << max_distance_ << std::endl;
      leftpt_pair_.push_back(leftpt);
      rightpt_pair_.push_back(rightpt);
    } else if (i == ind.size() - 1) {
      // todo
      if (delau_triangle_type_[ind[i]].first != 2) return false;
      int end_ind = 0;
      for (int j = 0; j < 3; j++) {
        if (delau_triangle_type_[ind[i]].second[j + 3] == 0) {
          end_ind = (j + 2) % 3;
          break;
        }
      }
      main_pt_.push_back(main_pt_.back());
      if (check_result) {
        main_pt_.push_back((edge2_[0] + edge2_[1]) * 0.5);
      } else {
        main_pt_.push_back((edge1_[0] + edge1_[1]) * 0.5);
      }
      // main_pt_.push_back(delau_triangle_[ind[i]][end_ind]);
      SiteVec leftpt, rightpt;
      for (std::size_t j = 0; j < 3; j++) {
        // if (j == end_ind) continue;
        if (LeftOfLine(delau_triangle_[ind[i]][j],
                       main_pt_[main_pt_.size() - 2],
                       main_pt_[main_pt_.size() - 1])) {
          leftpt.push_back(delau_triangle_[ind[i]][j]);
        } else {
          rightpt.push_back(delau_triangle_[ind[i]][j]);
        }
        double dis = GetDisFromPtToLine(delau_triangle_[ind[i]][j],
                                        main_pt_[main_pt_.size() - 2],
                                        main_pt_[main_pt_.size() - 1]);
        if (dis < max_distance_) {
          std::cout << dis << "[]" << max_distance_ << std::endl;
          max_distance_ = dis;
        }
        std::cout << "max_distance" << max_distance_ << std::endl;
      }
      leftpt_pair_.push_back(leftpt);
      rightpt_pair_.push_back(rightpt);
    } else {
      // todo
      if (delau_triangle_type_[ind[i]].first > 1) return false;
      if (delau_triangle_type_[ind[i]].first == 1) {
        int edge_ind;
        for (int j = 0; j < 3; j++) {
          if (delau_triangle_type_[ind[i]].second[j + 3] == 1) {
            edge_ind = j;
            break;
          }
        }
        Site middle_pt1, middle_pt2;
        middle_pt1 = (delau_triangle_[ind[i]][(edge_ind + 1) % 3] +
                      delau_triangle_[ind[i]][(edge_ind + 2) % 3]) *
                     0.5;
        middle_pt2 = (delau_triangle_[ind[i]][edge_ind % 3] +
                      delau_triangle_[ind[i]][(edge_ind + 2) % 3]) *
                     0.5;
        main_pt_.push_back(main_pt_.back());
        if ((main_pt_.back() - middle_pt1).mold() < 1e-1) {
          main_pt_.push_back(middle_pt2);
        } else {
          main_pt_.push_back(middle_pt1);
        }
      } else {
        Site middle_pt1, middle_pt2;
        std::vector<int> common;
        IsCommonEdge(delau_triangle_type_[ind[i]].second,
                     delau_triangle_type_[ind[i - 1]].second, common);
        middle_pt1 = (delau_triangle_[ind[i]][common[0]] +
                      delau_triangle_[ind[i]][common[1]]) *
                     0.5;
        IsCommonEdge(delau_triangle_type_[ind[i]].second,
                     delau_triangle_type_[ind[i + 1]].second, common);
        middle_pt2 = (delau_triangle_[ind[i]][common[0]] +
                      delau_triangle_[ind[i]][common[1]]) *
                     0.5;
        main_pt_.push_back(main_pt_.back());
        if ((main_pt_.back() - middle_pt1).mold() < 1e-1) {
          main_pt_.push_back(middle_pt2);
        } else {
          main_pt_.push_back(middle_pt1);
        }
      }
      SiteVec leftpt, rightpt;
      for (std::size_t j = 0; j < 3; j++) {
        if (LeftOfLine(delau_triangle_[ind[i]][j],
                       main_pt_[main_pt_.size() - 2],
                       main_pt_[main_pt_.size() - 1])) {
          leftpt.push_back(delau_triangle_[ind[i]][j]);
        } else {
          rightpt.push_back(delau_triangle_[ind[i]][j]);
        }
        double dis = GetDisFromPtToLine(delau_triangle_[ind[i]][j],
                                        main_pt_[main_pt_.size() - 2],
                                        main_pt_[main_pt_.size() - 1]);
        if (dis < max_distance_) max_distance_ = dis;
      }
      leftpt_pair_.push_back(leftpt);
      rightpt_pair_.push_back(rightpt);
    }
  }
  // for (std::size_t i = 0; i < main_pt_.size(); i++) {
  //   std::cout << "[main_pt_]" << main_pt_[i].x << ","
  //             << main_pt_[i].y << std::endl;
  // }
  // for (const auto &p : leftpt_pair_) {
  //   for (const auto &t : p) {
  //     std::cout << "[leftpt]" << t.x << "," << t.y << std::endl;
  //   }
  // }
  return true;
}

bool PipePlanner::SkeletonDilation() {
  // std::cout << main_pt_.size() << "[]" << leftpt_pair_.size() << "[]"
  //           << rightpt_pair_.size() << std::endl;
  // for (int i = 0; i < leftpt_pair_.size(); i++) {
  //   for (int j = 0; j < leftpt_pair_[i].size(); j++) {
  //     std::cout << "[left]" << i << ","
  //               << leftpt_pair_[i][j].x << ","
  //               << leftpt_pair_[i][j].y << std::endl;
  //   }
  // }
  int segment = ceil(max_distance_ / kCarWidth);
  for (int ind = 0; ind < segment + 1; ind++) {
    SiteVec l_single, r_single;
    for (std::size_t i = 0; i < leftpt_pair_.size(); i++) {
      if (leftpt_pair_[i].size() == 1) {
        Site p1 = (leftpt_pair_[i][0] - main_pt_[i * 2]) / segment * ind +
                  main_pt_[i * 2];
        Site p2 = (leftpt_pair_[i][0] - main_pt_[i * 2 + 1]) / segment * ind +
                  main_pt_[i * 2 + 1];
        l_single.push_back(p1);
        l_single.push_back(p2);
      } else if (leftpt_pair_[i].size() == 2) {
        if (SameDirection(main_pt_[i * 2], main_pt_[i * 2 + 1],
                          leftpt_pair_[i][0], leftpt_pair_[i][1])) {
          Site p1 = (leftpt_pair_[i][0] - main_pt_[i * 2]) / segment * ind +
                    main_pt_[i * 2];
          Site p2 = (leftpt_pair_[i][1] - main_pt_[i * 2 + 1]) / segment * ind +
                    main_pt_[i * 2 + 1];
          l_single.push_back(p1);
          l_single.push_back(p2);
        } else {
          Site p1 = (leftpt_pair_[i][1] - main_pt_[i * 2]) / segment * ind +
                    main_pt_[i * 2];
          Site p2 = (leftpt_pair_[i][0] - main_pt_[i * 2 + 1]) / segment * ind +
                    main_pt_[i * 2 + 1];
          // if (i == 0) {
          //   std::cout << "--------" << std::endl;
          //   std::cout << main_pt_[i*2].x << "," << main_pt_[i*2].y <<
          //   std::endl; std::cout << main_pt_[i*2].x << "," << main_pt_[i*2].y
          //   << std::endl; std::cout << "--------" << std::endl;
          // }
          l_single.push_back(p1);
          l_single.push_back(p2);
        }
      } else {
        // std::cout << "leftpt_pair_.size" << __LINE__ << std::endl;
        return false;
      }

      if (rightpt_pair_[i].size() == 1) {
        Site p1 = (rightpt_pair_[i][0] - main_pt_[i * 2]) / segment * ind +
                  main_pt_[i * 2];
        Site p2 = (rightpt_pair_[i][0] - main_pt_[i * 2 + 1]) / segment * ind +
                  main_pt_[i * 2 + 1];
        r_single.push_back(p1);
        r_single.push_back(p2);
      } else if (rightpt_pair_[i].size() == 2) {
        if (SameDirection(main_pt_[i * 2], main_pt_[i * 2 + 1],
                          rightpt_pair_[i][0], rightpt_pair_[i][1])) {
          Site p1 = (rightpt_pair_[i][0] - main_pt_[i * 2]) / segment * ind +
                    main_pt_[i * 2];
          Site p2 =
              (rightpt_pair_[i][1] - main_pt_[i * 2 + 1]) / segment * ind +
              main_pt_[i * 2 + 1];
          r_single.push_back(p1);
          r_single.push_back(p2);
        } else {
          Site p1 = (rightpt_pair_[i][1] - main_pt_[i * 2]) / segment * ind +
                    main_pt_[i * 2];
          Site p2 =
              (rightpt_pair_[i][0] - main_pt_[i * 2 + 1]) / segment * ind +
              main_pt_[i * 2 + 1];
          r_single.push_back(p1);
          r_single.push_back(p2);
        }
      } else {
        // std::cout << "leftpt_pair_.size" << __LINE__ << std::endl;
        return false;
      }
    }
    left_segment_.push_back(l_single);
    right_segment_.push_back(r_single);
  }
  left_segment_.erase(left_segment_.begin());
  right_segment_.erase(right_segment_.begin());
  std::reverse(left_segment_.begin(), left_segment_.end());

  // for (std::size_t i = 0; i < left_segment_.size(); i++) {
  //   for (std::size_t j = 0; j < left_segment_[i].size(); j++) {
  //     std::cout << "segment:" << i << "," << left_segment_[i][j].x << ","
  //               << left_segment_[i][j].y << std::endl;
  //   }
  // }
  // for (std::size_t i = 0; i < right_segment_.size(); i++) {
  //   for (std::size_t j = 0; j < right_segment_[i].size(); j++) {
  //     std::cout << "segment:" << right_segment_[i][j].x << ","
  //               << right_segment_[i][j].y << std::endl;
  //   }
  // }
  return true;
}

bool PipePlanner::GenerateEdgeCurve() {
  if (!GenerateFiltCurve(0)) return false;
  if (!GenerateFiltCurve(main_ind_.size() - 1)) return false;

  return true;
}

bool PipePlanner::GenerateFiltCurve(int index) {
  if (delau_triangle_type_[main_ind_[index]].first != 2) return false;
  int ind;
  for (int j = 0; j < 3; j++) {
    if (delau_triangle_type_[main_ind_[index]].second[j + 3] == 0) {
      ind = (j + 2) % 3;
      break;
    }
  }
  Site pt = delau_triangle_[main_ind_[index]][ind % 3];
  Site pt1 = delau_triangle_[main_ind_[index]][(ind + 1) % 3];
  Site pt2 = delau_triangle_[main_ind_[index]][(ind + 2) % 3];
  Site l_pt, r_pt, l_pt_unit, r_pt_unit;
  if (LeftOfLine(pt1, pt, (pt1 + pt2) * 0.5)) {
    l_pt = pt1;
    r_pt = pt2;
  } else {
    l_pt = pt2;
    r_pt = pt1;
  }
  double l_dis = (pt - l_pt).mold();
  double r_dis = (pt - r_pt).mold();
  l_pt_unit = (pt - l_pt).direction();
  r_pt_unit = (r_pt - pt).direction();
  double cross = l_pt_unit.x * r_pt_unit.y - l_pt_unit.y * r_pt_unit.x;
  double n_dot = l_pt_unit.x * r_pt_unit.x + l_pt_unit.y * r_pt_unit.y;
  double smooth_dis = std::fabs(kTurningRadius / cross * (1.0 - n_dot));
  if (smooth_dis >= l_dis || smooth_dis >= r_dis) {
    std::cout << "the edge is too small" << std::endl;
    return false;
  }
  Site start_i, end_i, cent_i;
  if (cross > 0.0) {
    start_i = pt - l_pt_unit * kTurningRadius / cross * (1.0 - n_dot);
    end_i = pt + r_pt_unit * kTurningRadius / cross * (1.0 - n_dot);
    cent_i = pt + (r_pt_unit - l_pt_unit) * kTurningRadius / cross;
  } else {
    start_i = pt + l_pt_unit * kTurningRadius / cross * (1.0 - n_dot);
    end_i = pt - r_pt_unit * kTurningRadius / cross * (1.0 - n_dot);
    cent_i = pt - (r_pt_unit - l_pt_unit) * kTurningRadius / cross;
  }
  if (index == 0) {
    ArcSampling(cent_i, start_i, end_i, cross, start_curve_);
  } else {
    ArcSampling(cent_i, start_i, end_i, cross, end_curve_);
  }
  return true;
}

bool PipePlanner::GenerateFilterCurve() {
  // auto pp_smoother = std::make_shared<optimizer::PurePursuit>();
  // pp_smoother->Init(Site(0, 0, 0));
  // SiteVec path;
  // pp_smoother->Smooth(path);
  // for (const auto &p : path) {
  //   std::cout << "[smooth]" << p.x << "," << p.y << std::endl;
  // }
  std::cout << "[radius]" << kTurningRadius << "," << max_distance_
            << std::endl;
  if (kTurningRadius > max_distance_) {
    std::cout << "there is no more space for turning" << std::endl;
    return false;
  } else if (kTurningRadius < (max_distance_ / 2.0)) {
    std::cout << "there is enough space for turning" << std::endl;
    for (int i = 0; i < left_segment_.size(); i++) {
      Site start_l = left_segment_[i].front();
      Site start_r = right_segment_[i].front();
      start_l.angle = (main_pt_[0] - main_pt_[1]).inerangle();
      start_r.angle = (main_pt_[1] - main_pt_[0]).inerangle();
      geometry::Dubins dbs;
      SiteVec temp_curve;
      dbs.GetPath(temp_curve, start_l, start_r, kTurningRadius, kSampleDis);
      FilterCurve cur_curve;
      cur_curve.cur_l_ind = i;
      cur_curve.cur_r_ind = i;
      cur_curve.l_ind = -1;
      cur_curve.r_ind = -1;
      cur_curve.l_pt = left_segment_[i].front();
      cur_curve.r_pt = right_segment_[i].front();
      cur_curve.curves.clear();
      cur_curve.curves.insert(cur_curve.curves.end(), temp_curve.begin(),
                              temp_curve.end());
      start_filter_.push_back(cur_curve);
    }
    for (int i = 0; i < right_segment_.size() - 1; i++) {
      Site end_l = left_segment_[i + 1].back();
      Site end_r = right_segment_[i].back();
      end_l.angle =
          (main_pt_[main_pt_.size() - 2] - main_pt_[main_pt_.size() - 1])
              .inerangle();
      end_r.angle =
          (main_pt_[main_pt_.size() - 1] - main_pt_[main_pt_.size() - 2])
              .inerangle();
      geometry::Dubins dbs;
      SiteVec temp_curve;
      dbs.GetPath(temp_curve, end_r, end_l, kTurningRadius, kSampleDis);
      FilterCurve cur_curve;
      cur_curve.cur_l_ind = i + 1;
      cur_curve.cur_r_ind = i;
      cur_curve.l_ind = -1;
      cur_curve.r_ind = -1;
      cur_curve.l_pt = left_segment_[i + 1].back();
      cur_curve.r_pt = right_segment_[i].back();
      cur_curve.curves.clear();
      cur_curve.curves.insert(cur_curve.curves.end(), temp_curve.begin(),
                              temp_curve.end());
      end_filter_.push_back(cur_curve);
    }
  } else {
    std::cout << "filt the curve to adapt the turning radius" << std::endl;
    int filt_num = ceil((2 * kTurningRadius - max_distance_) / kCarWidth);
    // std::cout << "filt_num:" << filt_num << "[]"
    //           << left_segment_.size() << std::endl;
    for (int i = 0; i < left_segment_.size() - filt_num; i++) {
      Site start_l = left_segment_[i].front();
      Site start_r = right_segment_[i + filt_num].front();
      start_l.angle = (main_pt_[0] - main_pt_[1]).inerangle();
      start_r.angle = (main_pt_[1] - main_pt_[0]).inerangle();
      geometry::Dubins dbs;
      SiteVec temp_curve;
      dbs.GetPath(temp_curve, start_l, start_r, kTurningRadius, kSampleDis);
      FilterCurve cur_curve;
      cur_curve.cur_l_ind = i;
      cur_curve.cur_r_ind = i + filt_num;
      cur_curve.l_ind = -1;
      cur_curve.r_ind = -1;
      cur_curve.l_pt = left_segment_[i].front();
      cur_curve.r_pt = right_segment_[i + filt_num].front();
      cur_curve.curves.clear();
      cur_curve.curves.insert(cur_curve.curves.end(), temp_curve.begin(),
                              temp_curve.end());
      start_filter_.push_back(cur_curve);
    }
    for (int i = left_segment_.size() - filt_num; i < left_segment_.size();
         i++) {
      Site start_l = left_segment_[0].front();
      Site start_r = right_segment_[filt_num].front();
      start_l.angle = (main_pt_[0] - main_pt_[1]).inerangle();
      start_r.angle = (main_pt_[1] - main_pt_[0]).inerangle();
      geometry::Dubins dbs;
      SiteVec temp_curve;
      dbs.GetPath(temp_curve, start_l, start_r, kTurningRadius, kSampleDis);
      double l_dis = kCarWidth * i;
      double r_dis = kCarWidth * (left_segment_.size() - 1 - i);
      double sample_l_dis = 0.0;
      double sample_r_dis = 0.0;
      double l_off_dis = 0.0;
      double r_off_dis = 0.0;
      bool l_flag = false;
      bool r_flag = false;
      int l_ind = 0;
      int r_ind = 0;
      for (std::size_t ind = 0; ind < main_ind_.size(); ind++) {
        double single_l_dis =
            (left_segment_[i][ind * 2] - left_segment_[i][ind * 2 + 1]).mold();
        double single_r_dis =
            (right_segment_[left_segment_.size() - 1 - i][ind * 2] -
             right_segment_[left_segment_.size() - 1 - i][ind * 2 + 1])
                .mold();
        sample_l_dis += single_l_dis;
        sample_r_dis += single_r_dis;
        if (!l_flag) {
          if (sample_l_dis > 3 * l_dis) {
            l_flag = true;
            l_ind = ind;
            l_off_dis = single_l_dis - (sample_l_dis - 3 * l_dis);
          }
        }
        if (!r_flag) {
          if (sample_r_dis > 3 * r_dis) {
            r_flag = true;
            r_ind = ind;
            r_off_dis = single_r_dis - (sample_r_dis - 3 * r_dis);
          }
        }
        if (l_flag && r_flag) break;
      }
      Site l_off_pt =
          left_segment_[i][l_ind * 2] +
          (left_segment_[i][l_ind * 2 + 1] - left_segment_[i][l_ind * 2])
                  .direction() *
              l_off_dis;
      Site r_off_pt =
          right_segment_[left_segment_.size() - 1 - i][r_ind * 2] +
          (right_segment_[left_segment_.size() - 1 - i][r_ind * 2 + 1] -
           right_segment_[left_segment_.size() - 1 - i][r_ind * 2])
                  .direction() *
              r_off_dis;
      SiteVec l_pts, r_pts;
      LineSampling(l_off_pt, left_segment_[i][l_ind * 2 + 1], l_pts);
      LineSampling(r_off_pt,
                   right_segment_[left_segment_.size() - 1 - i][r_ind * 2 + 1],
                   r_pts);
      auto pp_smoother_l = std::make_shared<optimizer::PurePursuit>();
      auto pp_smoother_r = std::make_shared<optimizer::PurePursuit>();
      start_l.angle = (main_pt_[1] - main_pt_[0]).inerangle();
      pp_smoother_l->Init(start_l);
      pp_smoother_l->Smooth(l_pts);
      pp_smoother_r->Init(start_r);
      pp_smoother_r->Smooth(r_pts);
      std::reverse(l_pts.begin(), l_pts.end());
      temp_curve.insert(temp_curve.end(), r_pts.begin(), r_pts.end());
      l_pts.insert(l_pts.end(), temp_curve.begin(), temp_curve.end());
      // start_filter_.push_back(std::make_pair(
      //                           std::make_pair(0, Site(0, 0)),
      //                           temp_curve));
      FilterCurve cur_curve;
      cur_curve.cur_l_ind = i;
      cur_curve.cur_r_ind = left_segment_.size() - 1 - i;
      cur_curve.l_ind = l_ind;
      cur_curve.r_ind = r_ind;
      cur_curve.l_pt = l_off_pt;
      cur_curve.r_pt = r_off_pt;
      cur_curve.curves.clear();
      cur_curve.curves.insert(cur_curve.curves.end(), l_pts.begin(),
                              l_pts.end());
      start_filter_.push_back(cur_curve);
    }
    for (int i = 0; i < right_segment_.size() - filt_num; i++) {
      Site end_l = left_segment_[i + 1].back();
      Site end_r = right_segment_[i + filt_num].back();
      end_l.angle =
          (main_pt_[main_pt_.size() - 2] - main_pt_[main_pt_.size() - 1])
              .inerangle();
      end_r.angle =
          (main_pt_[main_pt_.size() - 1] - main_pt_[main_pt_.size() - 2])
              .inerangle();
      geometry::Dubins dbs;
      SiteVec temp_curve;
      dbs.GetPath(temp_curve, end_r, end_l, kTurningRadius, kSampleDis);
      FilterCurve cur_curve;
      cur_curve.cur_l_ind = i + 1;
      cur_curve.cur_r_ind = i + filt_num;
      cur_curve.l_ind = -1;
      cur_curve.r_ind = -1;
      cur_curve.l_pt = left_segment_[i + 1].back();
      cur_curve.r_pt = right_segment_[i + filt_num].back();
      cur_curve.curves.clear();
      cur_curve.curves.insert(cur_curve.curves.end(), temp_curve.begin(),
                              temp_curve.end());
      end_filter_.push_back(cur_curve);
      std::cout << end_l.x << "," << end_l.y << std::endl;
      std::cout << end_r.x << "," << end_r.y << std::endl;
    }
    std::cout << "right_normal" << right_segment_.size() << "," << filt_num
              << std::endl;
    for (int i = right_segment_.size() - filt_num;
         i < (right_segment_.size() - 1); i++) {
      Site end_l = left_segment_[1].back();
      Site end_r = right_segment_[filt_num].back();
      end_l.angle =
          (main_pt_[main_pt_.size() - 2] - main_pt_[main_pt_.size() - 1])
              .inerangle();
      end_r.angle =
          (main_pt_[main_pt_.size() - 1] - main_pt_[main_pt_.size() - 2])
              .inerangle();
      geometry::Dubins dbs;
      SiteVec temp_curve;
      dbs.GetPath(temp_curve, end_r, end_l, kTurningRadius, kSampleDis);
      double l_dis = kCarWidth * (i + 1);
      double r_dis = kCarWidth * (right_segment_.size() - 1 - i);
      double sample_l_dis = 0.0;
      double sample_r_dis = 0.0;
      double l_off_dis = 0.0;
      double r_off_dis = 0.0;
      bool l_flag = false;
      bool r_flag = false;
      int l_ind = 0;
      int r_ind = 0;
      for (std::size_t ind = main_ind_.size() - 1; ind >= 0; ind--) {
        double single_l_dis =
            (left_segment_[i + 1][ind * 2] - left_segment_[i + 1][ind * 2 + 1])
                .mold();
        double single_r_dis =
            (right_segment_[right_segment_.size() - 1 - i][ind * 2] -
             right_segment_[right_segment_.size() - 1 - i][ind * 2 + 1])
                .mold();
        sample_l_dis += single_l_dis;
        sample_r_dis += single_r_dis;
        if (!l_flag) {
          if (sample_l_dis > 3 * l_dis) {
            l_flag = true;
            l_ind = ind;
            l_off_dis = single_l_dis - (sample_l_dis - 3 * l_dis);
          }
        }
        if (!r_flag) {
          if (sample_r_dis > 3 * r_dis) {
            r_flag = true;
            r_ind = ind;
            r_off_dis = single_r_dis - (sample_r_dis - 3 * r_dis);
          }
        }
        if (l_flag && r_flag) break;
      }
      Site l_off_pt = left_segment_[i + 1][l_ind * 2] +
                      (left_segment_[i + 1][l_ind * 2 + 1] -
                       left_segment_[i + 1][l_ind * 2])
                              .direction() *
                          l_off_dis;
      Site r_off_pt =
          right_segment_[right_segment_.size() - 1 - i][r_ind * 2] +
          (right_segment_[right_segment_.size() - 1 - i][r_ind * 2 + 1] -
           right_segment_[right_segment_.size() - 1 - i][r_ind * 2])
                  .direction() *
              r_off_dis;
      SiteVec l_pts, r_pts;
      LineSampling(l_off_pt, left_segment_[i + 1][l_ind * 2], l_pts);
      LineSampling(r_off_pt,
                   right_segment_[right_segment_.size() - 1 - i][r_ind * 2],
                   r_pts);
      auto pp_smoother_l = std::make_shared<optimizer::PurePursuit>();
      auto pp_smoother_r = std::make_shared<optimizer::PurePursuit>();
      end_r.angle =
          (main_pt_[main_pt_.size() - 2] - main_pt_[main_pt_.size() - 1])
              .inerangle();
      pp_smoother_l->Init(end_l);
      pp_smoother_l->Smooth(l_pts);
      pp_smoother_r->Init(end_r);
      pp_smoother_r->Smooth(r_pts);
      std::reverse(r_pts.begin(), r_pts.end());
      temp_curve.insert(temp_curve.end(), l_pts.begin(), l_pts.end());
      r_pts.insert(r_pts.end(), temp_curve.begin(), temp_curve.end());
      FilterCurve cur_curve;
      cur_curve.cur_l_ind = i + 1;
      cur_curve.cur_r_ind = right_segment_.size() - 1 - i;
      cur_curve.l_ind = l_ind;
      cur_curve.r_ind = r_ind;
      cur_curve.l_pt = l_off_pt;
      cur_curve.r_pt = r_off_pt;
      cur_curve.curves.clear();
      cur_curve.curves.insert(cur_curve.curves.end(), r_pts.begin(),
                              r_pts.end());
      end_filter_.push_back(cur_curve);
    }
    // std::cout << "right_abnormal" << std::endl;
  }
  // std::cout << "filter_size:" << start_filter_.size() << ","
  //           << end_filter_.size() << std::endl;
  return true;
}

bool PipePlanner::PathConnect() {
  int circle_size = start_filter_.size();
  if (circle_size < 2) return false;
  for (int i = 0; i < circle_size - 1; i++) {
    int r_start_ind = 0;
    int l_start_ind = 0;
    int r_end_ind = 0;
    int l_end_ind = 0;
    int next_l_end_ind = 0;

    l_start_ind = start_filter_[i].l_ind < 0 ? 0 : start_filter_[i].l_ind + 1;
    r_start_ind = start_filter_[i].r_ind < 0 ? 0 : start_filter_[i].r_ind + 1;
    if (end_filter_[i].l_ind < 0) {
      l_end_ind = right_segment_[end_filter_[i].cur_l_ind].size() / 2;
    } else {
      l_end_ind = end_filter_[i].l_ind;
    }
    if (end_filter_[i].r_ind < 0) {
      r_end_ind = right_segment_[end_filter_[i].cur_r_ind].size() / 2;
    } else {
      r_end_ind = end_filter_[i].r_ind;
    }
    if (i == 0) {
      next_l_end_ind = left_segment_[0].size() / 2;
    } else {
      if (end_filter_[i - 1].l_ind < 0) {
        next_l_end_ind =
            right_segment_[end_filter_[i - 1].cur_l_ind].size() / 2;
      } else {
        next_l_end_ind = end_filter_[i - 1].l_ind;
      }
    }
    // std::cout << "test:"<< start_filter_[i].cur_r_ind << ","
    //           << end_filter_[i].cur_r_ind << ","
    //           << end_filter_[i].cur_l_ind << ","
    //           << start_filter_[i].cur_l_ind << std::endl;
    if (start_filter_[i].cur_r_ind != end_filter_[i].cur_r_ind ||
        (end_filter_[i].cur_l_ind - start_filter_[i].cur_l_ind) != 1) {
      std::cout << __LINE__ << std::endl;
      return false;
    }
    if (l_start_ind >= next_l_end_ind) {
      std::cout << __LINE__ << std::endl;
      return false;
    }
    if (r_start_ind >= r_end_ind) {
      std::cout << __LINE__ << std::endl;
      return false;
    }
    // filt the left_line
    SiteVec left_line;
    for (int j = l_start_ind; j < next_l_end_ind; j++) {
      Site p1 = left_segment_[start_filter_[i].cur_l_ind][j * 2];
      Site p2 = left_segment_[start_filter_[i].cur_l_ind][j * 2 + 1];
      LineSampling(p1, p2, left_line);
    }
    std::reverse(left_line.begin(), left_line.end());
    // filt the left curve
    left_line.insert(left_line.end(), start_filter_[i].curves.begin(),
                     start_filter_[i].curves.end());
    // filt the right line
    for (int j = r_start_ind; j < r_end_ind; j++) {
      Site p1 = right_segment_[start_filter_[i].cur_r_ind][j * 2];
      Site p2 = right_segment_[start_filter_[i].cur_r_ind][j * 2 + 1];
      LineSampling(p1, p2, left_line);
    }
    // filt the right curve
    left_line.insert(left_line.end(), end_filter_[i].curves.begin(),
                     end_filter_[i].curves.end());
    test_path_.push_back(left_line);
  }
  int l_start = start_filter_[circle_size - 1].l_ind < 0
                    ? 0
                    : start_filter_[circle_size - 1].l_ind + 1;
  int r_start = start_filter_[circle_size - 1].r_ind < 0
                    ? 0
                    : start_filter_[circle_size - 1].r_ind + 1;
  int r_end = right_segment_[0].size() / 2;
  int next_l_end;
  if (end_filter_[circle_size - 2].l_ind < 0) {
    next_l_end =
        right_segment_[end_filter_[circle_size - 2].cur_l_ind].size() / 2;
  } else {
    next_l_end = end_filter_[circle_size - 2].l_ind;
  }
  // int next_l_end =
  // right_segment_[end_filter_[circle_size-2].cur_l_ind].size() / 2;
  SiteVec tmp_line;
  for (int j = l_start; j < next_l_end; j++) {
    Site p1 = left_segment_[start_filter_[circle_size - 1].cur_l_ind][j * 2];
    Site p2 =
        left_segment_[start_filter_[circle_size - 1].cur_l_ind][j * 2 + 1];
    LineSampling(p1, p2, tmp_line);
  }
  std::reverse(tmp_line.begin(), tmp_line.end());
  // filt the left curve
  tmp_line.insert(tmp_line.end(), start_filter_[circle_size - 1].curves.begin(),
                  start_filter_[circle_size - 1].curves.end());
  // filt the right line
  for (int j = r_start; j < r_end; j++) {
    Site p1 = right_segment_[start_filter_[circle_size - 1].cur_r_ind][j * 2];
    Site p2 =
        right_segment_[start_filter_[circle_size - 1].cur_r_ind][j * 2 + 1];
    LineSampling(p1, p2, tmp_line);
  }
  test_path_.push_back(tmp_line);
  return true;
}

bool PipePlanner::PathSmooth() {
  for (const auto &p : test_path_) {
    final_path_.insert(final_path_.end(), p.begin(), p.end());
  }
  if (final_path_.size() < 2) return false;
  auto pp_smoother_l = std::make_shared<optimizer::PurePursuit>();
  final_path_[0].angle = (final_path_[1] - final_path_[0]).inerangle();
  pp_smoother_l->Init(final_path_[0]);
  pp_smoother_l->Smooth(final_path_);
  return true;
}

void PipePlanner::Recovery(geometry::SiteVec &path) {
  for (std::size_t i = 0; i < path.size(); i++) {
    path[i].x = path[i].x + x_offset_;
    path[i].y = path[i].y + y_offset_;
  }
}

bool PipePlanner::LineSampling(const geometry::Site &start,
                               const geometry::Site &end,
                               std::vector<geometry::Site> &sample) {
  double dis = (start - end).mold();
  geometry::Site n_sample = (end - start).direction();
  double sample_line = 0.0;
  while (sample_line < dis) {
    geometry::Site temp;
    temp.x = start.x + n_sample.x * sample_line;
    temp.y = start.y + n_sample.y * sample_line;
    sample.push_back(temp);
    sample_line += kSampleDis;
  }
  return true;
}

bool PipePlanner::ArcSampling(const geometry::Site &cent,
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
  double single = kSampleDis / kTurningRadius;
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
      double x = kTurningRadius * cos(cur_rad);
      double y = kTurningRadius * sin(cur_rad);
      temp.x = cent.x + x;
      temp.y = cent.y + y;
    } else {
      double cur_rad = start_rad - sample_rad;
      // std::cout << "cur_rad" << cur_rad << std::endl;
      cur_rad = cur_rad < -M_PI ? cur_rad + 2 * M_PI : cur_rad;
      double x = kTurningRadius * cos(cur_rad);
      double y = kTurningRadius * sin(cur_rad);
      temp.x = cent.x + x;
      temp.y = cent.y + y;
      // std::cout << "[circle]" << temp.x << "," << temp.y << std::endl;
    }
    sample.push_back(temp);
    sample_rad += single;
  }
  return true;
}

// basic math tool
Site PipePlanner::Incenter(const Site &a, const Site &b, const Site &c) {
  Site p;
  double A = (b - c).mold();
  double B = (a - c).mold();
  double C = (a - b).mold();
  double S = A + B + C;
  double x = (A * a.x + B * b.x + C * c.x) / S;
  double y = (A * a.y + B * b.y + C * c.y) / S;
  p.x = x;
  p.y = y;
  return p;
}

bool PipePlanner::LeftOfLine(const Site &p, const Site &p1, const Site &p2) {
  double tmp = (p1 - p).x * (p2 - p).y - (p1 - p).y * (p2 - p).x;
  if (tmp > 0) return true;
  return false;
}

double PipePlanner::GetDisFromPtToLine(const geometry::Site &pt,
                                       const geometry::Site &p1,
                                       const geometry::Site &p2) {
  double a = (pt.x - p1.x) * (p2.y - p1.y) + (pt.y - p1.y) * (p1.x - p2.x);
  double b = (p2.y - p1.y) * (p2.y - p1.y) + (p1.x - p2.x) * (p1.x - p2.x);
  double result = sqrt(pow(a, 2) / b);
  return result;
}

bool PipePlanner::SameDirection(const geometry::Site &p1,
                                const geometry::Site &p2,
                                const geometry::Site &p3,
                                const geometry::Site &p4) {
  Site t1 = (p1 - p2).direction();
  Site t2 = (p3 - p4).direction();
  if ((t1 - t2).mold() < 1e-2) return true;
  // double result = (p1.x-p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x);
  // std::cout << result << std::endl;
  // if (std::fabs(result) < 1e-2) return true;
  return false;
}

bool PipePlanner::CheckCrossOfLines(const geometry::Site &p1,
                                    const geometry::Site &p2,
                                    const geometry::Site &p3,
                                    const geometry::Site &p4) {
  if (std::max(p1.x, p2.x) < std::min(p3.x, p4.x)) return false;
  if (std::max(p1.y, p2.y) < std::min(p3.y, p4.y)) return false;
  if (std::max(p3.x, p4.x) < std::min(p1.x, p2.x)) return false;
  if (std::max(p3.y, p4.y) < std::min(p1.y, p2.y)) return false;

  if (Mult(p3, p1, p4) * Mult(p3, p2, p4) < 0 &&
      Mult(p1, p3, p2) * Mult(p1, p4, p2) < 0)
    return true;
  return false;
}

double PipePlanner::Mult(const geometry::Site &p0, const geometry::Site &p1,
                         const geometry::Site &p2) {
  return (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
}

bool PipePlanner::IfAutoclockwise(const SiteVec &poly) {
  double x_max = std::numeric_limits<double>::min();
  int right_index = 0;
  // find the rightest point
  for (std::size_t i = 0; i < poly.size(); i++) {
    if (x_max < poly[i].x) {
      x_max = poly[i].x;
      right_index = i;
    }
  }
  int before_index = right_index < 1 ? poly.size() - 1 : right_index - 1;
  int after_index = right_index > poly.size() - 2 ? 0 : right_index + 1;
  double cross_product = (poly[right_index].x - poly[before_index].x) *
                             (poly[after_index].y - poly[right_index].y) -
                         (poly[right_index].y - poly[before_index].y) *
                             (poly[after_index].x - poly[right_index].x);
  if (cross_product > 0.0) return true;
  return false;
}

Site PipePlanner::GetCrossPt(const geometry::Site &p1, const geometry::Site &p2,
                             const geometry::Site &p3,
                             const geometry::Site &p4) {
  geometry::Site pt;
  geometry::Site delta1(p2.x - p1.x, -(p4.x - p3.x));
  geometry::Site delta2(p2.y - p1.y, -(p4.y - p3.y));
  geometry::Site lamda1(p3.x - p1.x, -(p4.x - p3.x));
  geometry::Site lamda2(p3.y - p1.y, -(p4.y - p3.y));
  // geometry::Site mue1(p2.x - p1.x, p3.x - p1.x);
  // geometry::Site mue2(p2.y - p1.y, p3.y - p1.y);
  double delta = Mult(delta1, delta2);
  double lamda = Mult(lamda1, lamda2) / delta;
  // double mue = Mult(mue1, mue2) / delta;
  pt.x = p1.x + delta * (p2.x - p1.x);
  pt.y = p1.y + delta * (p2.y - p2.y);
  return pt;
}

double PipePlanner::Mult(const geometry::Site &p1, const geometry::Site &p2) {
  return p1.x * p2.y - p1.y * p2.x;
}

}  // namespace coverage
