#include "pathplanner/blanket/ringplanner.h"
#include "pathplanner/delaunay/delaunay.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "optimizer/pure_pursuit.h"

namespace coverage {

bool RingPlanner::Interface(const double &sx, const double &sy,
                            const double &syaw, const double &dx,
                            const double &dy, const double &dyaw,
                            const std::vector<double> &x,
                            const std::vector<double> &y) {
  if (!EdgeSizeValid(x, y)) return false;
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
  if (!Edges2Circles()) return false;
  Dilating(orig_inner_edge_, inner_edge_);
  // std::cout << __LINE__ << std::endl;
  // 3. set the start_point and the end_point
  start_.x = sx - x_offset_;
  start_.y = sy - y_offset_;
  start_.angle = syaw * 180.0 / M_PI;
  end_.x = dx - x_offset_;
  end_.y = dy - y_offset_;
  end_.angle = dyaw * 180.0 / M_PI;
  // if (!PointValid()) return false;
  return true;
}

bool RingPlanner::Planning() {
  polygonlist erode_poly;
  polygonlist dilate_poly;
  polygonlist connect_poly;
  PolygonEroding(erode_poly);
  if (erode_poly.size() < 2) return false;
  ConvertPoly(erode_poly);
  PolygonDilating(erode_poly.back(), dilate_poly);
  ConvertPoly(dilate_poly);
  std::reverse(dilate_poly.begin(), dilate_poly.end());

  polygonlist linepoly;
  ReconstructPoly(erode_poly, linepoly);
  for (int i = 0; i < linepoly.size(); i++) {
    if (i % 2 == 0) {
      test_poly_.insert(test_poly_.end(), linepoly[i].begin(),
                        linepoly[i].end());
    } else {
      test_poly_.insert(test_poly_.end(), linepoly[i].rbegin(),
                        linepoly[i].rend());
    }
  }
  // if (debug_mode) CurveConnecting(dilate_poly);

  Out2Inner(erode_poly, dilate_poly, connect_poly);
  CurveConnecting(connect_poly);
  std::vector<geometry::Site> sample_pts;
  if (!test_poly_interpolation_.empty())
    final_result_.insert(final_result_.end(), test_poly_interpolation_.begin(),
                         test_poly_interpolation_.end());
  AverageSimpling(sample_pts, final_result_);
  GenerateCurve(sample_pts, final_path_);
  PtInterpolation();
  sample_pts.clear();
  AverageSimpling(sample_pts, test_poly_interpolation_);
  GenerateCurve(sample_pts, final_path_);
  // Recovery(final_path_);
  return true;
}

bool RingPlanner::ReconstructPoly(const polygonlist &orig_poly,
                                  polygonlist &poly) {
  if (orig_poly.size() < 1) return false;
  for (int i = 0; i < orig_poly.front().size(); i++) {
    polygonsite new_poly;
    poly.push_back(new_poly);
    for (int j = 1; j < orig_poly.size(); j++) {
      if (orig_poly[j].size() > i) {
        poly.back().push_back(orig_poly[j][i]);
      }
    }
  }
  for (std::size_t i = 0; i < poly.size(); i++) {
    double break_ind = 0;
    if (poly[i].size() < 3) continue;
    for (std::size_t j = 1; j < poly[i].size() - 1; j++) {
      double bf_angle = (poly[i][j] - poly[i][j - 1]).inerangle();
      double af_angle = (poly[i][j + 1] - poly[i][j]).inerangle();
      if (std::fabs(bf_angle - af_angle) > 10) {
        break_ind = j + 1;
        break;
      }
    }
    int poly_count = poly[i].size();
    for (int k = break_ind; k < poly_count; k++) {
      poly[i].pop_back();
    }
  }
  return true;
}

bool RingPlanner::AverageSimpling(geometry::SiteVec &sample_point,
                                  const geometry::SiteVec &final_result) {
  int point_size = final_result.size();
  int i = 0;
  for (i = 0; i < point_size; i += 6) {
    sample_point.push_back(final_result[i]);
  }
  if (i < point_size + 6) {
    sample_point.push_back(final_result.back());
  }
  return true;
}

bool RingPlanner::GenerateCurve(const geometry::SiteVec &source_data,
                                geometry::SiteVec &return_data) {
  // return_data.clear();

  int i, j;
  float t;
  geometry::Site tmp_point;  //插值点坐标
  float g[4][4], g1[4][4], g2[4][4];  // region 创建并计算G(t)及其一阶及二阶导数

  if (source_data.size() >= 4) {
    for (j = 0; j < 4; j++) {
      t = j / 4.0;
      g[0][j] = (-t * t * t + 3 * t * t - 3 * t + 1) / 6;
      g[1][j] = (3 * t * t * t - 6 * t * t + 4) / 6;
      g[2][j] = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
      g[3][j] = t * t * t / 6;

      g1[0][j] = (-t * t + 2 * t - 1) / 2;
      g1[1][j] = (3 * t * t - 4 * t) / 2;
      g1[2][j] = (-3 * t * t + 2 * t + 1) / 2;
      g1[3][j] = t * t / 2;

      g2[0][j] = -t + 1;
      g2[1][j] = 3 * t - 2;
      g2[2][j] = -3 * t + 1;
      g2[3][j] = t;
    }

    // 为保证经过起点，在点集开始加上两点

    for (j = 0; j < 4; j++) {
      tmp_point.x =
          g[0][j] * source_data.at(0).x + g[1][j] * source_data.at(0).x +
          g[2][j] * source_data.at(0).x + g[3][j] * source_data.at(1).x;
      tmp_point.y =
          g[0][j] * source_data.at(0).y + g[1][j] * source_data.at(0).y +
          g[2][j] * source_data.at(0).y + g[3][j] * source_data.at(1).y;
      //        tmp_point.x = g[0][j] * opts[0][0] + g[1][j] * opts[0][0] +
      //        g[2][j] * opts[0][0] + g[3][j] * opts[1][0]; tmp_point.y =
      //        g[0][j] * opts[0][1] + g[1][j] * opts[0][1] + g[2][j] *
      //        opts[0][1] + g[3][j] * opts[1][1]; x1 = g1[0][j] * opts[0][0] +
      //        g1[1][j] * opts[0][0] + g1[2][j] * opts[0][0] + g1[3][j] *
      //        opts[1][0]; y1 = g1[0][j] * opts[0][1] + g1[1][j] * opts[0][1] +
      //        g1[2][j] * opts[0][1] + g1[3][j] * opts[1][1]; x2 = g2[0][j] *
      //        opts[0][0] + g2[1][j] * opts[0][0] + g2[2][j] * opts[0][0] +
      //        g2[3][j] * opts[1][0]; y2 = g2[0][j] * opts[0][1] + g2[1][j] *
      //        opts[0][1] + g2[2][j] * opts[0][1] + g2[3][j] * opts[1][1];
      //        ls.Add(new double[] { x, y, a });
      return_data.push_back(tmp_point);
    }
    for (j = 0; j < 4; j++) {
      tmp_point.x =
          g[0][j] * source_data.at(0).x + g[1][j] * source_data.at(0).x +
          g[2][j] * source_data.at(1).x + g[3][j] * source_data.at(2).x;
      tmp_point.y =
          g[0][j] * source_data.at(0).y + g[1][j] * source_data.at(0).y +
          g[2][j] * source_data.at(1).y + g[3][j] * source_data.at(2).y;
      //        x = g[0][j] * opts[0][0] + g[1][j] * opts[0][0] + g[2][j] *
      //        opts[1][0] + g[3][j] * opts[2][0]; y = g[0][j] * opts[0][1] +
      //        g[1][j] * opts[0][1] + g[2][j] * opts[1][1] + g[3][j] *
      //        opts[2][1]; x1 = g1[0][j] * opts[0][0] + g1[1][j] * opts[0][0] +
      //        g1[2][j] * opts[1][0] + g1[3][j] * opts[2][0]; y1 = g1[0][j] *
      //        opts[0][1] + g1[1][j] * opts[0][1] + g1[2][j] * opts[1][1] +
      //        g1[3][j] * opts[2][1]; x2 = g2[0][j] * opts[0][0] + g2[1][j] *
      //        opts[0][0] + g2[2][j] * opts[1][0] + g2[3][j] * opts[2][0]; y2 =
      //        g2[0][j] * opts[0][1] + g2[1][j] * opts[0][1] + g2[2][j] *
      //        opts[1][1] + g2[3][j] * opts[2][1]; ls.Add(new double[] { x, y,
      //        a });
      return_data.push_back(tmp_point);
    }

    //    iPointXY1 =  m_gNavRoute.begin();
    //    source_data.push_front(*iPointXY1);

    for (i = 0; i < source_data.size() - 3; i++) {
      for (j = 0; j < 4; j++) {
        tmp_point.x = g[0][j] * source_data.at(i).x +
                      g[1][j] * source_data.at(i + 1).x +
                      g[2][j] * source_data.at(i + 2).x +
                      g[3][j] * source_data.at(i + 3).x;
        tmp_point.y = g[0][j] * source_data.at(i).y +
                      g[1][j] * source_data.at(i + 1).y +
                      g[2][j] * source_data.at(i + 2).y +
                      g[3][j] * source_data.at(i + 3).y;
        //            x = g[0][j] * opts[i][0] + g[1][j] * opts[i + 1][0]
        //                    + g[2][j] * opts[i + 2][0] + g[3][j] * opts[i +
        //                    3][0];
        //            y = g[0][j] * opts[i][1] + g[1][j] * opts[i + 1][1]
        //                    + g[2][j] * opts[i + 2][1] + g[3][j] * opts[i +
        //                    3][1];
        //            x1 = g1[0][j] * opts[i][0] + g1[1][j] * opts[i + 1][0]
        //                    + g1[2][j] * opts[i + 2][0] + g1[3][j] * opts[i +
        //                    3][0];
        //            y1 = g1[0][j] * opts[i][1] + g1[1][j] * opts[i + 1][1]
        //                    + g1[2][j] * opts[i + 2][1] + g1[3][j] * opts[i +
        //                    3][1];
        //            x2 = g2[0][j] * opts[i][0] + g2[1][j] * opts[i + 1][0]
        //                    + g2[2][j] * opts[i + 2][0] + g2[3][j] * opts[i +
        //                    3][0];
        //            y2 = g2[0][j] * opts[i][1] + g2[1][j] * opts[i + 1][1]
        //                    + g2[2][j] * opts[i + 2][1] + g2[3][j] * opts[i +
        //                    3][1];
        //            ls.Add(new double[] { x, y, a });
        return_data.push_back(tmp_point);
      }
    }

    // 为保证经过终点，在点集结尾加上两点
    int endNum = source_data.size();
    for (j = 0; j < 4; j++) {
      tmp_point.x = g[0][j] * source_data.at(endNum - 3).x +
                    g[1][j] * source_data.at(endNum - 2).x +
                    g[2][j] * source_data.at(endNum - 1).x +
                    g[3][j] * source_data.at(endNum - 1).x;
      tmp_point.y = g[0][j] * source_data.at(endNum - 3).y +
                    g[1][j] * source_data.at(endNum - 2).y +
                    g[2][j] * source_data.at(endNum - 1).y +
                    g[3][j] * source_data.at(endNum - 1).y;
      //        x = g[0][j] * opts[opts.Count - 3][0] + g[1][j] *
      //        opts[opts.Count - 2][0]
      //                + g[2][j] * opts[opts.Count - 1][0] + g[3][j] *
      //                opts[opts.Count - 1][0];
      //        y = g[0][j] * opts[opts.Count - 3][1] + g[1][j] *
      //        opts[opts.Count - 2][1]
      //                + g[2][j] * opts[opts.Count - 1][1] + g[3][j] *
      //                opts[opts.Count - 1][1];
      //        x1 = g1[0][j] * opts[opts.Count - 3][0] + g1[1][j] *
      //        opts[opts.Count - 2][0]
      //                + g1[2][j] * opts[opts.Count - 1][0] + g1[3][j] *
      //                opts[opts.Count - 1][0];
      //        y1 = g1[0][j] * opts[opts.Count - 3][1] + g1[1][j] *
      //        opts[opts.Count - 2][1]
      //                + g1[2][j] * opts[opts.Count - 1][1] + g1[3][j] *
      //                opts[opts.Count - 1][1];
      //        x2 = g2[0][j] * opts[opts.Count - 3][0] + g2[1][j] *
      //        opts[opts.Count - 2][0]
      //                + g2[2][j] * opts[opts.Count - 1][0] + g2[3][j] *
      //                opts[opts.Count - 1][0];
      //        y2 = g2[0][j] * opts[opts.Count - 3][1] + g2[1][j] *
      //        opts[opts.Count - 2][1]
      //                + g2[2][j] * opts[opts.Count - 1][1] + g2[3][j] *
      //                opts[opts.Count - 1][1];
      //        ls.Add(new double[] { x, y, a });
      return_data.push_back(tmp_point);
    }
    for (j = 0; j < 4; j++) {
      tmp_point.x = g[0][j] * source_data.at(endNum - 2).x +
                    g[1][j] * source_data.at(endNum - 1).x +
                    g[2][j] * source_data.at(endNum - 1).x +
                    g[3][j] * source_data.at(endNum - 1).x;
      tmp_point.y = g[0][j] * source_data.at(endNum - 2).y +
                    g[1][j] * source_data.at(endNum - 1).y +
                    g[2][j] * source_data.at(endNum - 1).y +
                    g[3][j] * source_data.at(endNum - 1).y;
      //        x = g[0][j] * opts[opts.Count - 2][0] + g[1][j] *
      //        opts[opts.Count - 1][0]
      //                + g[2][j] * opts[opts.Count - 1][0] + g[3][j] *
      //                opts[opts.Count - 1][0];
      //        y = g[0][j] * opts[opts.Count - 2][1] + g[1][j] *
      //        opts[opts.Count - 1][1]
      //                + g[2][j] * opts[opts.Count - 1][1] + g[3][j] *
      //                opts[opts.Count - 1][1];
      //        x1 = g1[0][j] * opts[opts.Count - 2][0] + g1[1][j] *
      //        opts[opts.Count - 1][0]
      //                + g1[2][j] * opts[opts.Count - 1][0] + g1[3][j] *
      //                opts[opts.Count - 1][0];
      //        y1 = g1[0][j] * opts[opts.Count - 2][1] + g1[1][j] *
      //        opts[opts.Count - 1][1]
      //                + g1[2][j] * opts[opts.Count - 1][1] + g1[3][j] *
      //                opts[opts.Count - 1][1];
      //        x2 = g2[0][j] * opts[opts.Count - 2][0] + g2[1][j] *
      //        opts[opts.Count - 1][0]
      //                + g2[2][j] * opts[opts.Count - 1][0] + g2[3][j] *
      //                opts[opts.Count - 1][0];
      //        y2 = g2[0][j] * opts[opts.Count - 2][1] + g2[1][j] *
      //        opts[opts.Count - 1][1]
      //                + g2[2][j] * opts[opts.Count - 1][1] + g2[3][j] *
      //                opts[opts.Count - 1][1];
      //        ls.Add(new double[] { x, y, a });
      return_data.push_back(tmp_point);
    }
    // std::cout << "[true]" << return_data.size() << std::endl;
    return true;
  } else {
    std::cout << "[false]" << source_data.size() << std::endl;
    return false;
  }
  // return true;
}

void RingPlanner::Recovery(geometry::SiteVec &path) {
  for (std::size_t i = 0; i < path.size(); i++) {
    path[i].x = path[i].x + x_offset_;
    path[i].y = path[i].y + y_offset_;
  }
}

bool RingPlanner::ConvertPoly(polygonlist &erode_poly) {
  for (int i = 0; i < erode_poly.size() - 1; i++) {
    geometry::Site begin_i = erode_poly[i][0];
    double dis_min = std::numeric_limits<double>::max();
    int temp = 0;
    for (int j = 0; j < erode_poly[i + 1].size(); j++) {
      double dis = (begin_i - erode_poly[i + 1][j]).mold();
      if (dis_min > dis) {
        temp = j;
        dis_min = dis;
      }
    }
    auto temp_it = erode_poly[i + 1].begin();
    for (int k = 0; k < temp; k++) {
      erode_poly[i + 1].push_back(erode_poly[i + 1][k]);
    }
    for (int k = 0; k < temp; k++) {
      erode_poly[i + 1].erase(erode_poly[i + 1].begin());
    }
  }
  // for (int i = 0; i < erode_poly.size(); i++) {
  //   for (int j = 0; j < erode_poly[i].size(); j++) {
  //     std::cout << "[jf]" << erode_poly[i][j].x << ","
  //               << erode_poly[i][j].y << std::endl;
  //   }
  // }
  return true;
}

bool RingPlanner::Out2Inner(polygonlist &erode_poly, polygonlist &dilate_poly,
                            polygonlist &connect) {
  std::vector<std::pair<geometry::Site, int>> cross_pts;
  int main_index = 0;
  geometry::Site out_end = erode_poly.back().back();
  geometry::Site out_start = erode_poly.back()[erode_poly.back().size() - 2];
  polygonsite check_inpolygon = dilate_poly.front();
  for (int i = 0; i < check_inpolygon.size(); i++) {
    int next_i = i == check_inpolygon.size() - 1 ? 0 : i + 1;
    if (Mult(out_start, check_inpolygon[i], out_end) *
            Mult(out_start, check_inpolygon[next_i], out_end) <
        0) {
      cross_pts.push_back(
          std::make_pair(GetCrossPt(out_start, out_end, check_inpolygon[i],
                                    check_inpolygon[next_i], true),
                         i));
    }
  }
  geometry::Site normal_dir = out_end - out_start;
  for (std::size_t i = 0; i < cross_pts.size(); i++) {
    geometry::Site cur_dir = cross_pts[i].first - out_start;
    if (normal_dir.x * cur_dir.x + normal_dir.y * cur_dir.y > 0) {
      main_index = i;
      break;
    }
  }

  erode_poly.back().pop_back();
  for (std::size_t i = 0; i < erode_poly.size(); i++) {
    connect.push_back(erode_poly[i]);
  }
  polygonsite temp;
  connect.push_back(temp);
  connect.back().push_back(cross_pts[main_index].first);
  for (std::size_t i = cross_pts[main_index].second + 1;
       i < dilate_poly.front().size(); i++) {
    connect.back().push_back(dilate_poly.front()[i]);
  }
  for (std::size_t i = 1; i < dilate_poly.size(); i++) {
    connect.push_back(dilate_poly[i]);
  }
  return true;
}

bool RingPlanner::EdgeSizeValid(const std::vector<double> &x,
                                const std::vector<double> &y) {
  if (x.size() == 0 || y.size() == 0) return false;
  if (x.size() != y.size()) return false;
  return true;
}

bool RingPlanner::PointValid() {
  if (!IsInSide(start_, orig_outer_edge_)) return false;
  if (!IsInSide(end_, orig_outer_edge_)) return false;
  if (IsInSide(start_, orig_inner_edge_)) return false;
  if (IsInSide(end_, orig_inner_edge_)) return false;
  return true;
}

bool RingPlanner::IsInSide(const geometry::Site &pt,
                           const polygonsite &polygon) {
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

bool RingPlanner::Edges2Circles() {
  if (orig_edge_.size() < 8) return false;
  // if (debug_mode) {
  //   for (const auto &p : orig_edge_) {
  //     std::cout << "orig:" << p.x << "," << p.y << std::endl;
  //   }
  // }
  std::vector<int> neigh_pt;
  FindNeighPt(neigh_pt);
  polygonsite edge1, edge2;
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
  return true;
}

void RingPlanner::FindNeighPt(std::vector<int> &neigh) {
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

bool RingPlanner::CircleClassification(const polygonsite &edge1,
                                       const polygonsite &edge2) {
  if (edge1.size() < 3 || edge2.size() < 3) return false;
  if (IsInSide(edge1[0], edge2)) {
    // the edge1 is the inner edge, the edge2 is the outer edge
    for (const auto &p : edge2) {
      if (IsInSide(p, edge1)) return false;
    }
    // inner_edge_.swap(edge1);
    // outer_edge_.swap(edge2);
    for (const auto &p : edge1) orig_inner_edge_.push_back(p);
    for (const auto &p : edge2) outer_edge_.push_back(p);
    for (const auto &p : edge2) orig_outer_edge_.push_back(p);
  } else {
    // the edge2 is the inner edge, the edge1 is the outer edge
    for (const auto &p : edge2) {
      if (!IsInSide(p, edge1)) return false;
    }
    // inner_edge_.swap(edge2);
    // outer_edge_.swap(edge1);
    for (const auto &p : edge2) orig_inner_edge_.push_back(p);
    for (const auto &p : edge1) outer_edge_.push_back(p);
    for (const auto &p : edge1) orig_outer_edge_.push_back(p);
  }
  if (!IfAutoclockwise(orig_inner_edge_))
    std::reverse(orig_inner_edge_.begin(), orig_inner_edge_.end());
  if (!IfAutoclockwise(orig_outer_edge_)) {
    std::reverse(orig_outer_edge_.begin(), orig_outer_edge_.end());
    std::reverse(outer_edge_.begin(), outer_edge_.end());
  }
  return true;
}

bool RingPlanner::IfAutoclockwise(const polygonsite &poly) {
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

void RingPlanner::PolygonEroding(polygonlist &poly) {
  while (1) {
    polygonsite orig_poly, generated_poly;
    std::vector<bool> poly_result;
    if (poly.empty()) {
      for (auto &p : outer_edge_) {
        orig_poly.push_back(p);
        poly_result.push_back(false);
      }
    } else {
      for (const auto &p : poly.back()) {
        orig_poly.push_back(p);
        poly_result.push_back(false);
      }
    }
    int cross_result = CheckInner(orig_poly, inner_edge_, poly_result);
    if (cross_result == 0) {
      Eroding(orig_poly, generated_poly);
      // std::cout << __LINE__ << std::endl;
    } else if (cross_result < poly_result.size()) {
      Eroding(orig_poly, generated_poly, poly_result);
      // std::cout << __LINE__ << std::endl;
    } else {
      // std::cout << __LINE__ << std::endl;
      return;
    }
    poly.push_back(generated_poly);
  }
  return;
}

void RingPlanner::PolygonDilating(const polygonsite &inner_poly,
                                  polygonlist &poly) {
  // todo(jf)
  while (1) {
    polygonsite orig_poly, generated_poly;
    std::vector<bool> poly_result;
    if (poly.empty()) {
      for (const auto &p : orig_inner_edge_) {
        orig_poly.push_back(p);
        poly_result.push_back(false);
      }
    } else {
      for (const auto &p : poly.back()) {
        orig_poly.push_back(p);
        poly_result.push_back(false);
      }
    }
    int cross_result = CheckCollision(orig_poly, outer_edge_, poly_result);
    if (cross_result > 0) return;
    int outer_result = CheckOuter(orig_poly, inner_poly, poly_result);
    if (outer_result == 0) {
      Dilating(orig_poly, generated_poly);
    } else if (outer_result < poly_result.size()) {
      Dilating(orig_poly, generated_poly, poly_result);
    } else {
      return;
    }
    poly.push_back(generated_poly);
  }
  return;
}

void RingPlanner::CurveConnecting(const polygonlist &polys) {
  polygonsite poly;
  final_polygon_.clear();
  for (int i = 0; i < polys.size(); i++) {
    for (int j = 0; j < polys[i].size(); j++) {
      poly.push_back(polys[i][j]);
      final_polygon_.push_back(polys[i][j]);
    }
  }
  edgelist edgeset, n_edgeset;
  int count = poly.size();
  for (int i = 0; i < count - 1; i++) {
    int next = i + 1;
    geometry::Site temp = poly[next] - poly[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  std::vector<std::vector<geometry::Site>> curve;
  std::vector<std::vector<geometry::Site>> line;
  for (int i = 1; i < count - 1; i++) {
    std::vector<geometry::Site> new_curve;
    std::vector<geometry::Site> new_line;
    int start = i - 1;
    double cross_product = n_edgeset[start].x * n_edgeset[i].y -
                           n_edgeset[start].y * n_edgeset[i].x;
    double n_dot = (n_edgeset[start].x * n_edgeset[i].x +
                    n_edgeset[start].y * n_edgeset[i].y);
    geometry::Site start_i = poly[i] - n_edgeset[start] * turning_radius /
                                           cross_product * (1.0 - n_dot);
    geometry::Site end_i =
        poly[i] + n_edgeset[i] * turning_radius / cross_product * (1.0 - n_dot);
    geometry::Site cent_i = poly[i] + (n_edgeset[i] - n_edgeset[start]) *
                                          turning_radius / cross_product;

    line.push_back(new_line);
    if (curve.empty()) {
      geometry::Site line_start = poly[i - 1];
      LineSampling(line_start, start_i, line.back());
    } else {
      geometry::Site line_start = curve.back().back();
      LineSampling(line_start, start_i, line.back());
    }
    curve.push_back(new_curve);
    ArcSampling(cent_i, start_i, end_i, cross_product, curve.back());
  }
  geometry::Site final_point = poly[count - 1];
  std::vector<geometry::Site> end_line;
  line.push_back(end_line);
  LineSampling(curve.back().back(), final_point, line.back());
  // add to the final result
  for (std::size_t i = 0; i < curve.size(); i++) {
    for (std::size_t j = 0; j < line[i].size(); j++) {
      final_result_.push_back(line[i][j]);
    }
    for (std::size_t k = 0; k < curve[i].size(); k++) {
      final_result_.push_back(curve[i][k]);
    }
  }
  for (std::size_t i = 0; i < line.back().size(); i++) {
    final_result_.push_back(line.back()[i]);
  }
  return;
}

bool RingPlanner::PtInterpolation() {
  if (test_poly_.size() < 1) return false;
  if (final_path_.size() < 2) return false;
  // filter
  test_poly_filter_.push_back(test_poly_[0]);
  for (std::size_t i = 1; i < test_poly_.size(); i++) {
    double dis = (test_poly_[i] - test_poly_[i + 1]).mold();
    if (dis > 1e-2) test_poly_filter_.push_back(test_poly_[i]);
  }
  if (test_poly_filter_.size() < 1) return false;
  // find a nearest point and rearrange the test_poly_
  double nearest_dis = std::numeric_limits<double>::max();
  int n_ind = 0;
  for (std::size_t i = 0; i < test_poly_filter_.size(); i++) {
    double dis = std::hypot(test_poly_filter_[i].x - final_path_.back().x,
                            test_poly_filter_[i].y - final_path_.back().y);
    if (nearest_dis > dis) {
      nearest_dis = dis;
      n_ind = i;
    }
  }
  for (std::size_t i = 0; i < n_ind; i++) {
    test_poly_filter_.push_back(test_poly_filter_[i]);
  }
  for (std::size_t i = 0; i < n_ind; i++) {
    test_poly_filter_.erase(test_poly_filter_.begin());
  }
  // todo by jf
  SecondaryFilt();

  CurveSmoothing();
  auto pp_smoother = std::make_shared<optimizer::PurePursuit>();
  double last_angle =
      (final_path_.back() - final_path_[final_path_.size() - 2]).inerangle();
  pp_smoother->Init(
      geometry::Site(final_path_.back().x, final_path_.back().y, last_angle));
  pp_smoother->Smooth(test_poly_interpolation_);
  return true;
}

bool RingPlanner::CurveSmoothing() {
  // search the connor point
  geometry::SiteVec main_pts;
  main_pts.clear();
  if (test_poly_filter_.size() < 3) return false;
  main_pts.push_back(test_poly_filter_.front());
  // std::cout << "[test]" << test_poly_filter_.front().x << "," <<
  // test_poly_filter_.front().y << std::endl;
  for (std::size_t i = 1; i < test_poly_filter_.size() - 1; i++) {
    double bf_angle =
        (test_poly_filter_[i] - test_poly_filter_[i - 1]).inerangle();
    double af_angle =
        (test_poly_filter_[i + 1] - test_poly_filter_[i]).inerangle();
    if (std::fabs(bf_angle - af_angle) > 10) {
      main_pts.push_back(test_poly_filter_[i]);
    }
  }
  main_pts.push_back(test_poly_filter_.back());
  // curve smoothing
  if (main_pts.size() < 3) return false;
  edgelist n_edgeset;
  for (int i = 0; i < main_pts.size() - 1; i++) {
    int next = i + 1;
    geometry::Site temp = (main_pts[next] - main_pts[i]).direction();
    n_edgeset.push_back(temp);
    // std::cout << "[temp]" << temp.x << "," << temp.y << std::endl;
  }
  std::vector<std::pair<bool, std::pair<double, geometry::SiteVec>>> curves;
  for (int i = 1; i < main_pts.size() - 1; i++) {
    int start = i - 1;
    double cur_line_dis;
    double nxt_line_dis;
    if (curves.empty() || curves[i - 2].first == false) {
      cur_line_dis = (main_pts[i] - main_pts[i - 1]).mold();
      nxt_line_dis = (main_pts[i + 1] - main_pts[i]).mold();
    } else {
      cur_line_dis = (main_pts[i] - curves[i - 2].second.second[1]).mold();
      nxt_line_dis = (main_pts[i + 1] - main_pts[i]).mold();
    }
    double cross = n_edgeset[start].x * n_edgeset[i].y -
                   n_edgeset[start].y * n_edgeset[i].x;
    double n_dot = n_edgeset[start].x * n_edgeset[i].x +
                   n_edgeset[start].y * n_edgeset[i].y;
    double smooth_dis = std::fabs(turning_radius / cross * (1.0 - n_dot));
    // std::cout << "[breaking ind]" << smooth_dis << "," << cur_line_dis << ","
    // << nxt_line_dis << std::endl;
    if (smooth_dis >= cur_line_dis || smooth_dis >= nxt_line_dis) {
      geometry::SiteVec tmp;
      curves.push_back(std::make_pair(false, std::make_pair(1.0, tmp)));
      // std::cout << "[breaking ind]" << i << std::endl;
      continue;
    }
    geometry::Site start_i, end_i, cent_i;
    if (cross > 0.0) {
      start_i = main_pts[i] -
                n_edgeset[start] * turning_radius / cross * (1.0 - n_dot);
      end_i =
          main_pts[i] + n_edgeset[i] * turning_radius / cross * (1.0 - n_dot);
      cent_i = main_pts[i] +
               (n_edgeset[i] - n_edgeset[start]) * turning_radius / cross;
    } else {
      start_i = main_pts[i] +
                n_edgeset[start] * turning_radius / cross * (1.0 - n_dot);
      end_i =
          main_pts[i] - n_edgeset[i] * turning_radius / cross * (1.0 - n_dot);
      cent_i = main_pts[i] -
               (n_edgeset[i] - n_edgeset[start]) * turning_radius / cross;
    }

    test_circle.push_back(start_i);
    test_circle.push_back(end_i);
    test_circle.push_back(cent_i);
    geometry::SiteVec temp;
    temp.push_back(start_i);
    temp.push_back(end_i);
    temp.push_back(cent_i);
    curves.push_back(std::make_pair(true, std::make_pair(cross, temp)));
  }
  // interpolation and connection
  // std::cout << curves.size() << "[]" << main_pts.size() << std::endl;
  for (int i = 0; i < main_pts.size() - 2; i++) {
    // std::cout << "[curve]" << curves[i].first << "," <<
    // curves[i].second.first << std::endl;
    if (curves[i].first == false) {
      if (i == 0) {
        LineSampling(main_pts[i], main_pts[i + 1], test_poly_interpolation_);
      } else {
        if (curves[i - 1].first == false) {
          LineSampling(main_pts[i], main_pts[i + 1], test_poly_interpolation_);
        } else {
          LineSampling(curves[i - 1].second.second[1], main_pts[i + 1],
                       test_poly_interpolation_);
        }
      }
    } else {
      if (i == 0) {
        LineSampling(main_pts[i], curves[i].second.second[0],
                     test_poly_interpolation_);
      } else {
        if (curves[i - 1].first == false) {
          LineSampling(main_pts[i], curves[i].second.second[0],
                       test_poly_interpolation_);
        } else {
          LineSampling(curves[i - 1].second.second[1],
                       curves[i].second.second[0], test_poly_interpolation_);
        }
      }
      // std::cout << "[----------------]" << std::endl;
      ArcSampling(curves[i].second.second[2], curves[i].second.second[0],
                  curves[i].second.second[1], curves[i].second.first,
                  test_poly_interpolation_);
    }
  }
  return true;
}

bool RingPlanner::SecondaryFilt() {
  // todo by jf
  return true;
}

bool RingPlanner::LineSampling(const geometry::Site &start,
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
    sample_line += sample_dis;
  }
  return true;
}

bool RingPlanner::ArcSampling(const geometry::Site &cent,
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
  double single = sample_dis / turning_radius;
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
      double x = turning_radius * cos(cur_rad);
      double y = turning_radius * sin(cur_rad);
      temp.x = cent.x + x;
      temp.y = cent.y + y;
    } else {
      double cur_rad = start_rad - sample_rad;
      // std::cout << "cur_rad" << cur_rad << std::endl;
      cur_rad = cur_rad < -M_PI ? cur_rad + 2 * M_PI : cur_rad;
      double x = turning_radius * cos(cur_rad);
      double y = turning_radius * sin(cur_rad);
      temp.x = cent.x + x;
      temp.y = cent.y + y;
      // std::cout << "[circle]" << temp.x << "," << temp.y << std::endl;
    }
    sample.push_back(temp);
    sample_rad += single;
  }
  return true;
}

int RingPlanner::CheckInner(const polygonsite &poly, const polygonsite &base,
                            std::vector<bool> &inner) {
  int result = 0;
  for (int i = 0; i < poly.size(); i++) {
    int next_i = i == poly.size() - 1 ? 0 : i + 1;
    if (IsInSide(poly[i], base)) {
      inner[i] = true;
      // std::cout << "[inside]" << poly[i].x << "," << poly[i].y << std::endl;
    } else {
      for (int j = 0; j < base.size(); j++) {
        int next_j = j == base.size() - 1 ? 0 : j + 1;
        if (CheckCrossOfLines(poly[i], poly[next_i], base[j], base[next_j])) {
          inner[i] = true;
          // std::cout << "[inside]" << poly[i].x << "," << poly[i].y <<
          // std::endl;
          break;
        }
      }
    }
  }
  for (const auto &p : inner) {
    if (p == true) result++;
  }
  // std::cout << "result" << result << std::endl;
  return result;
}

int RingPlanner::CheckOuter(const polygonsite &poly, const polygonsite &base,
                            std::vector<bool> &outer) {
  int result = 0;
  if (poly.size() < 1 || base.size() < 1) {
    // std::cout << "---------" << std::endl;
  }
  for (int i = 0; i < poly.size(); i++) {
    int next_i = i == poly.size() - 1 ? 0 : i + 1;
    if (!IsInSide(poly[i], base) && !IsInSide(poly[next_i], base)) {
      for (int j = 0; j < base.size(); j++) {
        int next_j = j == base.size() - 1 ? 0 : j + 1;
        if (CheckCrossOfLines(poly[i], poly[next_i], base[j], base[next_j])) {
          outer[i] = true;
          break;
        }
      }
    } else {
      outer[i] = true;
    }
  }
  for (const auto &p : outer) {
    if (p == false) result++;
  }
  return result;
}

int RingPlanner::CheckCollision(const polygonsite &poly,
                                const polygonsite &base,
                                std::vector<bool> &collision) {
  int result = 0;
  for (int i = 0; i < poly.size(); i++) {
    int next_i = i == poly.size() - 1 ? 0 : i + 1;
    for (int j = 0; j < base.size(); j++) {
      int next_j = j == base.size() - 1 ? 0 : j + 1;
      if (CheckCrossOfLines(poly[i], poly[next_i], base[j], base[next_j])) {
        // std::cout << "collision:" << poly[i].x << "," << poly[i].y
        //           << "," << poly[next_i].x << "," << poly[next_i].y
        //           << "," << base[j].x << "," << base[j].y
        //           << "," << base[next_j].x << "," << base[next_j].y <<
        //           std::endl;
        collision[i] = true;
        // collision[next_i] = true;
        break;
      }
    }
  }
  for (const auto &p : collision) {
    if (p == true) result++;
  }
  return result;
}

bool RingPlanner::Eroding(const polygonsite &poly, polygonsite &new_poly,
                          std::vector<bool> &collision) {
  // 1.edge set and nornalize it
  edgelist edgeset, n_edgeset;
  int count = poly.size();
  for (int i = 0; i < count; i++) {
    int next = (i == count - 1) ? 0 : (i + 1);
    geometry::Site temp = poly[next] - poly[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  // 2.compute the eroding point
  polygonsite erodingpoly, tmperodingpoly;
  std::vector<std::pair<geometry::Site, bool>> eroding_poly;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : (i - 1);
    int end = i;
    double cross_product = n_edgeset[start].x * n_edgeset[end].y -
                           n_edgeset[start].y * n_edgeset[end].x;
    // todo
    geometry::Site eroding_pt;
    eroding_pt =
        (n_edgeset[end] - n_edgeset[start]) * carwidth / cross_product +
        poly[end];
    tmperodingpoly.push_back(eroding_pt);
  }
  // for (const auto &p : tmperodingpoly) {
  //   std::cout << "[tmp]" << p.x << "," << p.y << std::endl;
  // }
  // for (const auto &p : collision) {
  //   std::cout << "[]" << p << std::endl;
  // }
  polygonsite orig_erodingpoly;
  for (int i = 0; i < count; i++) {
    int next_i = i == count - 1 ? 0 : i + 1;
    int before_i = i == 0 ? count - 1 : i - 1;
    if (collision[i] == false) {
      if (collision[before_i] == false) {
        orig_erodingpoly.push_back(tmperodingpoly[i]);
      } else {
        orig_erodingpoly.push_back(GetCrossPt(tmperodingpoly[i],
                                              tmperodingpoly[next_i],
                                              poly[before_i], poly[i], true));
      }
      if (collision[next_i] == false) {
        orig_erodingpoly.push_back(tmperodingpoly[next_i]);
      } else {
        int next_ii = next_i == count - 1 ? 0 : next_i + 1;
        orig_erodingpoly.push_back(
            GetCrossPt(tmperodingpoly[i], tmperodingpoly[next_i], poly[next_i],
                       poly[next_ii], true));
      }
    } else {
      if (collision[before_i] == false) {
        orig_erodingpoly.push_back(GetCrossPt(poly[i], poly[next_i],
                                              tmperodingpoly[before_i],
                                              tmperodingpoly[i], true));
      } else {
        orig_erodingpoly.push_back(poly[i]);
      }
      if (collision[next_i] == false) {
        int next_ii = next_i == count - 1 ? 0 : next_i + 1;
        orig_erodingpoly.push_back(GetCrossPt(poly[i], poly[next_i],
                                              tmperodingpoly[next_i],
                                              tmperodingpoly[next_ii], true));
      } else {
        orig_erodingpoly.push_back(poly[next_i]);
      }
    }
  }
  for (std::size_t i = 0; i < orig_erodingpoly.size(); i += 2) {
    erodingpoly.push_back(orig_erodingpoly[i]);
  }
  if (erodingpoly.size() != tmperodingpoly.size()) erodingpoly.pop_back();
  for (const auto &p : erodingpoly) {
    // erodingpoly.push_back(p);
    eroding_poly.push_back(std::make_pair(p, false));
  }
  // 4. check if there is cross between the edges
  std::vector<int> cross_points;
  std::vector<geometry::Site> sub_polygon;
  polygonlist sub_polygons;
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
    polygonsite tmp_poly;
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
  for (auto &p : sub_polygons[select_index]) {
    // std::cout << "[erode_cross]" << p.x << "," << p.y << std::endl;
    new_poly.push_back(p);
  }
  return true;
}

bool RingPlanner::Eroding(const polygonsite &poly, polygonsite &new_poly) {
  if (poly.size() < 3) return false;
  // 1.edge set and nornalize it
  edgelist edgeset, n_edgeset;
  int count = poly.size();
  for (int i = 0; i < count; i++) {
    int next = (i == count - 1) ? 0 : (i + 1);
    geometry::Site temp = poly[next] - poly[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  // 2.compute the eroding point
  polygonsite erodingpoly;
  std::vector<std::pair<geometry::Site, bool>> eroding_poly;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : (i - 1);
    int end = i;
    double cross_product = n_edgeset[start].x * n_edgeset[end].y -
                           n_edgeset[start].y * n_edgeset[end].x;
    // todo
    geometry::Site eroding_pt;
    eroding_pt =
        (n_edgeset[end] - n_edgeset[start]) * carwidth / cross_product +
        poly[end];
    erodingpoly.push_back(eroding_pt);
    eroding_poly.push_back(std::make_pair(eroding_pt, false));
  }
  // 4. check if there is cross between the edges
  std::vector<int> cross_points;
  std::vector<geometry::Site> sub_polygon;
  polygonlist sub_polygons;
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
    polygonsite tmp_poly;
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
  for (auto &p : sub_polygons[select_index]) {
    // std::cout << "[erode]" << p.x << "," << p.y << std::endl;
    new_poly.push_back(p);
  }
  return true;
}

bool RingPlanner::Dilating(const polygonsite &poly, polygonsite &new_poly) {
  if (poly.size() < 3) return false;
  // 1.edge set and nornalize it
  edgelist edgeset, n_edgeset;
  int count = poly.size();
  for (int i = 0; i < count; i++) {
    int next = (i == count - 1) ? 0 : (i + 1);
    geometry::Site temp = poly[next] - poly[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  // 2.compute the dilating point
  polygonsite tmpdilatingpoly;
  std::vector<std::pair<geometry::Site, bool>> dilating_poly;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : i - 1;
    int end = i;
    double cross_product = n_edgeset[start].x * n_edgeset[end].y -
                           n_edgeset[start].y * n_edgeset[end].x;
    geometry::Site dilating_pt;
    dilating_pt = poly[end] - (n_edgeset[end] - n_edgeset[start]) * carwidth /
                                  cross_product;
    tmpdilatingpoly.push_back(dilating_pt);
    dilating_poly.push_back(std::make_pair(dilating_pt, false));
  }
  // if (debug_mode) {
  //   for (const auto &p : tmpdilatingpoly) {
  //     std::cout << "dilate:" << p.x << "," << p.y << std::endl;
  //   }
  // }
  // 4. check if there is cross between the edges
  std::vector<int> cross_points;
  std::vector<geometry::Site> sub_polygon;
  polygonlist sub_polygons;
  int correct_num = 0;
  while (correct_num < count) {
    int start_i = 0;
    int end_i = count;
    for (int i = 0; i < count; i++) {
      if (dilating_poly[i].second == true) {
        start_i++;
      } else {
        break;
      }
    }
    int tmp_i = start_i;
    bool cur_cross = false;
    polygonsite tmp_poly;
    tmp_poly.clear();
    sub_polygons.push_back(tmp_poly);
    while (end_i != start_i) {
      if (!cur_cross) {
        sub_polygons.back().push_back(tmpdilatingpoly[tmp_i]);
        dilating_poly[tmp_i].second = true;
        correct_num++;
      }
      int next_i = (tmp_i == count - 1) ? 0 : tmp_i + 1;
      int before_i = (tmp_i == 0) ? count - 1 : tmp_i - 1;
      cur_cross = false;
      for (int j = tmp_i + 1; j < count; j++) {
        int next_j = (j == count - 1) ? 0 : (j + 1);
        if (j == before_i || j == next_i) continue;
        if (CheckCrossOfLines(tmpdilatingpoly[tmp_i], tmpdilatingpoly[next_i],
                              tmpdilatingpoly[j], tmpdilatingpoly[next_j])) {
          cur_cross = true;
          geometry::Site cross_pt =
              GetCrossPt(tmpdilatingpoly[tmp_i], tmpdilatingpoly[next_i],
                         tmpdilatingpoly[j], tmpdilatingpoly[next_j]);
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
  for (auto &p : sub_polygons[select_index]) new_poly.push_back(p);
  return true;
}

bool RingPlanner::Dilating(const polygonsite &poly, polygonsite &new_poly,
                           std::vector<bool> &collision) {
  if (poly.size() < 3) return false;
  // 1.edge set and nornalize it
  edgelist edgeset, n_edgeset;
  int count = poly.size();
  for (int i = 0; i < count; i++) {
    int next = (i == count - 1) ? 0 : (i + 1);
    geometry::Site temp = poly[next] - poly[i];
    edgeset.push_back(temp);
    n_edgeset.push_back(temp.direction());
  }
  // 2.compute the dilating point
  polygonsite tmpdilatingpoly;
  std::vector<std::pair<geometry::Site, bool>> dilating_poly;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : i - 1;
    int end = i;
    double cross_product = n_edgeset[start].x * n_edgeset[end].y -
                           n_edgeset[start].y * n_edgeset[end].x;
    geometry::Site dilating_pt;
    dilating_pt = poly[end] - (n_edgeset[end] - n_edgeset[start]) * carwidth /
                                  cross_product;
    tmpdilatingpoly.push_back(dilating_pt);
    dilating_poly.push_back(std::make_pair(dilating_pt, false));
  }
  polygonsite orig_dilatingpoly;
  for (int i = 0; i < count; i++) {
    int next_i = i == count - 1 ? 0 : i + 1;
    int before_i = i == 0 ? count - 1 : i - 1;
    if (collision[i] == true) {
      if (collision[before_i] == true) {
        orig_dilatingpoly.push_back(tmpdilatingpoly[i]);
      } else {
        orig_dilatingpoly.push_back(GetCrossPt(tmpdilatingpoly[i],
                                               tmpdilatingpoly[next_i],
                                               poly[before_i], poly[i], true));
      }
      if (collision[next_i] == true) {
        orig_dilatingpoly.push_back(tmpdilatingpoly[next_i]);
      } else {
        int next_ii = next_i == count - 1 ? 0 : next_i + 1;
        orig_dilatingpoly.push_back(
            GetCrossPt(tmpdilatingpoly[i], tmpdilatingpoly[next_i],
                       poly[next_i], poly[next_ii], true));
      }
    } else {
      if (collision[before_i] == true) {
        orig_dilatingpoly.push_back(GetCrossPt(poly[i], poly[next_i],
                                               tmpdilatingpoly[before_i],
                                               tmpdilatingpoly[i], true));
      } else {
        orig_dilatingpoly.push_back(poly[i]);
      }
      if (collision[next_i] == true) {
        int next_ii = next_i == count - 1 ? 0 : next_i + 1;
        orig_dilatingpoly.push_back(GetCrossPt(poly[i], poly[next_i],
                                               tmpdilatingpoly[next_i],
                                               tmpdilatingpoly[next_ii], true));
      } else {
        orig_dilatingpoly.push_back(poly[next_i]);
      }
    }
  }
  polygonsite dilatingpoly;
  for (std::size_t i = 0; i < orig_dilatingpoly.size(); i += 2) {
    dilatingpoly.push_back(orig_dilatingpoly[i]);
  }
  // 4. check if there is cross between the edges
  std::vector<int> cross_points;
  std::vector<geometry::Site> sub_polygon;
  polygonlist sub_polygons;
  int correct_num = 0;
  while (correct_num < count) {
    int start_i = 0;
    int end_i = count;
    for (int i = 0; i < count; i++) {
      if (dilating_poly[i].second == true) {
        start_i++;
      } else {
        break;
      }
    }
    int tmp_i = start_i;
    bool cur_cross = false;
    polygonsite tmp_poly;
    tmp_poly.clear();
    sub_polygons.push_back(tmp_poly);
    while (end_i != start_i) {
      if (!cur_cross) {
        sub_polygons.back().push_back(dilatingpoly[tmp_i]);
        dilating_poly[tmp_i].second = true;
        correct_num++;
      }
      int next_i = (tmp_i == count - 1) ? 0 : tmp_i + 1;
      int before_i = (tmp_i == 0) ? count - 1 : tmp_i - 1;
      cur_cross = false;
      for (int j = tmp_i + 1; j < count; j++) {
        int next_j = (j == count - 1) ? 0 : (j + 1);
        if (j == before_i || j == next_i) continue;
        if (CheckCrossOfLines(dilatingpoly[tmp_i], dilatingpoly[next_i],
                              dilatingpoly[j], dilatingpoly[next_j])) {
          cur_cross = true;
          geometry::Site cross_pt =
              GetCrossPt(dilatingpoly[tmp_i], dilatingpoly[next_i],
                         dilatingpoly[j], dilatingpoly[next_j]);
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
  for (auto &p : sub_polygons[select_index]) new_poly.push_back(p);
  return true;
}

bool RingPlanner::CheckLeftOfEdge(const geometry::Site &p,
                                  const geometry::Site &p1,
                                  const geometry::Site &p2) {
  double tmpx = (p1.x - p2.x) / (p1.y - p2.y) * (p.y - p2.y) + p2.x;
  if (tmpx < p.x) return true;
  return false;
}

bool RingPlanner::CheckCrossOfLines(const geometry::Site &p1,
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

double RingPlanner::Mult(const geometry::Site &p0, const geometry::Site &p1,
                         const geometry::Site &p2) {
  return (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
}

double RingPlanner::Mult(const geometry::Site &p1, const geometry::Site &p2) {
  return p1.x * p2.y - p1.y * p2.x;
}

geometry::Site RingPlanner::GetCrossPt(const geometry::Site &p1,
                                       const geometry::Site &p2,
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

geometry::Site RingPlanner::GetCrossPt(const geometry::Site &p1,
                                       const geometry::Site &p2,
                                       const geometry::Site &p3,
                                       const geometry::Site &p4,
                                       bool straight) {
  double a1 = p2.y - p1.y;
  double b1 = p1.x - p2.x;
  double c1 = p1.x * p2.y - p1.y * p2.x;
  double a2 = p4.y - p3.y;
  double b2 = p3.x - p4.x;
  double c2 = p3.x * p4.y - p3.y * p4.x;
  double d = a1 * b2 - a2 * b1;
  geometry::Site pt;
  pt.y = (a1 * c2 - a2 * c1) / d;
  pt.x = (c1 * b2 - c2 * b1) / d;
  return pt;
}

double RingPlanner::GetDisFromPtToLine(const geometry::Site &pt,
                                       const geometry::Site &p1,
                                       const geometry::Site &p2) {
  double a = (pt.x - p1.x) * (p2.y - p1.y) + (pt.y - p1.y) * (p1.x - p2.x);
  double b = (p2.y - p1.y) * (p2.y - p1.y) + (p1.x - p2.x) * (p1.x - p2.x);
  double result = sqrt(pow(a, 2) / b);
  return result;
}

double RingPlanner::GetNearestDis(const geometry::Site &pt,
                                  const polygonsite &poly) {
  double min_dis = std::numeric_limits<double>::max();
  for (int i = 0; i < poly.size(); i++) {
    int next_i = i == poly.size() - 1 ? 0 : i + 1;
    double cur_dis = GetDisFromPtToLine(pt, poly[i], poly[next_i]);
    if (cur_dis < min_dis) {
      min_dis = cur_dis;
    }
  }
  return min_dis;
}

}  // namespace coverage
