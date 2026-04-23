#include "pathplanner/blanket/hybridtrack.h"
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <queue>

#include "coverage/clipper/clipper.hpp"
#include "math/cgl.h"
#include "pathplanner/blanket/conststruct.h"

namespace coverage {

HybridTrack::HybridTrack() {
  status_ = false;
  offset_dis_ = 1.0;
  start_ind_ = 0;
}

bool HybridTrack::Interface(const std::vector<double>& x,
                            const std::vector<double>& y, const double x_orig,
                            const double y_orig) {
  // ����3����
  if (x.size() < 3 || y.size() < 3 || x.size() != y.size()) return false;

  // ���ɱ�Ե
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
  start_pt_.x = x_orig - x_offset_;
  start_pt_.y = y_orig - y_offset_;

  for (int i = 0; i < x.size(); i++) {
    geometry::Site temp;
    temp.x = x[i] - x_offset_;
    temp.y = y[i] - y_offset_;
    if ((start_pt_ - temp).mold() < 1e-3) start_ind_ = i;
    orig_edge_.push_back(temp);
  }

  // ����ģ�ת�������
  if (!math::cgl::IfAutoclockwise(orig_edge_)) {
    std::reverse(orig_edge_.begin(), orig_edge_.end());
    start_ind_ = orig_edge_.size() - 1 - start_ind_;
  }

  status_ = true;
  return true;
}

int FindNearstPoint(const std::vector<Site>& mousePoint, const Site& point) {
  int cur_idx = -1;

  double minDist = 1000000;
  for (int i = 0; i < mousePoint.size(); i++) {
    const Site& pnt = mousePoint[i];

    double detX = point.x - pnt.x;
    double detY = point.y - pnt.y;
    double dist = sqrt(detX * detX + detY * detY);

    if (dist < minDist) {
      cur_idx = i;
      minDist = dist;
    }
  }

  return cur_idx;
}

bool HybridTrack::Planning(int key) {
  if (!status_) return false;

  // ����������·��
  // TODO
  // 1.
  // ��͹�Ŀ��ܻ��Խ�����͹������֮���û��
  // 2. �ж��Ļ�δ����
  // 3. ��������
  SiteVec current_poly = orig_edge_;  // ��ǰȦ
  SiteVec path;
  std::vector<PolyNode> poly_node;

  // ��һ��������ȫ���룬�ڶ�������һ����ɨ���
  double first_offset = 0.8, normal_offset = 0.5 * 2;  // ����ͼԼ���ĵ�
  int iteration = 0;
  double offset = 0;
  while (1) {
    if (iteration == 0)
      offset = first_offset;
    else
      offset = normal_offset;

    SiteVec next_poly;
    poly_node.push_back(PolyNode(current_poly));
    bool shrink = WeltPathGeneration(poly_node, next_poly, offset);
    if (shrink == false) break;

    if (iteration == 0) {
      //�������򶥵�
      int idx = FindNearstPoint(next_poly, orig_edge_[key]);
      for (int i = 0; i < idx; i++) {
        next_poly.push_back(next_poly[i]);
      }
      next_poly.erase(next_poly.begin(), next_poly.begin() + idx);
    }

    current_poly = next_poly;
    poly_node.clear();
    path.insert(path.end(), next_poly.begin(), next_poly.end());

    iteration++;
  }

  if (!PathConnection(path)) return false;

  for (std::size_t i = 0; i < final_path_.size(); i++) {
    final_path_[i].x += x_offset_;
    final_path_[i].y += y_offset_;
  }

  return true;

  // std::queue<PolyNode> nodequeue;
  // std::vector<PolyNode> output_node_list;
  // output_node_list.push_back(PolyNode(orig_edge_));
  // PolyNode orig_node(orig_edge_);
  // nodequeue.push(orig_node);
  // int count = 0;

  /*
  while (!nodequeue.empty()) {
          count++;
          // std::cout << "count:" << count << std::endl;
          PolyNode cur_node = nodequeue.front();
          std::vector<PolyNode> inter_polys;
          nodequeue.pop();

          // ��cur_node������С���inter polys
          if (CheckRingInPoly(cur_node, inter_polys)) {
                  for (std::size_t i = 0; i < inter_polys.size(); i++) {
                          if (inter_polys[i].if_thrink) {
                                  nodequeue.push(inter_polys[i]);
                          }
                          else {
                                  output_node_list.push_back(inter_polys[i]);
                          }
                  }
          } else if (nodequeue.empty()) {
                  output_node_list.push_back(cur_node);
          }
  }
  */
  // thrink and expand the polygons using the if_thrink flag
  // std::cout << "start shrink" << std::endl;
  // SiteVec output_list;
  // if (!WeltPathGeneration(output_node_list, output_list)) return false;
  // if (!PathConnection(output_list)) return false;
}

bool HybridTrack::CheckRingInPoly(const PolyNode& poly,
                                  std::vector<PolyNode>& subpoly) {
  // todo(by jf)
  subpoly.clear();
  if (!poly.if_thrink) return false;
  std::vector<std::pair<int, int>> pt_pair = FindPtPair(poly);
  // std::cout << __LINE__ << "," << pt_pair.size() << std::endl;
  if (pt_pair.size() < 2) return false;
  if (!FindSubpoly(poly, pt_pair, subpoly)) return false;
  return true;
}

// true:represent the poly1 and the poly2 have some relationship
// false:represent the poly and the poly have no relationship
bool HybridTrack::RelationBetweenPolys(PolyNode& poly1, PolyNode& poly2) {
  bool reverse_flag_1 = false;
  bool reverse_flag_2 = false;
  if (!math::cgl::IfAutoclockwise(poly1.poly)) {
    std::reverse(poly1.poly.begin(), poly1.poly.end());
    reverse_flag_1 = true;
  }
  if (!math::cgl::IfAutoclockwise(poly2.poly)) {
    std::reverse(poly2.poly.begin(), poly2.poly.end());
    reverse_flag_2 = true;
  }
  if (math::cgl::IsInside(poly1.poly[0], poly2.poly)) {
    // std::cout << __LINE__ << std::endl;
    for (const auto& p : poly1.poly) {
      if (!math::cgl::IsInside(p, poly2.poly)) return false;
    }
    for (const auto& p : poly2.poly) {
      if (math::cgl::IsInside(p, poly1.poly)) return false;
    }
    // poly1 is inside poly2
    poly1.ind = reverse_flag_1 ? poly1.poly.size() - 1 : 0;
    poly2.ind = reverse_flag_2 ? poly2.poly.size() - 1 : 0;
    poly1.if_thrink = false;
    poly1.outer_pt = poly2.poly[poly2.ind];
    poly2.if_thrink = true;
    return true;
  } else {
    // std::cout << __LINE__ << std::endl;
    for (const auto& p : poly2.poly) {
      if (!math::cgl::IsInside(p, poly1.poly)) return false;
    }
    for (const auto& p : poly1.poly) {
      if (math::cgl::IsInside(p, poly2.poly)) return false;
    }
    // std::cout << __LINE__ << std::endl;
    // poly2 is inside poly1
    poly2.ind = reverse_flag_2 ? poly2.poly.size() - 1 : 0;
    poly1.ind = reverse_flag_1 ? poly1.poly.size() - 1 : 0;
    poly2.if_thrink = false;
    poly2.outer_pt = poly1.poly[poly1.ind];
    poly1.if_thrink = true;
    return true;
  }
  return false;
}

std::vector<std::pair<int, int>> HybridTrack::FindPtPair(const PolyNode& poly) {
  std::vector<std::pair<int, int>> pt_pair;
  pt_pair.clear();
  std::vector<std::pair<int, geometry::Site>> tmplist;
  for (std::size_t i = 0; i < poly.poly.size(); i++) {
    tmplist.push_back(std::make_pair(i, poly.poly[i]));
    if (!tmplist.empty()) {
      int tmp_size = tmplist.size();
      for (int j = 0; j < tmp_size - 1; j++) {
        if (std::fabs(tmplist[j].second.x - poly.poly[i].x) < 1e-1 &&
            std::fabs(tmplist[j].second.y - poly.poly[i].y) < 1e-1) {
          pt_pair.push_back(std::make_pair(tmplist[j].first, i));
          tmplist.erase(tmplist.begin() + j);
          tmplist.pop_back();
          break;
        }
      }
    }
  }
  // std::cout << "poly_map_size: " << tmplist.size() << std::endl;
  return pt_pair;
}

bool HybridTrack::FindSubpoly(const PolyNode& poly,
                              const std::vector<std::pair<int, int>>& pt_pair,
                              std::vector<PolyNode>& subpoly) {
  int pair_size = pt_pair.size();
  for (int i = 0; i < pair_size; i++) {
    for (int j = i + 1; j < pair_size; j++) {
      if (ConnectionValid(poly, pt_pair[i], pt_pair[j], subpoly)) {
        return true;
      }
    }
  }
  return false;
}

bool HybridTrack::ConnectionValid(const PolyNode& poly,
                                  const std::pair<int, int>& pair1,
                                  const std::pair<int, int>& pair2,
                                  std::vector<PolyNode>& subpoly) {
  int total = poly.poly.size();
  if (pair1.first == pair2.first || pair1.first == pair2.second ||
      pair1.second == pair2.first || pair1.second == pair2.second)
    return false;

  // set the order  for two pairs
  // p1's order: first < second
  // p2's order: first < second
  // pairs' order: p1 < p2
  bool ifchange = false;
  int p1first, p1second, p2first, p2second;
  if (pair1.first < pair1.second) {
    p1first = pair1.first;
    p1second = pair1.second;
  } else {
    p1first = pair1.second;
    p1second = pair1.first;
  }
  if (pair2.first < pair2.second) {
    p2first = pair2.first;
    p2second = pair2.second;
  } else {
    p2first = pair2.second;
    p2second = pair2.first;
  }
  if (p1first > p2first) {
    int temp;
    temp = p1first;
    p1first = p2first;
    p2first = temp;
    temp = p1second;
    p1second = p2second;
    p2second = temp;
    ifchange = true;
  }
  // std::cout << "p:" << p1first << "," << p1second << ","
  //           << p2first << "," << p2second << "," << ifchange << std::endl;
  if (p1second < p2first) {
    if (p1first == 0 && p2second == total - 1 && p2first - p1second == 1) {
      PolyNode temp1, temp2;
      subpoly.push_back(temp1);
      subpoly.push_back(temp2);
      for (int i = p1first; i < p1second; i++) {
        subpoly[0].poly.push_back(poly.poly[i]);
      }
      for (int i = p2first; i < p2second; i++) {
        subpoly[1].poly.push_back(poly.poly[i]);
      }
      if (!RelationBetweenPolys(subpoly[0], subpoly[1])) return false;
      return true;
    }
  } else {
    if (p1second > p2second && p2first - p1first == 1 &&
        p1second - p2second == 1) {
      PolyNode temp1, temp2;
      subpoly.push_back(temp1);
      subpoly.push_back(temp2);
      for (int i = p1second; i < total; i++) {
        subpoly[0].poly.push_back(poly.poly[i]);
      }
      for (int i = 0; i < p1first; i++) {
        subpoly[0].poly.push_back(poly.poly[i]);
      }
      for (int i = p2first; i < p2second; i++) {
        subpoly[1].poly.push_back(poly.poly[i]);
      }
      // std::cout << "subpoly_size:" << subpoly.size()
      //           << "," << subpoly[0].poly.size() << ","
      //           << subpoly[1].poly.size() << std::endl;
      // for (const auto &p : subpoly[0].poly) {
      //   std::cout << "subpoly[0]:" << p.x << "," << p.y << std::endl;
      // }
      // for (const auto &p : subpoly[1].poly) {
      //   std::cout << "subpoly[1]:" << p.x << "," << p.y << std::endl;
      // }
      if (!RelationBetweenPolys(subpoly[0], subpoly[1])) return false;
      return true;
    }
  }
  return false;
}

bool HybridTrack::PolyOffsetClipper(double offsetdis, const SiteVec& orig,
                                    SiteVec& poly, bool direction,
                                    std::vector<int>& match) {
  ClipperLib::ClipperOffset co(2.0);
  ClipperLib::Path path;
  ClipperLib::Paths solution;

  const double scale = 100;
  const double inv_scale = 1.0 / scale;

  for (auto& pt : orig) {
    path.push_back(ClipperLib::IntPoint(pt.x * scale, pt.y * scale));
  }
  co.AddPath(path, ClipperLib::JoinType::jtMiter,
             ClipperLib::EndType::etClosedPolygon);
  co.Execute(solution, -offsetdis * scale);

  if (solution.empty()) return false;

  // get the largest results
  std::vector<double> area;
  for (auto& s : solution) {
    area.push_back(ClipperLib::Area(s));
  }

  int best_sol = std::max_element(area.begin(), area.end()) - area.begin();

  if (area[best_sol] < 10.0 * scale * scale) {
    return false;
  }

  // fill poly and remove too close points
  Site last_pt;
  double closest_pt_th = 1.2;

  ClipperLib::Path mc_path = solution[best_sol];
  for (auto& p : mc_path) {
    Site cur_pt = Site(p.X * inv_scale, p.Y * inv_scale);
    if (poly.empty()) {
      poly.push_back(cur_pt);
      last_pt = cur_pt;
    } else if ((last_pt - cur_pt).mold() > closest_pt_th) {
      poly.push_back(cur_pt);
      last_pt = cur_pt;
    }
  }

  return true;
}

bool HybridTrack::PolyOffset(double offsetdis, const SiteVec& orig,
                             SiteVec& poly, bool direction,
                             std::vector<int>& match) {
  match.clear();
  if (orig.size() < 3) return false;

  // area of the origin
  std::vector<cv::Point2f> pt_orig_cv;
  for (auto& p : orig) {
    pt_orig_cv.push_back(cv::Point2f(p.x, p.y));
  }
  double area_orig = cv::contourArea(pt_orig_cv);

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
  SiteVec erodingpoly;  // �������С�����εĶ���

  // ��С֮��͹�Կ��ܷ����仯��������ߵķ���һ���ǶԵ�
  std::vector<std::pair<Site, bool>> poly_flag;
  for (int i = 0; i < count; i++) {
    int start = (i == 0) ? (count - 1) : (i - 1);
    int end = i;
    double cross_product =
        math::cgl::CrossProduct(n_edgeset[start], n_edgeset[end]);
    double cos_theta = math::cgl::DotProduct(n_edgeset[start], n_edgeset[end]);
    Site eroding_pt;
    if (direction) {
      eroding_pt =
          (n_edgeset[end] - n_edgeset[start]) * offsetdis / cross_product +
          orig[end];
    } else {
      eroding_pt = orig[end] - (n_edgeset[end] - n_edgeset[start]) * offsetdis /
                                   cross_product;
    }

    if (math::cgl::IsInside(eroding_pt, orig) == false) {
      // ���ܱ��Ƴ�����ı���
      // ����
      // eroding_pt = orig[end];
    }

    erodingpoly.push_back(eroding_pt);
    poly_flag.push_back(std::make_pair(eroding_pt, false));
  }

  // �����Խ�
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
        // ��Ӧ���ཻ�ĵط��ཻ�ˣ�զ��
        cross_ind[i].push_back(j);
        cross_ind[j].push_back(i);
      }
    }
  }

  // �г��Ӷ����
  SiteVec sub_polygon;
  std::vector<SiteVec> sub_polygons;
  std::vector<int> sub_ind;
  std::vector<std::vector<int>> sub_inds;
  int check_num = 0;
  int start_loop = 0;
  int end_loop = count;
  while (check_num < count) {
    start_loop = 0;
    for (int i = 0; i < count; i++) {
      if (poly_flag[i].second == true) {
        start_loop++;
      } else {
        break;
      }
    }
    // std::cout << "start_loop:" << start_loop << std::endl;
    // std::cout << erodingpoly[start_loop].x+x_offset_ << ","
    //           << erodingpoly[start_loop].y+y_offset_ << std::endl;
    int tmp = start_loop;
    SiteVec tmp_poly;
    tmp_poly.clear();
    sub_polygons.push_back(tmp_poly);
    std::vector<int> tmp_ind;
    sub_inds.push_back(tmp_ind);
    int loop_count = 0;
    while (1) {
      loop_count++;
      if (loop_count > count) {
        std::cout << "there must be some mistakes" << std::endl;
        break;
      }
      if (poly_flag[tmp].second == false) {
        sub_polygons.back().push_back(erodingpoly[tmp]);
        sub_inds.back().push_back(tmp);
        poly_flag[tmp].second = true;
        check_num++;
      }
      if (cross_ind[tmp].size() == 1) {
        // ����㲻�Խ�
        end_loop = cross_ind[tmp][0];
      } else {
        // ������Խ���
        end_loop = cross_ind[tmp][1];  // ������Խ����ù�����??

        int next_tmp = tmp == count - 1 ? 0 : tmp + 1;
        int next_end = end_loop == count - 1 ? 0 : end_loop + 1;
        Site cross_pt;
        math::cgl::GetCrossPtOfLines(erodingpoly[tmp], erodingpoly[next_tmp],
                                     erodingpoly[end_loop],
                                     erodingpoly[next_end], cross_pt);
        sub_polygons.back().push_back(cross_pt);
        sub_inds.back().push_back(1000);
        // sub_inds.back().push_back(tmp);
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
  // Ӧ��ѡ������
  int max_area = std::numeric_limits<int>::min();
  int select_index = 0;
  int correct_num = 0;
  for (std::size_t i = 0; i < sub_polygons.size(); i++) {
    if (math::cgl::IfAutoclockwise(sub_polygons[i])) {
      correct_num++;
      int cur_area = math::cgl::AreaOfPoly(sub_polygons[i]);
      if (max_area < cur_area) {
        select_index = i;
        max_area = sub_polygons[i].size();
      }
    }
  }

  if (sub_polygons.empty()) return false;
  poly.clear();
  poly.insert(poly.end(), sub_polygons[select_index].begin(),
              sub_polygons[select_index].end());
  match.insert(match.end(), sub_inds[select_index].begin(),
               sub_inds[select_index].end());

  /// ���ּ��
  // ���match���Ƿ���ŵ㱻�޳�
  bool is_zero_moved = true;
  int new_pt_index = 0;
  for (size_t i = 0; i < match.size(); ++i) {
    int m = match[i];
    if (m == 0) is_zero_moved = false;
    if (m == 1000) new_pt_index = i;
  }

  if (is_zero_moved) {
    // ��new_pt_index��ת����0����
    SiteVec new_poly;
    for (int i = new_pt_index, count = 0; count < match.size(); ++i, ++count) {
      i %= match.size();
      new_poly.push_back(poly[i]);
    }
    poly = new_poly;
  }

  // �ϲ���������ĵ�
  const double min_dis = 0.5;
  SiteVec new_poly;
  Site last_pt = poly[0];
  new_poly.push_back(last_pt);
  for (int i = 1; i < poly.size(); ++i) {
    Site cur_pt = poly[i];
    double dis = (last_pt - cur_pt).mold();
    if (dis < min_dis) {
      continue;
    } else {
      last_pt = cur_pt;
      new_poly.push_back(cur_pt);
    }
  }
  poly = new_poly;

  // ����������ϴεĶ���ξ����������ɾ�� �����ܵ����Խ���
  // new_poly.clear();
  // const double min_dis_to_last_poly = 0.5;
  // for (int i = 0; i < poly.size(); ++i) {
  // 	Site p = poly[i];
  // 	double dis = math::cgl::DisFromPt2Poly(p, orig);
  // 	if (math::cgl::IsInside(p, orig) == false || dis < min_dis_to_last_poly)
  // { 		continue; 	} else { 		new_poly.push_back(p);
  // 	}
  // }
  // poly = new_poly;

  if (poly.size() < 3) return false;

  // area of the optimized
  std::vector<cv::Point2f> pt_opt_cv;
  for (auto& p : poly) {
    pt_opt_cv.push_back(cv::Point2f(p.x, p.y));
  }
  double area_opt = cv::contourArea(pt_opt_cv);

  // �����ɵ����С����һ�εģ��������С�ڸ���ֵ������
  const double min_area = 2.0;
  if (area_opt < min_area || area_opt >= area_orig) {
    return false;
  }

  return true;
}

bool HybridTrack::WeltPathGeneration(std::vector<PolyNode>& polys,
                                     SiteVec& output_poly, double offset) {
  if (polys.size() < 1) return false;
  if (!polys.back().if_thrink) return false;

  // thrink the extreme outer poly
  // std::cout << __LINE__ << std::endl;
  std::vector<int> match;
  if (!PolyOffsetClipper(offset, polys.back().poly, polys.back().os_poly,
                         polys.back().if_thrink, match)) {
    std::cout << "the outer poly thrink error" << std::endl;
    return false;
  }

  if (polys.size() == 1) {
    output_poly.insert(output_poly.end(), polys.back().os_poly.begin(),
                       polys.back().os_poly.end());
    return true;
  }

  // expand the other polys
  std::map<int, std::pair<int, int>, KeyCompare> matching_map;
  int inner_size = polys.size() - 1;
  for (int i = 0; i < inner_size; i++) {
    if (polys[i].if_thrink) continue;
    if (!PolyOffsetClipper(offset, polys[i].poly, polys[i].os_poly,
                           polys[i].if_thrink, match)) {
      std::cout << "the inner poly expand error" << i << std::endl;
      return false;
    }
    // for (const auto &p : polys[i].os_poly) {
    //   std::cout << "inner:" << p.x << "," << p.y << std::endl;
    // }
    int inner_ind =
        GetNearestIndex(polys[i].poly[polys[i].ind], polys[i].os_poly);
    int outer_ind = GetNearestIndex(polys[i].outer_pt, polys.back().os_poly);
    // std::cout << "inner_ind" << inner_ind << "," << outer_ind << std::endl;
    matching_map.insert(
        std::make_pair(outer_ind, std::make_pair(inner_ind, i)));
  }
  std::vector<int> outer_intersects1;
  std::vector<int> outer_intersects2;
  std::vector<int> inner_intersects;
  std::vector<int> inner_polys;
  auto ite = matching_map.begin();
  for (; ite != matching_map.end(); ite++) {
    outer_intersects1.push_back(ite->first);
    inner_intersects.push_back(ite->second.first);
    inner_polys.push_back(ite->second.second);
  }
  outer_intersects2.push_back(0);
  for (int i = 1; i < outer_intersects1.size(); i++) {
    outer_intersects2.push_back(outer_intersects1[i]);
  }
  for (int i = 0; i < inner_intersects.size(); i++) {
    for (int j = outer_intersects2[i]; j <= outer_intersects1[i]; j++) {
      output_poly.push_back(polys.back().os_poly[j]);
    }
    for (int j = inner_intersects[i]; j < polys[inner_polys[i]].os_poly.size();
         j++) {
      output_poly.push_back(polys[inner_polys[i]].os_poly[j]);
    }
    for (int j = 0; j <= inner_intersects[i]; j++) {
      output_poly.push_back(polys[inner_polys[i]].os_poly[j]);
    }
  }
  for (int i = outer_intersects1.back(); i < polys.back().os_poly.size(); i++) {
    output_poly.push_back(polys.back().os_poly[i]);
  }
  output_poly.push_back(polys.back().os_poly[0]);
  if (output_poly.size() < 2) return false;
  return true;
}

int HybridTrack::GetNearestIndex(const Site& p, const SiteVec& poly) {
  double min_dis = std::numeric_limits<double>::max();
  int near_ind = 0;
  for (std::size_t i = 0; i < poly.size(); i++) {
    double dis = std::hypot(p.x - poly[i].x, p.y - poly[i].y);
    if (dis < min_dis) {
      min_dis = dis;
      near_ind = i;
    }
  }
  return near_ind;
}

bool HybridTrack::PathConnection(const SiteVec& poly) {
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
    b0_pos_ = 0.15 + (1 - std::fabs(cos_theta0)) * 0.05;
    b0_para_ = 0.05 + (1 - std::fabs(cos_theta0)) * 0.02;
    b1_pos_ = 0.85 - (1 - std::fabs(cos_theta1)) * 0.05;
    b1_para_ = 0.05 + (1 - std::fabs(cos_theta1)) * 0.02;

    SiteVec b_samples;
    BSampling(poly[i - 1], poly[i], r_unit, last_unit, b_samples);
    final_path_.insert(final_path_.end(), b_samples.begin(), b_samples.end());
  }

  SiteVec new_path1;
  LineSampling(final_path_.back(), poly.back(), new_path1);
  final_path_.insert(final_path_.end(), new_path1.begin(), new_path1.end());

  return true;
}

bool HybridTrack::BSampling(const Site& start, const Site& end,
                            const Site& next_direction,
                            const Site& last_direction,
                            std::vector<Site>& sample) {
  Site dir = end - start;
  double dis = dir.mold();
  Site n_sample = dir.direction();

  // ���
  bool short_lane = false;
  if (dis < 5.0) short_lane = true;
  double d1 = b0_pos_ * dis;
  double d2 = b1_pos_ * dis;
  if (d1 > 3.0) d1 = 3.0;
  if (d2 < (dis - 3.0)) d2 = dis - 3.0;

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

  geometry::Site pt1 = start + n_sample * d1 - last_direction * 0.5 * b0_para_;
  geometry::Site pt2 = start + n_sample * 0.5 * dis;
  geometry::Site pt3 = start + n_sample * d2 + next_direction * 0.5 * b1_para_;

  // ����ֻȡ�м��
  if (short_lane) {
    sample = SiteVec{pt2};
  } else {
    sample = SiteVec{pt1, pt2, pt3};
  }
  return true;
}

// sparse line sampling
bool HybridTrack::LineSampling(const geometry::Site& start,
                               const geometry::Site& end,
                               std::vector<geometry::Site>& sample) {
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

// sparse arc sampling
bool HybridTrack::ArcSampling(const geometry::Site& cent,
                              const geometry::Site& start,
                              const geometry::Site& end, const double cross,
                              std::vector<geometry::Site>& sample) {
  sample.clear();
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
  // double single = kSampleDis / kMinRadius;
  double single = 0.3 / kMinRadius;
  double sample_rad = 0.0;
  double dis_mold = (cent - start).mold();
  double dis_mold1 = (cent - end).mold();
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

  if (sample.empty() == false) {
    Site middle = sample[0.5 * sample.size()];
    sample.clear();
    sample.push_back(middle);
  }
  return true;
}

}  // namespace coverage
