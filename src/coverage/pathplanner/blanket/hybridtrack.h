#ifndef INCLUDE_PATHPLANNER_BLANKET_HYBRIDTRACK_H_
#define INCLUDE_PATHPLANNER_BLANKET_HYBRIDTRACK_H_

#include <iostream>
#include <unordered_map>
#include <vector>
#include "coverage/geometry/geoheader.h"

namespace coverage {

using geometry::Site;
using geometry::SiteVec;

struct KeyCompare {
  bool operator()(const int &lhs, const int &rhs) const { return lhs < rhs; }
};

struct KeyHash {
  std::size_t operator()(const geometry::Site &k) const {
    return std::hash<double>()(k.x) ^ std::hash<double>()(k.y) << 1;
  }
};

struct KeyEqual {
  bool operator()(const geometry::Site &l, const geometry::Site &r) const {
    return std::fabs(l.x - r.x) < 1e-1 && std::fabs(l.y - r.y) < 1e-1;
  }
};

using polyhashmap = std::unordered_map<geometry::Site, int, KeyHash, KeyEqual>;

// �����
struct PolyNode {
  SiteVec poly;
  SiteVec os_poly;
  bool if_thrink;
  int ind;
  Site inner_pt;  // no use
  int outer_ind;
  Site outer_pt;

  PolyNode() {
    poly.clear();
    os_poly.clear();
    if_thrink = true;
    ind = 0;
    outer_ind = 0;
  }

  PolyNode(const PolyNode &p) {
    poly.clear();
    poly.insert(poly.end(), p.poly.begin(), p.poly.end());
    os_poly.clear();
    os_poly.insert(os_poly.end(), p.os_poly.begin(), p.os_poly.end());
    if_thrink = p.if_thrink;
    ind = p.ind;
    inner_pt = p.inner_pt;
    outer_ind = p.outer_ind;
    outer_pt = p.outer_pt;
  }

  explicit PolyNode(const geometry::SiteVec &p) {
    poly.clear();
    poly.insert(poly.end(), p.begin(), p.end());
    os_poly.clear();
    if_thrink = true;
    ind = 0;
    outer_ind = 0;
  }

  void operator=(const PolyNode &p) {
    poly.clear();
    poly.insert(poly.end(), p.poly.begin(), p.poly.end());
    os_poly.clear();
    os_poly.insert(os_poly.end(), p.os_poly.begin(), p.os_poly.end());
    if_thrink = p.if_thrink;
    ind = p.ind;
    outer_ind = p.outer_ind;
    inner_pt = p.inner_pt;
    outer_pt = p.outer_pt;
  }
};

const double kOffsetWidth = 1.0;

/// �ⶫ��˼·���ұȽ���
class HybridTrack {
 public:
  HybridTrack();

  ~HybridTrack() = default;

  bool Interface(const std::vector<double> &x, const std::vector<double> &y,
                 const double x_orig = 0.0, const double y_orig = 0.0);

  bool Planning(int key);

 private:
  bool CheckRingInPoly(const PolyNode &poly, std::vector<PolyNode> &subpoly);

  bool RelationBetweenPolys(PolyNode &poly1, PolyNode &poly2);

  std::vector<std::pair<int, int>> FindPtPair(const PolyNode &poly);

  bool FindSubpoly(const PolyNode &poly,
                   const std::vector<std::pair<int, int>> &pt_pair,
                   std::vector<PolyNode> &subpoly);

  bool ConnectionValid(const PolyNode &poly, const std::pair<int, int> &pair1,
                       const std::pair<int, int> &pair2,
                       std::vector<PolyNode> &subpoly);

  // ����ʵ�ֵ�poly offset���Ӻö�
  bool PolyOffset(double offsetdis, const SiteVec &orig, SiteVec &poly,
                  bool direction, std::vector<int> &match);

  bool PolyOffsetClipper(double offsetdis, const SiteVec &orig, SiteVec &poly,
                         bool direction, std::vector<int> &match);

  // ʵ��ִ��shrink�ĵط�
  // @params[in] polys ����Ķ����
  // @params[out] output_poly ����֮��Ķ����
  // @params[in] offset ƫ����
  // @returns �Ƿ�ɹ�
  bool WeltPathGeneration(std::vector<PolyNode> &polys, SiteVec &output_poly,
                          double offset = 0.55);

  int GetNearestIndex(const Site &p, const SiteVec &poly);

  bool PathConnection(const SiteVec &poly);

  // �߶β�����Բ������
  // �������Ǹĳ�Bline
  bool BSampling(const Site &start, const Site &end, const Site &next_direction,
                 const Site &last_direction, std::vector<Site> &sample);

  bool LineSampling(const geometry::Site &start, const geometry::Site &end,
                    std::vector<geometry::Site> &sample);

  bool ArcSampling(const geometry::Site &cent, const geometry::Site &start,
                   const geometry::Site &end, const double cross,
                   std::vector<geometry::Site> &sample);

 public:
  SiteVec final_path_;  // ����·��
  SiteVec orig_edge_;   // ԭʼ�ⶥ��

 private:
  double offset_dis_;
  bool status_;  // true for success
  double x_offset_;
  double y_offset_;
  Site start_pt_;
  int start_ind_;
  // Site end_pt_;

  // BSpline params
  double b0_pos_ = 0.1, b0_para_ = 0.02;
  double b1_pos_ = 0.9, b1_para_ = 0.02;
};

}  // namespace coverage

#endif  // INCLUDE_PATHPLANNER_BLANKET_HYBRIDTRACK_H_
