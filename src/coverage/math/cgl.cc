#ifndef SRC_MATH_CGL_CPP__
#define SRC_MATH_CGL_CPP__

#include "math/cgl.h"
#include <algorithm>
#include <iostream>
#include <limits>

#include <opencv2/opencv.hpp>

namespace math {
namespace cgl {

double CrossProduct(const Site& p1, const Site& p2) {
  return p1.x * p2.y - p1.y * p2.x;
}

double CrossProduct(const Site& p1, const Site& p2, const Site& p3,
                    const Site& p4) {
  return CrossProduct(p2 - p1, p4 - p3);
}

double CrossProduct(const Site& p, const Site& p1, const Site& p2) {
  return CrossProduct(p1 - p, p2 - p);
}

double DotProduct(const Site& p1, const Site& p2) {
  return p1.x * p2.x + p1.y * p2.y;
}

double DotProduct(const Site& p1, const Site& p2, const Site& p3,
                  const Site& p4) {
  return DotProduct(p2 - p1, p4 - p3);
}

double DotProduct(const Site& p, const Site& p1, const Site& p2) {
  return DotProduct(p1 - p, p2 - p);
}

bool IfAutoclockwise(const SiteVec& poly) {
  if (poly.size() < 3) return false;
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
  double cross = CrossProduct(poly[right_index] - poly[before_index],
                              poly[after_index] - poly[right_index]);
  if (cross > 0.0) return true;
  return false;
}

bool SameDirection(const Site& p1, const Site& p2) {
  if (p1.mold() < 1e-2 || p2.mold() < 1e-2) return false;
  if ((p1.direction() - p2.direction()).mold() < 1e-2) return true;
  return false;
}

bool IsParallel(const Site& p1, const Site& p2) {
  // std::cout << "p1" << p1.x << "," << p1.y << std::endl;
  // std::cout << "p2" << p2.x << "," << p2.y << std::endl;
  if (p1.mold() < 1e-2 || p2.mold() < 1e-2) return false;
  // std::cout << "hello again" << std::endl;
  if (std::fabs(CrossProduct(p1, p2)) < 1e-2) return true;
  // std::cout << "hello again1" << std::endl;
  return false;
}

bool CheckCrossOfSegments(const Site& p1, const Site& p2, const Site& p3,
                          const Site& p4) {
  if (std::max(p1.x, p2.x) < std::min(p3.x, p4.x)) return false;
  if (std::max(p1.y, p2.y) < std::min(p3.y, p4.y)) return false;
  if (std::max(p3.x, p4.x) < std::min(p1.x, p2.x)) return false;
  if (std::max(p3.y, p4.y) < std::min(p1.y, p2.y)) return false;

  if ((p1 - p3).mold() < 1e-3 || (p1 - p4).mold() < 1e-3 ||
      (p2 - p3).mold() < 1e-3 || (p2 - p4).mold() < 1e-3)
    return true;

  double cond1 = CrossProduct(p3, p1, p4) * CrossProduct(p3, p2, p4);
  double cond2 = CrossProduct(p1, p3, p2) * CrossProduct(p1, p4, p2);
  if (cond1 < 0 && cond2 < 0) return true;
  return false;
}

bool CheckLeftOfEdge(const Site& p, const Site& p1, const Site& p2) {
  // double tmpx = (p1.x - p2.x) / (p1.y - p2.y) * (p.y - p2.y) + p2.x;
  // if (tmpx > p.x) return true;
  // return false;
  // Tmp = (y1 ¨C y2) * x + (x2 ¨C x1) * y + x1 * y2 ¨C x2 * y1;
  // double tmp = (p1.y - p2.y) * p.x +
  //              (p2.x - p1.x) * p.y +
  //              CrossProduct(p1, p2);
  // if (tmp > 0) return true;
  // return false;
  double tmp = (p1 - p).x * (p2 - p).y - (p1 - p).y * (p2 - p).x;
  if (tmp > 0) return true;
  return false;
}

bool GetCrossPtOfSegments(const Site& p1, const Site& p2, const Site& p3,
                          const Site& p4, Site& p) {
  if (!CheckCrossOfSegments(p1, p2, p3, p4)) return false;
  if (!GetCrossPtOfLines(p1, p2, p3, p4, p)) return false;
  return true;
}

bool GetCrossPtOfLines(const Site& p1, const Site& p2, const Site& p3,
                       const Site& p4, Site& p) {
  if (IsParallel(p1 - p2, p3 - p4)) return false;
  // Site delta1(p2.x - p1.x, -(p4.x - p3.x));
  // Site delta2(p2.y - p1.y, -(p4.y - p3.y));
  // Site lamda1(p3.x - p1.x, -(p4.x - p3.x));
  // Site lamda2(p3.y - p1.y, -(p4.y - p3.y));
  // // geometry::Site mue1(p2.x - p1.x, p3.x - p1.x);
  // // geometry::Site mue2(p2.y - p1.y, p3.y - p1.y);
  // double delta = CrossProduct(delta1, delta2);
  // double lamda = CrossProduct(lamda1, lamda2) / delta;
  // // double mue = Mult(mue1, mue2) / delta;
  // p.x = p1.x + delta * (p2.x - p1.x);
  // p.y = p1.y + delta * (p2.y - p2.y);
  // return true;
  double a1 = p2.y - p1.y;
  double b1 = p1.x - p2.x;
  double c1 = p1.x * p2.y - p1.y * p2.x;
  double a2 = p4.y - p3.y;
  double b2 = p3.x - p4.x;
  double c2 = p3.x * p4.y - p3.y * p4.x;
  double d = a1 * b2 - a2 * b1;
  // geometry::Site pt;
  p.y = (a1 * c2 - a2 * c1) / d;
  p.x = (c1 * b2 - c2 * b1) / d;
  return true;
}

double GetDisFromPtToLine(const Site& pt, const Site& p1, const Site& p2) {
  double a = (pt.x - p1.x) * (p2.y - p1.y) + (pt.y - p1.y) * (p1.x - p2.x);
  double b = (p2.y - p1.y) * (p2.y - p1.y) + (p1.x - p2.x) * (p1.x - p2.x);
  double result = sqrt(pow(a, 2) / b);
  return result;
}

double GetDisFromPtToPolyEdge(const Site& pt, const Site& p1, const Site& p2) {
  Site p12 = p1 - p2, p21 = p2 - p1, p1t = pt - p1, p2t = pt - p2;
  if (DotProduct(p12, p1t) < 0) {
    return p1t.mold();
  }
  if (DotProduct(p21, p2t) < 0) {
    return p2t.mold();
  }
  return GetDisFromPtToLine(pt, p1, p2);
}

bool IsInside(const Site& pt, const SiteVec& polygon) {
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::min();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::min();
  for (const auto& p : polygon) {
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

Site GravityPt(const Site& p1, const Site& p2, const Site& p3) {
  double x = (p1.x + p2.x + p3.x) / 3;
  double y = (p1.y + p2.y + p3.y) / 3;
  return Site(x, y);
}

Site IncenterPt(const Site& p1, const Site& p2, const Site& p3) {
  double A = (p2 - p3).mold();
  double B = (p1 - p3).mold();
  double C = (p1 - p2).mold();
  double S = A + B + C;
  double x = (A * p1.x + B * p2.x + C * p3.x) / S;
  double y = (A * p1.y + B * p2.y + C * p3.y) / S;
  return Site(x, y);
}

Site CircumPt(const Site& a, const Site& b, const Site& c) {
  double x1 = a.x;
  double y1 = a.y;
  double x2 = b.x;
  double y2 = b.y;
  double x3 = c.x;
  double y3 = c.y;

  double a1 = 2 * (x2 - x1);
  double b1 = 2 * (y2 - y1);
  double c1 = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;

  double a2 = 2 * (x3 - x2);
  double b2 = 2 * (y3 - y2);
  double c2 = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;

  double x = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
  double y = (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);
  return Site(x, y);
}

Site OrthoPt(const Site& a, const Site& b, const Site& c) {
  double A1 = b.x - c.x;
  double B1 = b.y - c.y;
  double C1 = A1 * a.y - B1 * a.x;

  double A2 = a.x - c.x;
  double B2 = a.y - c.y;
  double C2 = A2 * b.y - B2 * b.x;

  double x = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);
  double y = (B1 * C2 - B2 * C1) / (A2 * B1 - A1 * B2);

  return Site(x, y);
}

int CheckCrossSeg2Poly(const Site& p1, const Site& p2, const SiteVec& poly,
                       int& ind, double& dis) {
  std::vector<int> cross_inds;
  for (std::size_t i = 0; i < poly.size(); i++) {
    int next_i = i == poly.size() - 1 ? 0 : i + 1;
    if (CheckCrossOfSegments(p1, p2, poly[i], poly[next_i])) {
      cross_inds.push_back(i);
    }
  }
  if (cross_inds.size() == 0) return 0;
  if (cross_inds.size() > 2 || cross_inds.size() == 1) return -1;
  if ((cross_inds[1] - cross_inds[0]) > 1) {
    if (cross_inds[0] == 0 && cross_inds[1] == poly.size() - 1) {
      ind = 0;
      dis = GetDisFromPtToLine(poly[ind], p1, p2);
      return 1;
    } else {
      return -1;
    }
  } else {
    ind = cross_inds[1];
    dis = GetDisFromPtToLine(poly[ind], p1, p2);
    return 1;
  }
}

double DisFromPt2Poly(const Site& p, const SiteVec& poly) {
  if (poly.empty()) return 0;
  std::vector<double> distance;
  for (std::size_t i = 0; i < poly.size(); i++) {
    int next_i = i == poly.size() - 1 ? 0 : i + 1;
    distance.push_back(GetDisFromPtToPolyEdge(p, poly[i], poly[next_i]));
  }
  return *std::min_element(distance.begin(), distance.end());
}

double AreaOfPoly(const SiteVec& poly) {
  std::vector<cv::Point2f> pt_opt_cv;
  for (auto& p : poly) {
    pt_opt_cv.push_back(cv::Point2f(p.x, p.y));
  }
  double area_opt = cv::contourArea(pt_opt_cv);
  return area_opt;
}

}  // namespace cgl
}  // namespace math

#endif