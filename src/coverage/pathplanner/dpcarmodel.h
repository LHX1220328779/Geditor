#ifndef INCLUDE_PATHPLANNER_DPCARMODEL_H_
#define INCLUDE_PATHPLANNER_DPCARMODEL_H_

#include <cmath>
#include <set>
#include <vector>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace pathplanner {
namespace dpplanner {

struct Dppoint {
  double x;
  double y;
  double angle;
  double s;
  double l;
  Dppoint() : x(0.0), y(0.0), angle(0.0), s(0.0), l(0.0) {}
  Dppoint(double _x, double _y, double _angle) : x(_x), y(_y), angle(_angle) {}
  Dppoint(double _s, double _l) : s(_s), l(_l) {}
  Dppoint &operator=(const Dppoint &dppoint) {
    x = dppoint.x;
    y = dppoint.y;
    angle = dppoint.angle;
    s = dppoint.s;
    l = dppoint.l;
    return *this;
  }
  Dppoint(const Dppoint &dppoint) {
    x = dppoint.x;
    y = dppoint.y;
    angle = dppoint.angle;
    s = dppoint.s;
    l = dppoint.l;
  }
};

static double NormalizeRad(const double rad) {
  double a = std::fmod(rad + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += (2.0 * M_PI);
  return a - M_PI;
}

static double NormalizeDeg(const double deg) {
  double a = std::fmod(deg + 180.0, 360.0);
  if (a < 0.0) a += 360.0;
  return a - 180.0;
}

static double Deg2Rad(const double deg) {
  double cur = NormalizeDeg(deg);
  return cur * M_PI / 180.0;
}

static double Rad2Deg(const double rad) {
  double cur = NormalizeRad(rad);
  return cur * 180.0 / M_PI;
}

static void CarModel(std::vector<Dppoint> &polygon,
                     std::vector<Dppoint> &boundary, const Dppoint ego,
                     const double headlength, const double taillength,
                     const double halfwidth) {
  polygon.clear();
  boundary.clear();
  Dppoint pt, ld, ru;
  std::set<double> cmpx, cmpy;
  double hyaw = Deg2Rad(ego.angle);
  double vyaw = hyaw + M_PI / 2.0;
  pt.x = ego.x + headlength * cos(hyaw) - halfwidth * cos(vyaw);
  pt.y = ego.y + headlength * sin(hyaw) - halfwidth * sin(vyaw);
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.push_back(pt);
  pt.x = ego.x + headlength * cos(hyaw) + halfwidth * cos(vyaw);
  pt.y = ego.y + headlength * sin(hyaw) + halfwidth * sin(vyaw);
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.push_back(pt);
  pt.x = ego.x - taillength * cos(hyaw) + halfwidth * cos(vyaw);
  pt.y = ego.y - taillength * sin(hyaw) + halfwidth * sin(vyaw);
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.push_back(pt);
  pt.x = ego.x - taillength * cos(hyaw) - halfwidth * cos(vyaw);
  pt.y = ego.y - taillength * sin(hyaw) - halfwidth * sin(vyaw);
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.push_back(pt);
  ld.x = *(cmpx.begin());
  ld.y = *(cmpy.begin());
  ru.x = *(cmpx.rbegin());
  ru.y = *(cmpy.rbegin());
  boundary.push_back(ld);
  boundary.push_back(ru);
  return;
}

static bool IsInsideFootprint(const Dppoint pt,
                              const std::vector<Dppoint> &poly) {
  int counter = 0;
  int i;
  double xinters;
  Dppoint p1;
  Dppoint p2;
  int N = poly.size();
  p1 = poly.at(0);
  for (i = 1; i <= N; i++) {
    p2 = poly.at(i % N);
    if (pt.y >= std::min<float>(p1.y, p2.y)) {
      if (pt.y <= std::max<float>(p1.y, p2.y)) {
        if (pt.x <= std::max<float>(p1.x, p2.x)) {
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

}  // namespace dpplanner
}  // namespace pathplanner

#endif  // INCLUDE_PATHPLANNER_DPCARMODEL_H_
