
#pragma once

#include <vector>

#include "core/point3d.h"
#include "core/point_color.h"

namespace geditor {

struct Rect {
  double left = 0;
  double top = 0;
  double bottom = 0;
  double right = 0;
};

void SamplePoint(std::vector<Point3d> &linePoint, const Point3d &p0,
                 const Point3d &p1, const Point3d &p2, const Point3d &p3,
                 double d = 1.);

void B3Line(const std::vector<Point3d> &controlPoint,
            std::vector<Point3d> &linePoint, double d = 1.);

int IsOnLine(const std::vector<Point3d> &controlPoint,
             std::vector<Point3d> &mousePoint, const Point3d &pt, int &nIndex,
             bool bInsert = false);

bool EditMousePoint(std::vector<Point3d> &mousePoint, int nIndex,
                    const Point3d &movePt);

std::vector<Point3d> FindXY(const std::vector<Point3d> &controlPoint,
                            const std::vector<Point3d> &mousePoint, int nXY,
                            bool bIsX);

}  // namespace geditor
