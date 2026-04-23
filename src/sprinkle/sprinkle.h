
#pragma once

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include "sprinkle/sprinkle_tool.h"

namespace geditor {
namespace sprinkle {

class Sprinkle {
 public:
  Sprinkle(std::vector<Polygon> &Input, SprinkleConf &Conf,
           std::vector<Point2D> &SprinklePoints,
           std::vector<std::pair<int, int>> &Tprelationship);

  ~Sprinkle();

  void GridSprinkle(std::vector<Point2D> &SprinklePoints,
                    std::vector<Point2D> PolyonPoints, int Zone);

  int GetTopological(std::vector<Point2D> &SprinklePoints,
                     std::vector<Polygon> &Input,
                     std::vector<std::pair<int, int>> &Output);

  bool RemovePointInOverlapArea(std::vector<Point2D> &SprinklePoints,
                                Point2D Point);

  int IsInPolygon(std::vector<Point2D> PolyonPoints, Point2D Point);

  bool IsLinePolyIntersect(Point2D StartPoint, Point2D EndPoint,
                           std::vector<Polygon> InputPolyon);

  float PointToSegDist(const Point2D &P1, const Point2D &P2, const Point2D &Pt);

  double Distance(Point2D PA, Point2D PB);
  // k-d tree API

 private:
  PolygonParm PyParam;
  SprinkleConf SpParm;
  std::vector<SprinkleData> SprinklePointsOfSize;
  // k-d tree param
};
}  // namespace sprinkle
}  // namespace geditor
