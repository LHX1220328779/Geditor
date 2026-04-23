
#pragma once

#include "map/map_define.h"

namespace geditor {

class ProjectionMercator {
 public:
  void CartesianToLatLon(double x, double y, LatLon &latlon);

  void LatLonToCartesian(double lat, double lon, UTMPoint &xy);
};

}  // namespace geditor
