
#include "map/projection_mercator.h"
#include <cmath>

namespace geditor {

double pi = 3.1415926535897932384626433832795;

void ProjectionMercator::CartesianToLatLon(double mercatorx, double mercatory,
                                           LatLon &latlon) {
  double x = mercatorx / 20037508.34 * 180;
  double y = mercatory / 20037508.34 * 180;
  y = 180 / pi * (2 * atan(exp(y * pi / 180)) - pi / 2);
  latlon.lat = y;
  latlon.lon = x;
}

void ProjectionMercator::LatLonToCartesian(double lat, double lon,
                                           UTMPoint &mercator) {
  double x = lon * 20037508.34 / 180;
  double y = log(tan((90 + lat) * pi / 360)) / (pi / 180);
  y = y * 20037508.34 / 180;
  mercator.x = x;
  mercator.y = y;
}

}  // namespace geditor
