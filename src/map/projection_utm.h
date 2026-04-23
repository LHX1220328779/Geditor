
#pragma once

#include "map/map_define.h"

namespace geditor {

class ProjectionUTM {
 public:
  void CartesianToLatLon(double x, double y, int zone, bool southhemi,
                         LatLon &latlon);

  void LatLonToCartesian(double lat, double lon, UTMPoint &xy);

  static void SetZoneByLon(double lon) { zone = lon / 6.0 + 31; }

 private:
  void MapXYToLatLon(double x, double y, double lambda0, LatLon &philambda);

  void MapLatLonToXY(double phi, double lambda, double lambda0, UTMPoint &xy);

  double ArcLengthOfMeridian(double phi);

  double FootpointLatitude(double y);

  double UTMCentralMeridian(int zone);

 private:
  static double pi;

  static double sm_a;
  static double sm_b;
  static double sm_EccSquared;
  static double UTMScaleFactor;

 public:
  static int zone;
};

}  // namespace geditor
