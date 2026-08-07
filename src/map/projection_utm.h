
#pragma once

#include "map/map_define.h"

namespace geditor {

class ProjectionUTM {
 public:
  void CartesianToLatLon(double x, double y, int zone, bool southhemi,
                         LatLon &latlon);

  void LatLonToCartesian(double lat, double lon, UTMPoint &xy);

  // Convert a point in the local East-North-Up frame anchored at a mine
  // origin to WGS84 latitude/longitude/altitude. Local mine coordinates are
  // not UTM deltas: UTM grid north is rotated from true north by the meridian
  // convergence angle, which can otherwise introduce errors of tens of
  // metres over a mine-sized point cloud.
  void LocalENUToGPS(double east, double north, double up,
                     double origin_lat, double origin_lon,
                     double origin_alt, GPSPoint &gps);

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
