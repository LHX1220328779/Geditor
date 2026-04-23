
#pragma once

#include "map/map_define.h"

namespace geditor {

class TileTools {
 public:
  static void Grid2LatLon(int tileX, int tileY, LatLon &pLatLon);

  static void LatLon2Grid(double lat, double lon, TileGrid &grid);

  static bool ZoneCode2Grid(char *szTile, TileGrid &grid);

  static bool Grid2ZoneCode(int tileX, int tileY, char *szTile);

  static void GridCenter(int tileX, int tileY, LatLon &pCenter);

 private:
  static const double origin_lat_;
  static const double origin_lon_;
  static const char *alphabet_;
};

}  // namespace geditor
