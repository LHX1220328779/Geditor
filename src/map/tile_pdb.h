
#pragma once

#include <vector>
#include "map/map_define.h"

namespace geditor {

struct PDBPoint {
  LatLon latlon;
  float altitude = 0;
  float height = 0;
  float intensity = 0;
  float reserve = 0;

  PDBPoint() : intensity(0.0) {}

  PDBPoint(double lat, double lon, float alt, float insty, float hit)
      : latlon(alt, lon),
        altitude(alt),
        height(hit),
        intensity(insty),
        reserve(0.0) {}

  PDBPoint(const LatLon &latlon, float alt, float insty, float hit)
      : latlon(latlon),
        altitude(alt),
        height(hit),
        intensity(insty),
        reserve(0.0) {}
};

class TilePDB {
 public:
  TilePDB(const TileGrid &grid);

  ~TilePDB();

 public:
  void Add(const LatLon &latlon, float altitude, float intensity, float height);

  int GetPointCount();

  TileGrid GetTileGrid() { return grid_; }

  GPSPoint GetMinPoint() { return min_point_; }

  GPSPoint GetMaxPoint() { return max_point_; }

  GPSPoint GetCenterPoint();

  int DumpMemory(char *buffer, int bufLen);

 private:
  TileGrid grid_;

  GPSPoint min_point_;
  GPSPoint max_point_;

 public:
  std::vector<PDBPoint> point_cloud_;
};

}  // namespace geditor
