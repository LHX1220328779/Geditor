
#pragma once

#include <cstdint>
#include <vector>
#include "map/map_define.h"

namespace geditor {

struct PDBPoint {
  LatLon latlon;
  float altitude = 0;
  float height = 0;
  float intensity = 0;
  // Packed 0x00RRGGBB source color. This reuses the former four-byte reserve
  // field, so version-2 PDB point layout and old databases stay compatible.
  std::uint32_t rgb = 0;

  PDBPoint() : intensity(0.0) {}

  PDBPoint(double lat, double lon, float alt, float insty, float hit,
           std::uint32_t source_rgb = 0)
      : latlon(lat, lon),
        altitude(alt),
        height(hit),
        intensity(insty),
        rgb(source_rgb) {}

  PDBPoint(const LatLon &latlon, float alt, float insty, float hit,
           std::uint32_t source_rgb = 0)
      : latlon(latlon),
        altitude(alt),
        height(hit),
        intensity(insty),
        rgb(source_rgb) {}
};

class TilePDB {
 public:
  TilePDB(const TileGrid &grid);

  ~TilePDB();

 public:
  void Add(const LatLon &latlon, float altitude, float intensity, float height,
           std::uint32_t rgb = 0);

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
