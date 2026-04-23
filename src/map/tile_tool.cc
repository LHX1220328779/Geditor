#include "map/tile_tool.h"
#include <cstring>

namespace geditor {

const double TileTools::origin_lat_ = 16;
const double TileTools::origin_lon_ = 72;

const char *TileTools::alphabet_ = "0123456789ABCDEFGHJKMNPQRSTUWXYZ";

void TileTools::Grid2LatLon(int tileX, int tileY, LatLon &pLatLon) {
  pLatLon.lat = tileY * 2.0 / 3600 + origin_lat_;
  pLatLon.lon = tileX * 2.0 / 3600 + origin_lon_;
}

void TileTools::LatLon2Grid(double lat, double lon, TileGrid &grid) {
  double tileY = (lat - origin_lat_) * 3600 / 2.0;
  double tileX = (lon - origin_lon_) * 3600 / 2.0;

  grid.x = (int)tileX;
  grid.y = (int)tileY;
}

bool TileTools::ZoneCode2Grid(char *szTile, TileGrid &grid) {
  int len = strlen(szTile);
  if (len != 9) {
    return false;
  }

  long long tileCode = 0;

  for (int i = 1; i < len; i++) {
    bool notFind = true;
    for (int pos = 0; pos < 32; pos++) {
      if (szTile[i] == alphabet_[pos]) {
        tileCode = (tileCode << 5) | pos;
        notFind = false;
        break;
      }
    }

    if (notFind) {
      return false;
    }
  }

  grid.x = (tileCode >> 20) & 0xfffff;
  grid.y = (tileCode & 0xfffff);

  return true;
}

bool TileTools::Grid2ZoneCode(int tileX, int tileY, char *szTile) {
  long long titleCode = ((long long)tileX) << 20 | ((long long)tileY);

  szTile[0] = 'W';

  for (int i = 0; i < 8; i++) {
    int bits = i * 5;
    int idx = (titleCode >> bits) & 0x1f;

    int nlen = strlen(alphabet_);
    if (idx >= nlen) {
      return false;
    }

    szTile[8 - i] = alphabet_[idx];
  }

  szTile[9] = 0;

  return true;
}

void TileTools::GridCenter(int tileX, int tileY, LatLon &pLatLon) {
  pLatLon.lat = (tileY * 2.0 + 1) / 3600 + origin_lat_;
  pLatLon.lon = (tileX * 2.0 + 1) / 3600 + origin_lon_;
}

}  // namespace geditor
