
#include <memory.h>
#include <minilzo.h>
#include <cstdint>

#include "map/pdb_define.h"
#include "map/tile_pdb.h"
#include "utils/utils.h"

namespace geditor {

TilePDB::TilePDB(const TileGrid &grid) : grid_(grid) {
  point_cloud_.reserve(1024);

  max_point_.latlon = LatLon(-90.0, -180.0);
  max_point_.altitude = -20000.0f;

  min_point_.latlon = LatLon(90.0, 180.0);
  min_point_.altitude = 20000.0f;
}

void TilePDB::Add(const LatLon &latlon, float altitude, float intensity,
                  float height) {
  if (altitude < min_point_.altitude) {
    min_point_.altitude = altitude;
  }
  if (altitude > max_point_.altitude) {
    max_point_.altitude = altitude;
  }

  if (latlon.lat < min_point_.latlon.lat) {
    min_point_.latlon.lat = latlon.lat;
  }
  if (latlon.lat > max_point_.latlon.lat) {
    max_point_.latlon.lat = latlon.lat;
  }

  if (latlon.lon < min_point_.latlon.lon) {
    min_point_.latlon.lon = latlon.lon;
  }
  if (latlon.lon > max_point_.latlon.lon) {
    max_point_.latlon.lon = latlon.lon;
  }

  point_cloud_.push_back(PDBPoint(latlon, altitude, intensity, height));
}

int TilePDB::GetPointCount() { return point_cloud_.size(); }

GPSPoint TilePDB::GetCenterPoint() {
  float altitude = (max_point_.altitude + min_point_.altitude) * 0.5;
  double lon = (max_point_.latlon.lon + min_point_.latlon.lon) * 0.5;
  double lat = (max_point_.latlon.lat + min_point_.latlon.lat) * 0.5;

  return GPSPoint(lat, lon, altitude);
}

lzo_align_t __LZO_MMODEL
    wrkmem[((LZO1X_1_MEM_COMPRESS) + (sizeof(lzo_align_t) - 1)) /
           sizeof(lzo_align_t)];

int TilePDB::DumpMemory(char *buffer, int bufLen) {
  int datalen = 0;
  int nCount = point_cloud_.size();

  PDB_HDR pdbHdr;

  pdbHdr.magic = 0x00424450;
  pdbHdr.version = 2;
  pdbHdr.reserve = 0;
  pdbHdr.crc32 = 0;

  memcpy(buffer + datalen, &pdbHdr, sizeof(PDB_HDR));
  datalen += sizeof(PDB_HDR);

  //-------------------------

  PDB_INFO infoHdr;

  infoHdr.count = nCount;
  infoHdr.orginAlt = 0.0;
  infoHdr.orginLat = 0.0;
  infoHdr.orginLon = 0.0;
  infoHdr.minLat = min_point_.latlon.lon;
  infoHdr.maxLat = max_point_.latlon.lat;
  infoHdr.minLon = min_point_.latlon.lat;
  infoHdr.maxLon = max_point_.latlon.lon;
  infoHdr.minAlt = min_point_.altitude;
  infoHdr.maxAlt = max_point_.altitude;
  ;

  memcpy(buffer + datalen, &infoHdr, sizeof(PDB_INFO));
  datalen += sizeof(PDB_INFO);

  //-------------------------------------------
  if (nCount > 0) {
    int nBytes = sizeof(PDBPoint) * nCount;

    lzo_uint out_len = nBytes + nBytes / 16 + 64 + 3;
    unsigned char *out = new unsigned char[out_len];

    int r = lzo1x_1_compress((unsigned char *)&point_cloud_[0], nBytes, out,
                             &out_len, wrkmem);
    if (r == LZO_E_OK) {
      if (datalen + out_len < bufLen) {
        memcpy(buffer + datalen, out, out_len);
        datalen += out_len;

        delete[] out;
      } else {
        delete[] out;
        return false;
      }
    } else {
      delete[] out;
      return false;
    }
  }
  //-------------------------------------------

  unsigned int crc32 = Utils::CalcCrc32(buffer, datalen);

  /// FIXME 0指针强转???
  intptr_t offset = intptr_t(&(((PDB_HDR *)0)->crc32));
  *(unsigned int *)(buffer + offset) = crc32;

  return datalen;
}

TilePDB::~TilePDB() { point_cloud_.clear(); }

}  // namespace geditor