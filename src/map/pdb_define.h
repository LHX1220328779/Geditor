
#pragma once

namespace geditor {

struct PDB_HDR {
  unsigned int magic = 0;
  unsigned short version = 0;
  unsigned short reserve = 0;
  unsigned int crc32 = 0;
};

struct PDB_INFO {
  int count = 0;
  float orginAlt = 0;
  double orginLat = 0;
  double orginLon = 0;
  double minLat = 0;
  double maxLat = 0;
  double maxLon = 0;
  double minLon = 0;
  float minAlt = 0;
  float maxAlt = 0;
};

}  // namespace geditor
