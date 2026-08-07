
#pragma once

#include "map/tile_pdb.h"

namespace SQLite {
class Database;
}

namespace geditor {

class PDBManage {
 public:
  PDBManage();

  virtual ~PDBManage();

  bool Open(const char *filename);

  void Close();

  bool QueryBound(GPSPoint &minGPSPt, GPSPoint &maxGPSPt);

  bool SetUTMZone(int zone);

  bool GetUTMZone(int &zone);

  bool SetPointColorModeRGB(bool enabled);

  bool UsesPointColorRGB();

  bool Save(TilePDB *tile);

  bool Read(TilePDB *tile);

 private:
  SQLite::Database *m_database;
};

}  // namespace geditor
