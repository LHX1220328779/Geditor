
#include <glog/logging.h>
#include <minilzo.h>
#include <sqlite_cpp.h>

#include "map/pdb_define.h"
#include "map/pdb_manage.h"
#include "map/projection_utm.h"
#include "map/tile_tool.h"
#include "platform/date_time.h"
#include "utils/utils.h"

namespace geditor {

PDBManage::PDBManage() : m_database(NULL) {}

PDBManage::~PDBManage() {
  if (m_database) {
    delete m_database;
    m_database = NULL;
  }
}

bool PDBManage::QueryBound(GPSPoint &minGPSPt, GPSPoint &maxGPSPt) {
  minGPSPt.latlon.lat = 90.0;
  minGPSPt.latlon.lon = 180.0;
  minGPSPt.altitude = 50000;

  maxGPSPt.latlon.lat = -90.0;
  maxGPSPt.latlon.lon = -180.0;
  maxGPSPt.altitude = -50000;

  const char *szSql =
      "select east, south, west, north, minAlt, maxAlt from TileTb;";
  SQLite::Statement query(*m_database, szSql);
  int zone = -1;
  while (query.executeStep()) {
    double dEast = query.getColumn(0).getDouble();
    double dSouth = query.getColumn(1).getDouble();
    double dWest = query.getColumn(2).getDouble();
    double dNorth = query.getColumn(3).getDouble();
    double dMinAlt = query.getColumn(4).getDouble();
    double dMaxAlt = query.getColumn(5).getDouble();
    int nzone = (int)(dSouth / 6.0 + 31);
    if (zone >= 0 && zone != nzone) {
      LOG(WARNING) << "utm zone changed:" << zone << "->" << nzone;
      // break;
      minGPSPt.latlon.lat = 90.0;
      minGPSPt.latlon.lon = 180.0;
      minGPSPt.altitude = 50000;

      maxGPSPt.latlon.lat = -90.0;
      maxGPSPt.latlon.lon = -180.0;
      maxGPSPt.altitude = -50000;
    }
    zone = nzone;
    if (minGPSPt.latlon.lat > dWest) minGPSPt.latlon.lat = dWest;
    if (maxGPSPt.latlon.lat < dEast) maxGPSPt.latlon.lat = dEast;
    if (minGPSPt.latlon.lon > dSouth) minGPSPt.latlon.lon = dSouth;
    if (maxGPSPt.latlon.lon < dNorth) maxGPSPt.latlon.lon = dNorth;
    if (minGPSPt.altitude > dMinAlt) minGPSPt.altitude = dMinAlt;
    if (maxGPSPt.altitude < dMaxAlt) maxGPSPt.altitude = dMaxAlt;
  }
  ProjectionUTM::zone = zone;
  LOG(INFO) << "utm zone:" << ProjectionUTM::zone;
  return true;
}

bool PDBManage::Open(const char *filename) {
  m_database = new SQLite::Database(filename);
  if (m_database->open()) {
    const char *szSql =
        "create table if not exists TileTb(zone integer not null, tileX "
        "integer not null, tileY integer not null, east real , south real, "
        "west real, north real, minAlt real, maxAlt real, pcd blob, date "
        "text,primary key(zone, tileX, tileY));";

    int ret = m_database->exec(szSql);
    if (ret == 0) {
      const char *szSqlFind =
          "select south, north from TileTb where rowid = 1 ";
      SQLite::Statement query(*m_database, szSqlFind);

      while (query.executeStep()) {
        double south = query.getColumn(0).getDouble();
        double north = query.getColumn(1).getDouble();

        double northCnt = (south + north) * 0.5;
        int nzone = (int)(northCnt / 6.0 + 31);

        ProjectionUTM::zone = nzone;
      }
      return true;
    }
    m_database->close();
  }
  return false;
}

bool PDBManage::Save(TilePDB *tile) {
  int iCount = tile->point_cloud_.size();

  int bufLen = (iCount + iCount / 10) * sizeof(PDBPoint) + 256;
  char *szBuffer = new char[bufLen];

  int size = tile->DumpMemory(szBuffer, bufLen);
  if (size < 0) {
    delete[] szBuffer;
    return true;
  }

  const char *szSql =
      "insert into TileTb(zone, tileX, tileY, east, south, west, north, "
      "minAlt, maxAlt,pcd, date) values(? , ? , ? ,? , ? ,? , ? , ?, ?, ?, ? )";
  SQLite::Statement query(*m_database, szSql);

  TileGrid grid = tile->GetTileGrid();
  GPSPoint minPt = tile->GetMinPoint();
  GPSPoint maxPt = tile->GetMaxPoint();

  char szDate[64];
  DateTime dateTime;
  dateTime.SetToNow();
  dateTime.Format(szDate, 64);

  query.bind(1, grid.zoom);
  query.bind(2, grid.x);
  query.bind(3, grid.y);
  query.bind(4, maxPt.latlon.lat);
  query.bind(5, minPt.latlon.lon);
  query.bind(6, minPt.latlon.lat);
  query.bind(7, maxPt.latlon.lon);
  query.bind(8, minPt.altitude);
  query.bind(9, maxPt.altitude);
  query.bind(10, szBuffer, size);
  query.bind(11, szDate);

  int ret = query.exec();
  if (ret > 0) {
    delete[] szBuffer;
    return true;
  } else {
    delete[] szBuffer;

    return false;
  }
}

bool PDBManage::Read(TilePDB *tile) {
  const char *szSql =
      "select pcd from TileTb where tileX = ? and tileY = ? and zone = ?";
  TileGrid grid = tile->GetTileGrid();

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, grid.x);
  query.bind(2, grid.y);
  query.bind(3, grid.zoom);
  if (query.executeStep()) {
    unsigned char *pcdBuf = (unsigned char *)query.getColumn(0).getBlob();
    int size = query.getColumn(0).getBytes();

    int offset = 0;

    PDB_HDR pdbHdr;
    memcpy(&pdbHdr, pcdBuf + offset, sizeof(PDB_HDR));
    offset += sizeof(PDB_HDR);

    //���汾
    if (pdbHdr.version != 2 && pdbHdr.version != 1) {
      return false;
    }

    //���crc32У��
    intptr_t ofs = (intptr_t) & (((PDB_HDR *)0)->crc32);
    *(unsigned int *)(pcdBuf + ofs) = 0;

    if (Utils::CalcCrc32(pcdBuf, size) != pdbHdr.crc32) {
      return false;
    }

    PDB_INFO infoHdr;
    memcpy(&infoHdr, pcdBuf + offset, sizeof(PDB_INFO));
    offset += sizeof(PDB_INFO);

    PDBPoint *new_buf = new PDBPoint[infoHdr.count];
    lzo_uint new_len = infoHdr.count * sizeof(PDBPoint);

    int r = lzo1x_decompress(pcdBuf + offset, size - offset,
                             (unsigned char *)new_buf, &new_len);
    if (r == LZO_E_OK) {
      tile->point_cloud_.reserve(infoHdr.count);

      for (int i = 0; i < infoHdr.count; i++) {
        tile->Add(new_buf[i].latlon, new_buf[i].altitude, new_buf[i].intensity,
                  new_buf[i].intensity);
      }

      delete[] new_buf;

      return true;
    } else {
      delete[] new_buf;
    }
  }

  return false;
}

void PDBManage::Close() {
  if (m_database != NULL) {
    m_database->close();
  }
}
}  // namespace geditor
