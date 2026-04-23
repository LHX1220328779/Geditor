
#include <sqlite3.h>
#include <sqlite_cpp.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <map>
#include <vector>

#include "algorithm/bound_box.h"
#include "pcd/db_read_write.h"

sqlite3 *pDB = NULL;

namespace geditor {

void FormatNowTime(char *szBuffer, int nbufSize) {
  time_t timer = time(NULL);
  struct tm *nowTime = localtime(&timer);

  sprintf(szBuffer, "%04d-%02d-%02d %02d:%02d:%02d", nowTime->tm_year + 1900,
          nowTime->tm_mon + 1, nowTime->tm_mday, nowTime->tm_hour,
          nowTime->tm_min, nowTime->tm_sec);
}
//---------------------------------------------------------------------------

bool DBReadWrite::WriteHistogramBlock(int x, int y, const void *pData,
                                      int nSize) {
  const char *strSql =
      "insert or replace into loction_histogram(X, Y, data_block, date) "
      "values(?,?,?,?)";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // pose
  sqlite3_bind_double(stmt, 1, x);
  sqlite3_bind_double(stmt, 2, y);
  sqlite3_bind_blob(stmt, 3, pData, nSize, NULL);

  // now time
  char szBuffer[128];
  FormatNowTime(szBuffer, 128);
  sqlite3_bind_text(stmt, 4, szBuffer, -1, NULL);

  // save record
  int nRes = sqlite3_step(stmt);
  if (nRes != SQLITE_DONE) {
    sqlite3_finalize(stmt);

    std::cout << "insert data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  } else {
    sqlite3_finalize(stmt);

    return true;
  }

  return true;
}

bool DBReadWrite::QueryScanContextUnique(
    std::vector<std::pair<int, int>> &arrI) {
  const char *strSql = "select X,Y from scan_context;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  do {
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
      int x = sqlite3_column_int(stmt, 0);
      int y = sqlite3_column_int(stmt, 1);

      arrI.push_back(std::pair<int, int>(x, y));
    } else if (nRes == SQLITE_DONE) {
      break;
    } else {
      sqlite3_finalize(stmt);

      std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
      return false;
    }

  } while (true);

  sqlite3_finalize(stmt);

  return true;
}

bool DBReadWrite::QueryHistogramUnique(std::vector<std::pair<int, int>> &arrI) {
  const char *strSql = "select X,Y from loction_histogram;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  do {
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
      int x = sqlite3_column_int(stmt, 0);
      int y = sqlite3_column_int(stmt, 1);

      arrI.push_back(std::pair<int, int>(x, y));
    } else if (nRes == SQLITE_DONE) {
      break;
    } else {
      sqlite3_finalize(stmt);

      std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
      return false;
    }

  } while (true);

  sqlite3_finalize(stmt);

  return true;
}

bool DBReadWrite::ReadTrackLine(UniqueType uniqueId, char *&pData,
                                int &length) {
  const char *strSql = "select track from track_line where uniqueId = ?;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, uniqueId);

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 0);
    const int nBlobSize = sqlite3_column_bytes(stmt, 0);
    if (pBlobAddr != NULL && nBlobSize > 0) {
      pData = new char[nBlobSize];
      memcpy(pData, pBlobAddr, nBlobSize);

      length = nBlobSize;
    }

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

bool DBReadWrite::ReadTrackImage(UniqueType uniqueId, char *&pData,
                                 int &length) {
  const char *strSql = "select image from track_image where uniqueId = ?;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, uniqueId);

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 0);
    const int nBlobSize = sqlite3_column_bytes(stmt, 0);
    if (pBlobAddr != NULL && nBlobSize > 0) {
      pData = new char[nBlobSize];
      memcpy(pData, pBlobAddr, nBlobSize);

      length = nBlobSize;
    }

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

bool DBReadWrite::ReadHistogramBlock(int x, int y, char *&pData, int &length) {
  const char *strSql =
      "select X,Y,data_block from loction_histogram where x = ? and y = ?;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, x);
  sqlite3_bind_int(stmt, 2, y);

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 2);
    const int nBlobSize = sqlite3_column_bytes(stmt, 2);
    if (pBlobAddr != NULL && nBlobSize > 0) {
      pData = new char[nBlobSize];
      memcpy(pData, pBlobAddr, nBlobSize);

      length = nBlobSize;
    }

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

bool DBReadWrite::WriteScanContextBlock(int x, int y, const void *pData,
                                        int nSize) {
  const char *strSql =
      "insert or replace into scan_context(X, Y, data_block, date) "
      "values(?,?,?,?)";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // pose
  sqlite3_bind_double(stmt, 1, x);
  sqlite3_bind_double(stmt, 2, y);
  sqlite3_bind_blob(stmt, 3, pData, nSize, NULL);

  // now time
  char szBuffer[128];
  FormatNowTime(szBuffer, 128);
  sqlite3_bind_text(stmt, 4, szBuffer, -1, NULL);

  // save record
  int nRes = sqlite3_step(stmt);
  if (nRes != SQLITE_DONE) {
    sqlite3_finalize(stmt);

    std::cout << "insert data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  } else {
    sqlite3_finalize(stmt);

    return true;
  }

  return true;
}

bool DBReadWrite::ReadScanContextBlock(int x, int y, char *&pData,
                                       int &length) {
  const char *strSql =
      "select X,Y,data_block from scan_context where x = ? and y = ?;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, x);
  sqlite3_bind_int(stmt, 2, y);

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 2);
    const int nBlobSize = sqlite3_column_bytes(stmt, 2);
    if (pBlobAddr != NULL && nBlobSize > 0) {
      pData = new char[nBlobSize];
      memcpy(pData, pBlobAddr, nBlobSize);

      length = nBlobSize;
    }

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

//-------------------------------------------------------------------------------

bool DBReadWrite::ReadMoveOriginPoint(V3d &orgin, int &zone, bool &bEast) {
  const char *strSql =
      "select X,Y,Z,zone,east from point_table where uniqueId = 1;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    orgin[0] = sqlite3_column_double(stmt, 0);
    orgin[1] = sqlite3_column_double(stmt, 1);
    orgin[2] = sqlite3_column_double(stmt, 2);

    zone = sqlite3_column_int(stmt, 3);
    bEast = sqlite3_column_int(stmt, 4);

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

bool DBReadWrite::WriteMoveOriginPoint(double x, double y, double z, int zone,
                                       bool bEast) {
  const char *strSql =
      "insert or replace into point_table(uniqueId, X, Y, Z, zone, east, date) "
      "values(?,?,?,?,?,?,?)";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, 1);

  // pose
  sqlite3_bind_double(stmt, 2, x);
  sqlite3_bind_double(stmt, 3, y);
  sqlite3_bind_double(stmt, 4, z);

  sqlite3_bind_double(stmt, 5, zone);
  sqlite3_bind_double(stmt, 6, bEast);

  // now time
  char szBuffer[128];
  FormatNowTime(szBuffer, 128);
  sqlite3_bind_text(stmt, 7, szBuffer, -1, NULL);

  // save record
  int nRes = sqlite3_step(stmt);
  if (nRes != SQLITE_DONE) {
    sqlite3_finalize(stmt);

    std::cout << "insert data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  } else {
    sqlite3_finalize(stmt);

    return true;
  }
}

bool DBReadWrite::QueryRawLaserData(const V3d &position, double radius,
                                    std::vector<UniqueType> &arrayId) {
  if (map_idx_pose_.size() <= 0) {
    if (!ReadPoseData()) {
      return false;
    }
  }

  std::map<int, DBPose>::iterator iter;
  for (iter = map_idx_pose_.begin(); iter != map_idx_pose_.end(); iter++) {
    V3d tmp = iter->second.pos - position;
    if (tmp.squaredNorm() < 4 * radius * radius) {
      arrayId.push_back(iter->first);
    }
  }

  return true;
}

bool DBReadWrite::QueryAllRawLaserUnique(std::vector<UniqueType> &arrIdxPose) {
  if (map_idx_pose_.size() <= 0) {
    if (!ReadPoseData()) {
      return false;
    }
  }

  std::map<int, DBPose>::iterator iter;
  for (iter = map_idx_pose_.begin(); iter != map_idx_pose_.end(); iter++) {
    arrIdxPose.push_back(iter->first);
  }

  return true;
}

bool DBReadWrite::QueryAllFunctionPoint(std::vector<UniqueType> &mArrIdx) {
  const char *strSql = "select uniqueId from function_point;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  do {
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
      UniqueType uniqueId = sqlite3_column_int(stmt, 0);

      mArrIdx.push_back(uniqueId);
    } else if (nRes == SQLITE_DONE) {
      break;
    } else {
      std::cout << "read sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
      break;
    }
  } while (true);

  sqlite3_finalize(stmt);

  return true;
}

bool DBReadWrite::QueryAllTrackLine(std::vector<int> &mTrackIdx) {
  const char *strSql = "select uniqueId from track_line;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  do {
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
      UniqueType uniqueId = sqlite3_column_int(stmt, 0);
      mTrackIdx.push_back(uniqueId);
    } else if (nRes == SQLITE_DONE) {
      break;
    } else {
      std::cout << "read sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
      break;
    }
  } while (true);

  sqlite3_finalize(stmt);

  return true;
}

//------------------------------------------

bool DBReadWrite::QueryBoundary(V3d &vMin, V3d &vMax) {
  if (map_idx_pose_.size() <= 0) {
    if (!ReadPoseData()) {
      return false;
    }
  }

  BoundBox3d boundBox;

  std::map<int, DBPose>::iterator iter;
  for (iter = map_idx_pose_.begin(); iter != map_idx_pose_.end(); iter++) {
    boundBox.ExpandBy(iter->second.pos);
  }
  boundBox.Extend(70.0);

  vMin = boundBox.v_min_;
  vMax = boundBox.v_max_;

  return true;
}

bool DBReadWrite::ReadPoseData() {
  const char *strSql =
      "select uniqueId, X,Y,Z,quatX,quatY,quatZ,quatW,pcd from "
      "raw_point_cloud;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  do {
    int nRes = sqlite3_step(stmt);
    if (nRes == SQLITE_ROW) {
      DBPose pose;

      UniqueType uniqueId = sqlite3_column_int(stmt, 0);

      pose.pos[0] = sqlite3_column_double(stmt, 1);
      pose.pos[1] = sqlite3_column_double(stmt, 2);
      pose.pos[2] = sqlite3_column_double(stmt, 3);

      pose.quat.x = sqlite3_column_double(stmt, 4);
      pose.quat.y = sqlite3_column_double(stmt, 5);
      pose.quat.z = sqlite3_column_double(stmt, 6);
      pose.quat.w = sqlite3_column_double(stmt, 7);

      map_idx_pose_.insert(std::make_pair(uniqueId, pose));
    } else if (nRes == SQLITE_DONE) {
      break;
    } else {
      std::cout << "read sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
      break;
    }
  } while (true);

  sqlite3_finalize(stmt);

  return true;
}

//--------------------------------------------------------------------------

bool DBReadWrite::ReadRawLaserData(UniqueType uniqueId, DBPose &pose,
                                   void *&pPackBuf, int &nPackLenth,
                                   int &version) {
  const char *strSql =
      "select X,Y,Z,quatX,quatY,quatZ,quatW,pcd from raw_point_cloud where "
      "uniqueId = ?;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, uniqueId);

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    pose.pos[0] = sqlite3_column_double(stmt, 0);
    pose.pos[1] = sqlite3_column_double(stmt, 1);
    pose.pos[2] = sqlite3_column_double(stmt, 2);

    pose.quat.x = sqlite3_column_double(stmt, 3);
    pose.quat.y = sqlite3_column_double(stmt, 4);
    pose.quat.z = sqlite3_column_double(stmt, 5);
    pose.quat.w = sqlite3_column_double(stmt, 6);

    const void *pBlobAddr = (void *)sqlite3_column_blob(stmt, 7);
    const int nBlobSize = sqlite3_column_bytes(stmt, 7);
    if (pBlobAddr != NULL && nBlobSize > 0) {
      char *pData = new char[nBlobSize];
      memcpy(pData, pBlobAddr, nBlobSize);

      if (!Unpack_PointCloud(pData, nBlobSize, version, pPackBuf, nPackLenth)) {
        pPackBuf = pData;
        nPackLenth = nBlobSize;
      }
    }

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

//---------------------------------------------------------------------------

bool DBReadWrite::WriteRawLaserData(UniqueType uniqueId, const DBPose &pose,
                                    const void *pData, int length,
                                    int version) {
  const char *strSql =
      "insert or replace into "
      "raw_point_cloud(uniqueId,X,Y,Z,quatX,quatY,quatZ,quatW,pcd,date) "
      "values(?,?,?,?,?,?,?,?,?,?)";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, uniqueId);

  // pose
  sqlite3_bind_double(stmt, 2, pose.pos[0]);
  sqlite3_bind_double(stmt, 3, pose.pos[1]);
  sqlite3_bind_double(stmt, 4, pose.pos[2]);
  sqlite3_bind_double(stmt, 5, pose.quat.x);
  sqlite3_bind_double(stmt, 6, pose.quat.y);
  sqlite3_bind_double(stmt, 7, pose.quat.z);
  sqlite3_bind_double(stmt, 8, pose.quat.w);

  // pcd data
  if (pData != NULL && length > 0) {
    sqlite3_bind_blob(stmt, 9, pData, length, NULL);
  }

  // now time
  char szBuffer[128];
  FormatNowTime(szBuffer, 128);
  sqlite3_bind_text(stmt, 10, szBuffer, -1, NULL);

  // save record
  int nRes = sqlite3_step(stmt);
  if (nRes != SQLITE_DONE) {
    sqlite3_finalize(stmt);

    std::cout << "insert data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  } else {
    sqlite3_finalize(stmt);

    return true;
  }
}

//-----------------------------------------------------------------------

void DBReadWrite::CloseDB() {
  if (pDB != NULL) {
    sqlite3_close(pDB);
    pDB = NULL;
  }
}

bool DBReadWrite::IsOpen() const { return (pDB != NULL); }

//------------------------------------------------------------------------
bool DBReadWrite::ReOpen() {
  if (!pDB) {
    int nRes = sqlite3_open(file_name_.c_str(), &pDB);
    if (nRes != SQLITE_OK) {
      std::cout << "Open database fail: " << sqlite3_errmsg(pDB);
      return false;
    }
  }
  return true;
}

bool DBReadWrite::OpenDB(const char *szfileName) {
  const char *sqltext =
      "create table if not exists raw_point_cloud( \
						  	uniqueId integer primary KEY, X numeric, Y numeric, Z numeric, \
							quatX numeric, quatY numeric, quatZ numeric, quatW numeric, \
							pcd blob not null,                                         \
							date text not null)";

  const char *sqloptimize =
      "create table if not exists optimize_point_cloud( \
							uniqueId integer primary KEY, X numeric, Y numeric, Z numeric, \
							quatX numeric, quatY numeric, quatZ numeric, quatW numeric, \
							date text not null)";

  const char *sqlpoint =
      "create table if not exists point_table( \
						     uniqueId integer primary KEY, X numeric, Y numeric, Z numeric, \
						     zone integer, east integer, date text not null)";

  const char *sqlhist =
      "create table if not exists loction_histogram( \
						    X integer not null, Y integer  not null, data_block blob,\
						    date text not null,primary key (X,Y))";

  file_name_ = szfileName;
  int nRes = sqlite3_open(szfileName, &pDB);
  if (nRes != SQLITE_OK) {
    std::cout << "Open database fail: " << sqlite3_errmsg(pDB);
    return false;
  }

  char *cErrMsg;

  int nRet1 = sqlite3_exec(pDB, sqltext, 0, 0, &cErrMsg);
  int nRet2 = sqlite3_exec(pDB, sqlpoint, 0, 0, &cErrMsg);
  int nRet3 = sqlite3_exec(pDB, sqloptimize, 0, 0, &cErrMsg);
  int nRet4 = sqlite3_exec(pDB, sqlhist, 0, 0, &cErrMsg);

  if (nRet1 != SQLITE_OK || nRet2 != SQLITE_OK || nRet3 != SQLITE_OK ||
      nRet4 != SQLITE_OK) {
    sqlite3_close(pDB);

    std::cout << "create table fail: " << cErrMsg << std::endl;
    return false;
  }

  return true;
}

bool DBReadWrite::WriteFunctionPoint(UniqueType uniqueId, int type, int number,
                                     const V3d &pos, double heading) {
  return false;
}

bool DBReadWrite::ReadFunctionPoint(UniqueType uniqueId, int &type, int &number,
                                    V3d &pos, double &heading) {
  const char *strSql =
      "select type, number, X, Y, Z, heading from function_point where "
      "uniqueId = ?;";

  sqlite3_stmt *stmt;
  int nStatus = sqlite3_prepare_v2(pDB, strSql, -1, &stmt, NULL);
  if (nStatus != SQLITE_OK) {
    std::cout << "prepare sqlite3 fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }

  // id
  sqlite3_bind_int(stmt, 1, uniqueId);

  // read record
  int nRes = sqlite3_step(stmt);
  if (nRes == SQLITE_ROW) {
    type = sqlite3_column_int(stmt, 0);
    number = sqlite3_column_int(stmt, 1);
    pos[0] = sqlite3_column_double(stmt, 2);
    pos[1] = sqlite3_column_double(stmt, 3);
    pos[2] = sqlite3_column_double(stmt, 4);

    heading = sqlite3_column_double(stmt, 5);

    sqlite3_finalize(stmt);

    return true;
  } else {
    sqlite3_finalize(stmt);

    std::cout << "read data fail: " << sqlite3_errmsg(pDB) << std::endl;
    return false;
  }
}

struct BPC_HDR {
  unsigned int magic;
  unsigned short version;
  unsigned short reserve;
  unsigned int crc32;
};

bool DBReadWrite::Pack_PointCloud(const void *szData, int length, int version,
                                  char *&pPackBuf, int &nPackLenth) {
  if (version > 0) {
    BPC_HDR hdr;

    hdr.magic = 0x435042;
    hdr.version = 1;
    hdr.reserve = 0;
    hdr.crc32 = 0;

    int nNewLen = sizeof(BPC_HDR) + length;
    char *pMemBuf = new char[nNewLen];

    if (pMemBuf != NULL) {
      memcpy(pMemBuf, &hdr, sizeof(BPC_HDR));
      memcpy(pMemBuf + sizeof(BPC_HDR), szData, length);

      pPackBuf = pMemBuf;
      nPackLenth = nNewLen;

      return true;
    }
  }

  pPackBuf = (char *)szData;
  nPackLenth = length;

  return false;
}

bool DBReadWrite::Unpack_PointCloud(char *szData, int length, int &version,
                                    void *&pPackBuf, int &nPackLenth) {
  if (length > sizeof(BPC_HDR)) {
    BPC_HDR hdr;
    memcpy(&hdr, szData, sizeof(BPC_HDR));

    if (hdr.magic == 0x435042 && hdr.version > 0) {
      int datLen = length - sizeof(BPC_HDR);
      char *dataPtr = szData + sizeof(BPC_HDR);

      memmove(szData, dataPtr, datLen);

      pPackBuf = szData;
      nPackLenth = datLen;
      version = hdr.version;

      return true;
    }
  }

  version = 0;
  pPackBuf = szData;
  nPackLenth = length;

  return true;
}
}  // namespace geditor
