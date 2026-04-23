#include "sqlite_rwer.h"

#include <sqlite3.h>

#define LWARN LOG(WARNING)

namespace geditor {

SqliteRWer::SqliteRWer(const std::string file) {
  sqlite3 *sqlitedb = nullptr;
  int ret = sqlite3_open(file.c_str(), &sqlitedb);
  db_ = sqlitedb;
  if (ret != SQLITE_OK) {
    LWARN << "Open database fail: " << sqlite3_errmsg((sqlite3 *)db_);
    db_ = nullptr;
  } else {
    // SqliteRWer::PreparePointsStmt();
  }

  sqlite3_exec((sqlite3 *)db_, "PRAGMA synchronous = OFF; ", 0, 0, 0);
  sqlite3_exec((sqlite3 *)db_, "PRAGMA page_size =32768; ", 0, 0, 0);  // 32k
  // 10*32M
  sqlite3_exec((sqlite3 *)db_, "PRAGMA cache_size =2000; ", 0, 0, 0);
  sqlite3_exec((sqlite3 *)db_, "PRAGMA count_changes=OFF", 0, 0, 0);
  sqlite3_exec((sqlite3 *)db_, "PRAGMA journal_mode=OFF", 0, 0, 0);
  sqlite3_exec((sqlite3 *)db_, "PRAGMA temp_store=MEMORY", 0, 0, 0);
}

SqliteRWer::~SqliteRWer() { sqlite3_close((sqlite3 *)db_); }

int SqliteRWer::PreparePointsStmt() {
  const char *sqltext =
      "create table if not exists point_cloud(unique_id integer primary key, \
           offset_x double, offset_y double, offset_z double, points blob)";
  char *errmsg;
  int ret = sqlite3_exec((sqlite3 *)db_, sqltext, 0, 0, &errmsg);
  if (ret != SQLITE_OK) {
    CloseDB();
    LWARN << "create table fail: " << errmsg;
    return -1;
  }
  const char *sqltext1 =
      "create table if not exists tra( \
  unique_id integer primary key, tra_x double, tra_y double, tra_z double)";
  ret = sqlite3_exec((sqlite3 *)db_, sqltext1, 0, 0, &errmsg);
  if (ret != SQLITE_OK) {
    CloseDB();
    LWARN << "create table fail: " << errmsg;
    return -1;
  }
  return 0;
}

void SqliteRWer::CloseDB() {
  sqlite3_close((sqlite3 *)db_);
  db_ = nullptr;
}

bool SqliteRWer::IsOpen() { return db_; }

std::vector<DBTraPoint> SqliteRWer::ReadTra() {
  std::vector<DBTraPoint> tra;
  DBTraPoint tra_point;
  if (!db_) return tra;
  const char *sql_read_tra = "select * from tra;";
  sqlite3_stmt *stmt = nullptr;
  int ret =
      sqlite3_prepare_v2((sqlite3 *)db_, sql_read_tra, -1, &stmt, nullptr);
  if (ret != SQLITE_OK) {
    LWARN << "prepare sqlite3 fail: " << sqlite3_errmsg((sqlite3 *)db_);
    return tra;
  }
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    tra_point.time = sqlite3_column_int64(stmt, 0);
    tra_point.x = sqlite3_column_double(stmt, 1);
    tra_point.y = sqlite3_column_double(stmt, 2);
    tra_point.z = sqlite3_column_double(stmt, 3);
    tra.push_back(tra_point);
  }
  sqlite3_finalize(stmt);
  return tra;
}

DBPoints SqliteRWer::ReadFrameByTime(uint64_t time) {
  DBPoints ps;
  ps.time = -1;
  if (!db_) return ps;
  const char *sql_select = "select * from point_cloud where unique_id = ?;";
  sqlite3_stmt *stmt = nullptr;
  int ret = sqlite3_prepare_v2((sqlite3 *)db_, sql_select, -1, &stmt, nullptr);
  if (ret != SQLITE_OK) {
    LWARN << "prepare stmt fail: " << sqlite3_errmsg((sqlite3 *)db_);
    return ps;
  }
  sqlite3_bind_int64(stmt, 1, time);
  ret = sqlite3_step(stmt);
  if (ret == SQLITE_ROW) {
    ps.time = sqlite3_column_int64(stmt, 0);
    ps.offset_x = sqlite3_column_double(stmt, 1);
    ps.offset_y = sqlite3_column_double(stmt, 2);
    ps.offset_z = sqlite3_column_double(stmt, 3);
    const void *addr = (void *)sqlite3_column_blob(stmt, 4);
    int size = sqlite3_column_bytes(stmt, 4);
    int m = size % kDBPointLength;
    if (addr != nullptr && size > 0 && m == 0) {
      int n = size / kDBPointLength;
      ps.points.resize(n);
      memcpy(ps.points.data(), addr, size);
    }
  } else {
    LWARN << "read data fail: " << sqlite3_errmsg((sqlite3 *)db_);
  }
  sqlite3_finalize(stmt);
  return ps;
}

}  // namespace geditor
