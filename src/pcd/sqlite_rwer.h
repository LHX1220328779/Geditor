#pragma once

#include <glog/logging.h>
#include <cstdint>
#include <string>
#include <vector>

namespace geditor {

struct DBPoint {
  float x, y, z;
  uint8_t i;
};

struct DBPoints {
  uint64_t time;  // us 10-6
  double offset_x, offset_y, offset_z;
  std::vector<DBPoint> points;
};

struct DBTraPoint {
  uint64_t time;
  double x, y, z;
  int zone = 0;
};

constexpr size_t kDBPointLength = sizeof(DBPoint);
class SqliteRWer {
 public:
  SqliteRWer(const std::string file);
  ~SqliteRWer();
  bool IsOpen();
  std::vector<DBTraPoint> ReadTra();
  DBPoints ReadFrameByTime(uint64_t time);
  void CloseDB();

 private:
  int PreparePointsStmt();

 private:
  void *db_ = nullptr;
};

}  // namespace geditor
