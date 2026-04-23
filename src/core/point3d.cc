
#include "core/point3d.h"

namespace geditor {

int GenerateTimeID() {
  int id = 0;
  time_t tt = time(NULL);
  struct tm *stm = localtime(&tt);
  int year = stm->tm_year - 120;  // 4 1900+120=2020 2020~2036
  int month = stm->tm_mon;        // 4
  int mday = stm->tm_mday;        // 5
  int mhour = stm->tm_hour;       // 5
  int mmin = stm->tm_min;         // 6
  int msec = stm->tm_sec;         // 6
  id = id | (year & 0x0f) << 26;
  id = id | (month & 0x0f) << 22;
  id = id | (mday & 0x1f) << 17;
  id = id | (mhour & 0x1f) << 12;
  id = id | (mmin & 0x3f) << 6;
  id = id | (msec & 0x3f) << 0;
  return id;
}

int GenerateFeatureID() {
  static int id = GenerateTimeID();
  return id++;
}

Point3d::Point3d() : x(0.0), y(0.0), z(0.0) {}

Point3d::Point3d(double xx, double yy, double zz, int idd) {
  x = xx;
  y = yy;
  z = zz;
  id = idd;
}

Point3d::~Point3d() {}

Point3d &Point3d::operator=(const Point3d &p1) {
  this->x = p1.x;
  this->y = p1.y;
  this->z = p1.z;
  this->id = p1.id;
  return *this;
}

Point3d &Point3d::operator+=(const Point3d &p1) {
  this->x += p1.x;
  this->y += p1.y;
  this->z += p1.z;
  return *this;
}

Point3d &Point3d::operator-=(const Point3d &p1) {
  this->x -= p1.x;
  this->y -= p1.y;
  this->z -= p1.z;

  return *this;
}

Point3d &Point3d::operator*=(double c) {
  this->x *= c;
  this->y *= c;
  this->z *= c;

  return *this;
}

Point3d &Point3d::operator/=(double c) {
  this->x /= c;
  this->y /= c;
  this->z /= c;

  return *this;
}

Point3d const operator-(const Point3d &p1, const Point3d &p2) {
  return Point3d(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

bool Point3d::operator==(const Point3d &right) const {
  if (x == right.x && y == right.y && z == right.z)
    return true;
  else
    return false;
}

bool Point3d::operator!=(const Point3d &right) const {
  if (x == right.x && y == right.y && z == right.z)
    return false;
  else
    return true;
}

}  // namespace geditor
