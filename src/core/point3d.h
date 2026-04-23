
#pragma once

#include <time.h>

#include <cmath>
#include <string>
#include <vector>

namespace geditor {

int GenerateTimeID();

int GenerateFeatureID();

class Point3d {
 public:
  //! \brief
  double x = 0;
  //! \brief
  double y = 0;
  //! \brief
  double z = 0;
  int GetId() {
    if (id == 0) {
      id = GenerateFeatureID();
    }
    return id;
  }
  int Id() const { return id; }

 private:
  int id = 0;

 public:
  //! \brief
  Point3d();

  //! \brief
  ~Point3d();

  //! \brief
  Point3d(double xx, double yy, double zz, int idd = 0);
  static Point3d Point3dId(double x, double y, double z) {
    Point3d p(x, y, z);
    p.GetId();
    return p;
  }
  static double Distance(const Point3d &p1, const Point3d &p2) {
    double dlength2 = (p1.x - p2.x) * (p1.x - p2.x) +
                      (p1.y - p2.y) * (p1.y - p2.y) +
                      (p1.z - p2.z) * (p1.z - p2.z);
    double disance = sqrt(dlength2);

    return disance;
  }

  static Point3d Normalize(const Point3d &p1, const Point3d &p2) {
    Point3d dp = p2 - p1;
    dp /= Distance(p1, p2);
    return dp;
  }

  static Point3d MiddlePoint(const Point3d &p1, const Point3d &p2) {
    Point3d pnt;
    pnt.x = (p1.x + p2.x) * 0.5;
    pnt.y = (p1.y + p2.y) * 0.5;
    pnt.z = (p1.z + p2.z) * 0.5;
    return pnt;
  }

 public:
  Point3d &operator=(const Point3d &s);

  Point3d &operator+=(const Point3d &p);

  Point3d &operator-=(const Point3d &p);

  Point3d &operator*=(double c);

  Point3d &operator/=(double c);

  friend const Point3d operator-(const Point3d &p1, const Point3d &p2);

  bool operator==(const Point3d &) const;

  bool operator!=(const Point3d &) const;
};

struct TrackPoint {
  Point3d pnt;
  double ndt = 0;
  int img = 0;
  int idx = -1;
};

struct CurbsTrack {
  std::vector<TrackPoint> trackSet;
  // Original file path imported by the editor.
  std::string sourcePath;
  // File basename without extension, usually the mine segment code.
  std::string sourceCode;
  // Stable index carried in the first column of the imported txt/csv file.
  int sourceIndex = 0;
};

}  // namespace geditor
