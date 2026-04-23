
#pragma once

#include "core/geometry.h"

namespace geditor {

class GeoPolygon : public Geometry {
 public:
  GeoPolygon();

  virtual ~GeoPolygon();

 protected:
  GeoPolygon(GeometryType type);

 public:

  void Clear();

  int OnPoint(const Point3d &Q, double tolerance);

  double Length() const;

  double GetArea() const;

  double GetNeartPoint(const Point3d &point) const override;

  const void *GetDataPtr() const;

  const int GetDataSize() const;

  virtual bool IsVaild() const {
    if (m_pointSet.size() > 2) {
      return true;
    } else {
      return false;
    }
  }
};

}  // namespace geditor
