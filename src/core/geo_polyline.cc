
#include "core/geo_polyline.h"

#include <algorithm>

#include "algorithm/mc_math.h"
#include "map/map_define.h"

namespace geditor {

GeoPolyline::GeoPolyline() : Geometry(Geometry::GT_POLYLINE) {}

GeoPolyline::~GeoPolyline() {}

void GeoPolyline::MoveGeometry(double x, double y, double z) {
  int nSize = m_pointSet.size();
  if (nSize > 1) {
    for (int i = 0; i < nSize; i++) {
      Point3d end = m_pointSet[i];

      m_pointSet[i].x = end.x + x;
      m_pointSet[i].y = end.y + y;
      m_pointSet[i].z = end.z + z;
    }
  }
  m_bDirty = true;
}

const void *GeoPolyline::GetDataPtr() const { return &(m_pointSet[0]); }

const int GeoPolyline::GetDataSize() const {
  size_t count = m_pointSet.size();
  return count * sizeof(Point3d);
}

void GeoPolyline::Hermite(std::vector<Point3d> &items) { items = m_pointSet; }

}  // namespace geditor
