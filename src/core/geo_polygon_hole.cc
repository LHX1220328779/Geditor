
#include "core/geo_polygon_hole.h"

namespace geditor {

GeoPolygonHole::GeoPolygonHole() : GeoPolygon(GT_POLYGON_HOLE) {}

GeoPolygonHole::~GeoPolygonHole() {}

GeoPolygon *GeoPolygonHole::GetHole(int index) const {
  if (index < m_hole.size() && index >= 0) {
    return m_hole[index];
  } else {
    return NULL;
  }
}

int GeoPolygonHole::GetHoleCount() const { return m_hole.size(); }

void GeoPolygonHole::AppendHole(GeoPolygon *polygon) {
  m_hole.push_back(polygon);
}

void GeoPolygonHole::RemoveVertex(int index) {
  int iCount = m_pointSet.size();

  if (index < iCount) {
    if (iCount > 3) {
      GeoPolygon::RemoveVertex(index);
    }
  } else {
    index -= iCount;

    for (int x = 0; x < m_hole.size(); x++) {
      int iCount = m_hole[x]->GetVertexCount();
      if (index < iCount) {
        if (iCount > 3) {
          m_hole[x]->RemoveVertex(index);
        } else {
          m_hole.erase(m_hole.begin() + x);
        }

        m_bDirty = true;
        OnChange();

        break;
      }
      index -= iCount;
    }
  }
}

void GeoPolygonHole::MoveGeometry(const Point3d &vDir) {
  for (int x = 0; x < m_hole.size(); x++) {
    m_hole[x]->MoveGeometry(vDir);
  }
  GeoPolygon::MoveGeometry(vDir);
}

void GeoPolygonHole::MoveVertex(const Point3d &point, int index) {
  int iCount = m_pointSet.size();

  if (index < iCount) {
    GeoPolygon::MoveVertex(point, index);
  } else {
    index -= iCount;

    for (int x = 0; x < m_hole.size(); x++) {
      int iCount = m_hole[x]->GetVertexCount();
      if (index < iCount) {
        m_hole[x]->MoveVertex(point, index);
        m_bDirty = true;

        OnChange();

        break;
      }
      index -= iCount;
    }
  }
}

Point3d GeoPolygonHole::GetVertex(int index) const {
  int iCount = m_pointSet.size();
  if (index < m_pointSet.size()) {
    return m_pointSet[index];
  } else {
    index -= iCount;

    for (int x = 0; x < m_hole.size(); x++) {
      int iCount = m_hole[x]->GetVertexCount();
      if (index < iCount) {
        Point3d pt = m_hole[x]->GetVertex(index);
        return pt;
      }
      index -= iCount;
    }
  }

  return Point3d();
}

int GeoPolygonHole::IsPointInEdge(const Point3d &point, Point3d &outPnt,
                                  double tolerance, double &fal) {
  {
    double length;

    int index = GeoPolygon::IsPointInEdge(point, outPnt, tolerance, length);
    if (index > 0 && length < tolerance) {
      fal = length;
      return index;
    }
  }

  for (int i = 0; i < m_hole.size(); i++) {
    double length;
    m_hole[i]->CalculateBoundBox();
    int index = m_hole[i]->IsPointInEdge(point, outPnt, tolerance, length);
    if (index > 0 && length < tolerance) {
      fal = length;
      return index;
    }
  }
  return 0;
}

}  // namespace geditor