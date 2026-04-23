
#pragma once

#include "core/geo_polygon.h"
#include "core/geo_polyline.h"
#include "core/map_feature.h"
#include "core/vector_style.h"
#include "map/projection_utm.h"

namespace geditor {

class RoadArea : public MapFeature {
 public:
  RoadArea()
      : MapFeature(MFT_ROAD_AREA),
        m_lineStyle(NULL),
        m_Changed(false),
        m_highlightPoint(-1) {
    m_property.areaType = 1;
    m_property.proParam = 2.7f;
    m_property.gpsParam = 0;

    m_lineStyle = new PolygonSytle();
    m_lineStyle->SetArrowColor(V3f(0.0, 1.0f, 1.0f));
    m_lineStyle->SetLineColor(V4f(1.0, 1.0f, 1.0f, 1.0f));
    m_lineStyle->SetVertexColor(V3f(1.0, 1.0f, 1.0f));
    m_lineStyle->SetKeyVertexColor(V3f(1.0, 1.0f, 1.0f));
  }

  virtual ~RoadArea() {
    if (m_lineStyle) {
      delete m_lineStyle;
      m_lineStyle = NULL;
    }
  }

  bool IsChanged() { return (m_Changed || m_pGeometry->IsBoundDirty()); }

  void SetChanged(bool change) { m_Changed = change; }

  void SetHighlightPoint(int index) {
    if (index != m_highlightPoint) {
      m_highlightPoint = index;
      m_Changed = true;
    }
  }

  void SetProperty(AreaProperty *pProperty) { m_property = *pProperty; }

  void SetStyle(PolygonSytle *lineStyle) {}

  void SetSelectedState(bool selected) {
    if (selected) {
      m_lineStyle->SetLineColor(V4f(1.0f, 0.0f, 0.0f, 1.0f));
    } else {
      m_lineStyle->SetLineColor(V4f(1.0, 1.0f, 1.0f, 1.0f));
    }
    m_Changed = true;
    MapFeature::SetSelectedState(selected);
    MapFeature::SetHighlightState(false);
  }

  void SetHighlightState(bool bhigh) {
    if (m_bSelected) {
      return;
    }
    if (bhigh) {
      m_lineStyle->SetLineColor(V4f(1.0f, 0.0f, 0.0f, 1.0f));
    } else {
      m_lineStyle->SetLineColor(V4f(1.0, 1.0f, 1.0f, 1.0f));
    }
    m_Changed = true;
    MapFeature::SetHighlightState(bhigh);
  }

  int GetHighlightPoint() { return m_highlightPoint; }

  PolygonSytle *GetStyle() { return m_lineStyle; }

  AreaProperty *GetProperty() { return &m_property; }

  void SetAreaType(int areaType) { m_property.areaType = areaType; }

  double GetArea() {
    int point_num = m_pGeometry->GetVertexCount();
    if (point_num < 3) {
      return 0.0;
    }

    GeoPolygon geoPolyon;
    for (int i = 0; i < point_num; ++i) {
      Point3d point1 = m_pGeometry->GetVertex(i);
      geoPolyon.AppendVertex(Point3d(point1.x, point1.y, 0.0));
    }

    double s = geoPolyon.GetArea();
    return s;
  }

 private:
  AreaProperty m_property;

 private:
  bool m_Changed;
  PolygonSytle *m_lineStyle = NULL;
  int m_highlightPoint;
};

}  // namespace geditor
