
#pragma once

#include "core/geo_polyline.h"
#include "core/map_feature.h"
#include "core/vector_style.h"

#include <map>

namespace geditor {

class BoundSegment : public MapFeature {
 public:
  BoundSegment();

  virtual ~BoundSegment();

  virtual void SetSelectedState(bool selected);

  virtual void SetHighlightState(bool high);

 public:
  void SetHighlightPoint(int index);

  bool IsChanged();

  void SetChanged(bool change);

  void SetStyle(PolyLineSytle *lineStyle);

  PolyLineSytle *GetStyle();

  BoundaryProperty *GetProperty();

  void SetProperty(BoundaryProperty *pProperty);

  int GetHighlightPoint();

  void AddImageIndex(int index, int imageIdx);

  int GetImageIndex(int index);

 private:
  BoundaryProperty m_property;
  std::map<int, int> m_imageIndex;

 private:
  PolyLineSytle *m_lineStyle = NULL;
  bool m_Changed;
  int m_highlightPoint;
};

}  // namespace geditor
