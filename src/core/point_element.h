
#pragma once

#include "core/geo_polyline.h"
#include "core/map_feature.h"
#include "core/vector_style.h"

namespace geditor {

class LaneSegment;

class RoadArea;

//点元素
class PointElement : public MapFeature {
 public:
  PointElement();

  virtual ~PointElement();

 public:
  bool IsChanged();

  void SetChanged(bool change);

  void SetHighlightPoint(int index);

  int GetHighlightPoint();

  V4f GetColor() const { return color_; };

  void SetSelectedState(bool selected);

 private:
  bool changed_;
  int highlight_point_;
  Point3d point_;
  V4f color_;
};
}  // namespace geditor
