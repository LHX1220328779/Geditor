#include "core/point_element.h"
#include "core/road_area.h"

namespace geditor {
PointElement::PointElement()
    : MapFeature(MFT_POINT_ELEMENT), changed_(false), highlight_point_(-1) {
  color_ = V4f(0.0, 0.0, 1.0, 1.0);
}

PointElement::~PointElement() {}

bool PointElement::IsChanged() {
  return (changed_ || m_pGeometry->IsBoundDirty());
}

void PointElement::SetChanged(bool change) { changed_ = change; }

void PointElement::SetHighlightPoint(int index) {
  if (index != highlight_point_) {
    highlight_point_ = index;
    changed_ = true;
  }
}

void PointElement::SetSelectedState(bool selected) {
  if (selected) {
    color_ = V4f(1.0, 0.0, 0.0, 1.0);
  } else {
    color_ = V4f(0.0, 0.0, 1.0, 1.0);
  }
  changed_ = true;

  MapFeature::SetSelectedState(selected);
}

int PointElement::GetHighlightPoint() { return highlight_point_; }

}  // namespace geditor