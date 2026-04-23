
#include "core/bound_segment.h"

namespace geditor {

BoundSegment::BoundSegment()
    : MapFeature(MFT_BOUNDARY), m_Changed(true), m_highlightPoint(-1) {
  m_property.turnType = 1;
  m_property.length = 2.0f;
  m_property.speed = 2.0f;
  m_lineStyle = new PolyLineSytle();
  m_lineStyle->SetArrowColor(V3f(1.0, 1.0f, 0.0f));
  m_lineStyle->SetLineColor(V4f(1.0, 1.0f, 1.0f, 1.0f));
  m_lineStyle->SetVertexColor(V3f(1.0, 1.0f, 0.0f));
  m_lineStyle->SetKeyVertexColor(V3f(1.0, 1.0f, 1.0f));
}

BoundSegment::~BoundSegment() {
  if (m_lineStyle) {
    delete m_lineStyle;
    m_lineStyle = NULL;
  }
}

//����ѡ��״̬
void BoundSegment::SetSelectedState(bool selected) {
  if (selected) {
    m_lineStyle->SetLineColor(V4f(0.0f, 1.0f, 0.0f, 1.0f));
  } else {
    m_lineStyle->SetLineColor(V4f(1.0, 1.0f, 1.0f, 1.0f));
  }
  m_Changed = true;

  MapFeature::SetSelectedState(selected);
  MapFeature::SetHighlightState(false);
}

void BoundSegment::SetHighlightState(bool high) {
  if (m_bSelected) {
    return;
  }

  if (high) {
    m_lineStyle->SetLineColor(V4f(0.0f, 1.0f, 0.0f, 1.0f));
  } else {
    m_lineStyle->SetLineColor(V4f(1.0, 1.0f, 1.0f, 1.0f));
  }
  m_Changed = true;

  MapFeature::SetHighlightState(high);
}

void BoundSegment::SetHighlightPoint(int index) {
  if (index != m_highlightPoint) {
    m_highlightPoint = index;
    m_Changed = true;
  }
}

bool BoundSegment::IsChanged() {
  return (m_Changed || m_pGeometry->IsBoundDirty());
}

void BoundSegment::SetChanged(bool change) {
  m_Changed = change;
  m_pGeometry->OnChange();
}

void BoundSegment::SetStyle(PolyLineSytle *lineStyle) {
  // if (m_lineStyle != lineStyle)
  //{
  //	m_lineStyle = lineStyle;
  //	m_Changed = true;
  //}
}

PolyLineSytle *BoundSegment::GetStyle() { return m_lineStyle; }

BoundaryProperty *BoundSegment::GetProperty() {
  if (m_pGeometry) {
    GeoPolyline *polyline = (GeoPolyline *)m_pGeometry;

    m_property.length = (float)polyline->Length();
  }
  return &m_property;
}

void BoundSegment::SetProperty(BoundaryProperty *pProperty) {
  m_property = *pProperty;
}

int BoundSegment::GetHighlightPoint() { return m_highlightPoint; }

void BoundSegment::AddImageIndex(int index, int imageIdx) {
  if (index >= 0 && imageIdx != -1) {
    m_imageIndex.insert(std::pair<int, int>(index, imageIdx));
  }
}

int BoundSegment::GetImageIndex(int index) {
  std::map<int, int>::iterator iterRet = m_imageIndex.find(index);
  if (iterRet != m_imageIndex.end()) {
    return iterRet->second;
  }
  return -1;
}

}  // namespace geditor
