#include "core/sign_board.h"
#include "core/road_area.h"

namespace geditor {

SignBoard::SignBoard()
    : MapFeature(MFT_SIGNBORAD),
      m_Changed(false),
      m_highlightPoint(-1),
      m_relationStopline(NULL) {
  m_property.areaType = 1;
  m_lineStyle = new PolygonSytle();
}

SignBoard::~SignBoard() {
  if (m_lineStyle) {
    delete m_lineStyle;
    m_lineStyle = NULL;
  }
}

bool SignBoard::IsChanged() {
  return (m_Changed || m_pGeometry->IsBoundDirty());
}

void SignBoard::SetChanged(bool change) { m_Changed = change; }

void SignBoard::SetSelectedState(bool selected) {
  m_Changed = true;
  MapFeature::SetSelectedState(selected);
  MapFeature::SetHighlightState(false);
}

void SignBoard::SetHighlightState(bool bhigh) {
  if (m_bSelected) {
    return;
  }
  m_Changed = true;
  MapFeature::SetHighlightState(bhigh);
}

void SignBoard::SetHighlightPoint(int index) {
  if (index != m_highlightPoint) {
    m_highlightPoint = index;
    m_Changed = true;
  }
}

void SignBoard::SetProperty(SignBoardProperty *pProperty) {
  m_property = *pProperty;
}

void SignBoard::SetSignboardType(int type) { m_property.areaType = type; }

//�Ƿ��������
bool SignBoard::IsRelationSegment(LaneSegment *segment) {
  for (int i = 0; i < m_relationSegment.size(); i++) {
    if (m_relationSegment[i] == segment) {
      return true;
    }
  }
  return false;
}

void SignBoard::GetRelationSegment(std::vector<LaneSegment *> &segment) {
  for (int i = 0; i < m_relationSegment.size(); i++) {
    segment.push_back(m_relationSegment[i]);
  }
}

void SignBoard::AddRelationSegment(LaneSegment *segment) {
  bool bNotFind = true;

  for (int i = 0; i < m_relationSegment.size(); i++) {
    if (m_relationSegment[i] == segment) {
      bNotFind = false;
      break;
    }
  }

  if (bNotFind) {
    m_relationSegment.push_back(segment);
  }
}

void SignBoard::SetRelationStopline(RoadArea *roadArea) {
  if (m_relationStopline != roadArea) {
    m_relationStopline = roadArea;

    m_property.stopline = roadArea->GetUniqueID();
  }
}

RoadArea *SignBoard::GetRelationStopline() const { return m_relationStopline; }

int SignBoard::GetHighlightPoint() { return m_highlightPoint; }

PolygonSytle *SignBoard::GetStyle() { return m_lineStyle; }

SignBoardProperty *SignBoard::GetProperty() { return &m_property; }

}  // namespace geditor
