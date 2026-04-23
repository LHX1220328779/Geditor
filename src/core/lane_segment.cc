
#include "core/lane_segment.h"

#include "core/map_feature.h"
#include "core/road_segment.h"
#include "core/sign_board.h"

namespace geditor {

LaneSegment::LaneSegment()
    : MapFeature(MFT_LANE_SEG),
      m_linkRoad(NULL),
      m_lineStyle(NULL),
      m_Changed(false),
      m_highlightPoint(-1) {
  m_property.turnType = 1;
  m_property.length = 0.0f;
  m_property.speed = 30.0f;
  m_property.lanePitch = 0.0f;

  m_parallel.leftSegment = 0;
  m_parallel.rightSegment = 0;
  m_parallel.leftReverseSegment = 0;
  m_parallel.rightReverseSegment = 0;

  m_lineStyle = new PolyLineSytle();

  m_lineStyle->SetBackgroundColor(V4f(0.0f, 1.0f, 0.0f, 0.5f));
  m_lineStyle->SetLineColor(V4f(1.0f, 1.0f, 1.0f, 1.0f));
  m_lineStyle->SetArrowColor(V3f(1.0, 0.0f, 1.0f));
  m_lineStyle->SetVertexColor(V3f(1.0, 0.0f, 1.0f));
  m_lineStyle->SetKeyVertexColor(V3f(1.0, 1.0f, 1.0f));
}

LaneSegment::~LaneSegment() {
  if (m_lineStyle) {
    delete m_lineStyle;
    m_lineStyle = nullptr;
  }
}

void LaneSegment::SetSelectedState(bool selected) {
  if (selected) {
    m_lineStyle->SetBackgroundColor(V4f(0.0f, 1.0f, 0.0f, 0.5f));

    m_lineStyle->SetLineColor(V4f(0.0f, 1.0f, 0.0f, 1.0f));
    m_lineStyle->SetArrowColor(V3f(0.0, 1.0f, 0.0f));
    m_lineStyle->SetVertexColor(V3f(0.0, 1.0f, 0.0f));
  } else {
    m_lineStyle->SetBackgroundColor(V4f(0.0f, 1.0f, 0.0f, 0.5f));

    m_lineStyle->SetLineColor(V4f(1.0f, 1.0f, 1.0f, 1.0f));
    m_lineStyle->SetArrowColor(V3f(1.0, 0.0f, 1.0f));
    m_lineStyle->SetVertexColor(V3f(1.0, 0.0f, 1.0f));
  }
  m_Changed = true;

  MapFeature::SetSelectedState(selected);
  MapFeature::SetHighlightState(false);
}

void LaneSegment::SetHighlightState(bool high) {
  if (m_bSelected) {
    return;
  }

  if (high) {
    m_lineStyle->SetLineColor(V4f(0.0f, 1.0f, 0.0f, 1.0f));

  } else {
    m_lineStyle->SetLineColor(V4f(1.0f, 1.0f, 1.0f, 1.0f));
  }
  m_Changed = true;

  MapFeature::SetHighlightState(high);
}

RoadSegment *LaneSegment::GetParentLink() const { return m_linkRoad; }

void LaneSegment::SetParentLink(RoadSegment *pLink) { m_linkRoad = pLink; }

void LaneSegment::SetHighlightPoint(int index) {
  if (index == -1) {
    bool b = true;
  }

  if (index != m_highlightPoint) {
    m_highlightPoint = index;
    m_Changed = true;
  }
}

bool LaneSegment::IsChanged() {
  return (m_Changed || m_pGeometry->IsBoundDirty());
}

void LaneSegment::SetChanged(bool change) {
  m_Changed = change;
  m_pGeometry->OnChange();
}

PolyLineSytle *LaneSegment::GetStyle() { return m_lineStyle; }

SegmentProperty *LaneSegment::GetProperty() {
  if (m_pGeometry) {
    m_property.length = (float)m_pGeometry->Length();
  }
  return &m_property;
}

void LaneSegment::SetProperty(SegmentProperty *pProperty) {
  m_property = *pProperty;
}

ParallelSegment *LaneSegment::GetParallelSegment() { return &m_parallel; }

// void LaneSegment::SetLeftSegment(LaneSegment *pSegment) {
//   m_leftSegment = pSegment;
//   m_rightSegment = pSegment;
// }

void LaneSegment::SetLeftSegment(MapFeature *l) {
  if (l)
    m_parallel.leftSegment = l->GetUniqueID();
  else
    m_parallel.leftSegment = 0;
  m_leftLane = l;
}
void LaneSegment::SetRightSegment(MapFeature *r) {
  if (r)
    m_parallel.rightSegment = r->GetUniqueID();
  else
    m_parallel.rightSegment = 0;
  m_rightLane = r;
}
void LaneSegment::SetLeftReverseSegment(MapFeature *l) {
  if (l)
    m_parallel.leftReverseSegment = l->GetUniqueID();
  else
    m_parallel.leftReverseSegment = 0;
  m_leftLane_r = l;
}
void LaneSegment::SetRightReverseSegment(MapFeature *r) {
  if (r)
    m_parallel.rightReverseSegment = r->GetUniqueID();
  else
    m_parallel.rightReverseSegment = 0;
  m_rightLane_r = r;
}

void LaneSegment::GetSuccessorSegment(std::vector<int> &segArray) {
  for (int i = 0; i < m_successor.size(); i++) {
    segArray.push_back(m_successor[i]->GetUniqueID());
  }
}

void LaneSegment::GetPredecessorSegment(std::vector<int> &segArray) {
  for (int i = 0; i < m_predecessor.size(); i++) {
    segArray.push_back(m_predecessor[i]->GetUniqueID());
  }
}

void LaneSegment::GetSuccessorSegment(std::vector<LaneSegment *> &segArray) {
  for (int i = 0; i < m_successor.size(); i++) {
    segArray.push_back(m_successor[i]);
  }
}

void LaneSegment::GetPredecessorSegment(std::vector<LaneSegment *> &segArray) {
  for (int i = 0; i < m_predecessor.size(); i++) {
    segArray.push_back(m_predecessor[i]);
  }
}

void LaneSegment::SetSuccessorSegment(
    const std::vector<LaneSegment *> &segArray) {
  m_successor = segArray;
}

void LaneSegment::SetPredecessorSegment(
    const std::vector<LaneSegment *> &segArray) {
  m_predecessor = segArray;
}

void LaneSegment::GetAttachObject(std::vector<MapFeature *> &attachOjbects) {
  for (int i = 0; i < m_attachObject.size(); i++) {
    attachOjbects.push_back(m_attachObject[i]);
  }
}

void LaneSegment::AddSuccessorSegment(LaneSegment *segment) {
  bool bNotFind = true;
  if (!segment) return;
  for (int i = 0; i < m_successor.size(); i++) {
    if (m_successor[i] == segment) {
      bNotFind = false;
      break;
    }
  }

  if (bNotFind) {
    m_successor.push_back(segment);
  }
}

void LaneSegment::AddPredecessorSegment(LaneSegment *segment) {
  bool bNotFind = true;
  if (!segment) return;
  for (int i = 0; i < m_predecessor.size(); i++) {
    if (m_predecessor[i] == segment) {
      bNotFind = false;
      break;
    }
  }

  if (bNotFind) {
    m_predecessor.push_back(segment);
  }
}

void LaneSegment::SetSuccessorFeature(MapFeature *segment) {
  m_successorFeature = segment;
}

void LaneSegment::SetPredecessorFeature(MapFeature *segment) {
  m_predecessorFeature = segment;
}

MapFeature *LaneSegment::GetSuccessorFeature() { return m_successorFeature; }

MapFeature *LaneSegment::GetPredecessorFeature() {
  return m_predecessorFeature;
}

void LaneSegment::AddAttachObject(MapFeature *pOjbect) {
  bool bNotFind = true;

  for (int i = 0; i < m_attachObject.size(); i++) {
    if (m_attachObject[i] == pOjbect) {
      bNotFind = false;
      break;
    }
  }

  if (bNotFind) {
    m_attachObject.push_back(pOjbect);
  } else {
    if (pOjbect->GetType() == MapFeature::MFT_SIGNBORAD) {
      SignBoard *pSignBoard = (SignBoard *)pOjbect;
      SetSignBoardObject(pSignBoard);
    }
  }
}

void LaneSegment::SetSignBoardObject(SignBoard *pOjbect) {
  for (int i = 0; i < m_attachObject.size(); i++) {
    SignBoard *pSignBoard = (SignBoard *)m_attachObject[i];

    if (pSignBoard->GetProperty()->areaType ==
        pOjbect->GetProperty()->areaType) {
      m_attachObject[i] = pOjbect;
      break;
    }
  }
}

bool LaneSegment::IsSuccessorSegment(LaneSegment *segment) {
  for (int i = 0; i < m_successor.size(); i++) {
    if (m_successor[i] == segment) {
      return true;
    }
  }

  return false;
}

bool LaneSegment::IsPredecessorSegment(LaneSegment *segment) {
  for (int i = 0; i < m_predecessor.size(); i++) {
    if (m_predecessor[i] == segment) {
      return true;
    }
  }

  return false;
}

bool LaneSegment::IsAttachObject(MapFeature *pOjbect) {
  for (int i = 0; i < m_attachObject.size(); i++) {
    if (m_attachObject[i] == pOjbect) {
      return true;
    }
  }

  return false;
}

void LaneSegment::ClearSuccessorSegment() { m_successor.clear(); }

void LaneSegment::ClearPredecessorSegment() { m_predecessor.clear(); }

void LaneSegment::ClearAttachObject() { m_attachObject.clear(); }

int LaneSegment::GetHighlightPoint() { return m_highlightPoint; }

int LaneSegment::InLaneNet() {
  int iln = 0;
  for (auto &s : m_successor) {
    if (s && !s->deleted()) iln++;
  }
  return iln;
}

void LaneSegment::OverlapLaneVertex(int type) {
  if (type == 0) {
    if (m_predecessor.size() == 1) {
      *(GetGeometry()->GetStartVertexPtr()) =
          m_predecessor[0]->GetGeometry()->GetEndVertex();
      SetChanged(true);
      if (GetLeftBoundary() && m_predecessor[0]->GetLeftBoundary()) {
        *(GetLeftBoundary()->GetGeometry()->GetStartVertexPtr()) =
            m_predecessor[0]->GetLeftBoundary()->GetGeometry()->GetEndVertex();
        m_predecessor[0]->GetLeftBoundary()->SetChanged(true);
      }
      if (GetRightBoundary() && m_predecessor[0]->GetRightBoundary()) {
        *(GetRightBoundary()->GetGeometry()->GetStartVertexPtr()) =
            m_predecessor[0]->GetRightBoundary()->GetGeometry()->GetEndVertex();
        m_predecessor[0]->GetRightBoundary()->SetChanged(true);
      }
    }
  } else if (type == 1) {
    if (m_predecessor.size() > 1) {
      for (auto &pd : m_predecessor) {
        *(pd->GetGeometry()->GetEndVertexPtr()) =
            GetGeometry()->GetStartVertex();
        pd->SetChanged(true);
      }
      if (GetLeftBoundary()) {
        for (auto &pd : m_predecessor) {
          if (pd->GetLeftBoundary()) {
            *(pd->GetLeftBoundary()->GetGeometry()->GetEndVertexPtr()) =
                GetLeftBoundary()->GetGeometry()->GetStartVertex();
            pd->GetLeftBoundary()->SetChanged(true);
          }
        }
      }
      if (GetRightBoundary()) {
        for (auto &pd : m_predecessor) {
          if (pd->GetRightBoundary()) {
            *(pd->GetRightBoundary()->GetGeometry()->GetEndVertexPtr()) =
                GetRightBoundary()->GetGeometry()->GetStartVertex();
            pd->GetRightBoundary()->SetChanged(true);
          }
        }
      }
    }
    if (m_successor.size() > 1) {
      for (auto &pd : m_successor) {
        *(pd->GetGeometry()->GetStartVertexPtr()) =
            GetGeometry()->GetEndVertex();
        pd->SetChanged(true);
      }
      if (GetLeftBoundary()) {
        for (auto &pd : m_successor) {
          if (pd->GetLeftBoundary()) {
            *(pd->GetLeftBoundary()->GetGeometry()->GetStartVertexPtr()) =
                GetLeftBoundary()->GetGeometry()->GetEndVertex();
            pd->GetLeftBoundary()->SetChanged(true);
          }
        }
      }
      if (GetRightBoundary()) {
        for (auto &pd : m_successor) {
          if (pd->GetRightBoundary()) {
            *(pd->GetRightBoundary()->GetGeometry()->GetStartVertexPtr()) =
                GetRightBoundary()->GetGeometry()->GetEndVertex();
            pd->GetRightBoundary()->SetChanged(true);
          }
        }
      }
    }
  }
}

void LaneSegment::CheckAttachObject() {
  for (auto iter = m_attachObject.begin(); iter != m_attachObject.end();) {
    if ((*iter)->deleted())
      m_attachObject.erase(iter);
    else
      iter++;
  }
}
void LaneSegment::CheckCessor() {
  for (auto iter = m_successor.begin(); iter != m_successor.end();) {
    if ((*iter)->deleted())
      m_successor.erase(iter);
    else
      iter++;
  }

  for (auto iter = m_predecessor.begin(); iter != m_predecessor.end();) {
    if ((*iter)->deleted())
      m_predecessor.erase(iter);
    else
      iter++;
  }
}
void LaneSegment::CheckBoundary() {
  if (m_leftBoundary && m_leftBoundary->deleted()) SetLeftBoundary(nullptr);
  if (m_rightBoundary && m_rightBoundary->deleted()) SetRightBoundary(nullptr);
}

void LaneSegment::CheckParallelSegment() {
  if (m_leftLane && m_leftLane->deleted()) m_parallel.leftSegment = 0;
  if (m_rightLane && m_rightLane->deleted()) m_parallel.rightSegment = 0;
  if (m_leftLane_r && m_leftLane_r->deleted())
    m_parallel.leftReverseSegment = 0;
  if (m_rightLane_r && m_rightLane_r->deleted())
    m_parallel.rightReverseSegment = 0;
}

void LaneSegment::CheckDeletedRealtion() {
  CheckAttachObject();
  CheckBoundary();
  CheckCessor();
  CheckParallelSegment();
}
//-1 out 0 all 1 part
int LaneSegment::InLane(Geometry *geo) {
  int size = geo->GetVertexCount();
  auto *line = this->GetGeometry();
  for (int i = 0; i < size; i++) {
    Point3d point = geo->GetVertex(i);
    int positive = 0;
    int negative = 0;

    double nearPt = line->GetNeartPoint(point);
    if (nearPt > 0.0) {
      positive++;
    } else if (nearPt < 0.0) {
      negative++;
    }

    if (positive < negative) {
      // int temp = sequence[i];
      // sequence[i] = sequence[j];
      // sequence[j] = temp;
    }
  }
  return -1;
}
}  // namespace geditor
