
#pragma once

#include "core/bound_segment.h"
#include "core/geo_polyline.h"
#include "core/map_feature.h"
#include "core/vector_style.h"

namespace geditor {

class RoadSegment;
class SignBoard;

class LaneSegment : public MapFeature {
 public:
  LaneSegment();

  virtual ~LaneSegment();

 public:
  //����ѡ��״̬
  virtual void SetSelectedState(bool selected);

  virtual void SetHighlightState(bool high);

  RoadSegment *GetParentLink() const;

  void SetParentLink(RoadSegment *pLink);

  void SetHighlightPoint(int index);

  bool IsChanged();

  void SetChanged(bool change);

  PolyLineSytle *GetStyle();

  SegmentProperty *GetProperty();

  void SetProperty(SegmentProperty *pProperty);

  ParallelSegment *GetParallelSegment();

  void SetLeftSegment(MapFeature *l);
  void SetRightSegment(MapFeature *r);
  void SetLeftReverseSegment(MapFeature *l);
  void SetRightReverseSegment(MapFeature *r);

  // LaneSegment *GetLeftSegment() const;

  // LaneSegment *GetRightSegment() const;

  // void SetLeftSegment(LaneSegment *pSegment);

  // void SetRightSegment(LaneSegment *pSegment);

  void GetSuccessorSegment(std::vector<int> &segArray);

  void GetPredecessorSegment(std::vector<int> &segArray);

  void GetSuccessorSegment(std::vector<LaneSegment *> &segArray);

  void GetPredecessorSegment(std::vector<LaneSegment *> &segArray);

  void SetSuccessorSegment(const std::vector<LaneSegment *> &segArray);
  void SetPredecessorSegment(const std::vector<LaneSegment *> &segArray);

  void GetAttachObject(std::vector<MapFeature *> &attachOjbects);
  std::vector<MapFeature *> GetAttachObject() { return m_attachObject; }
  int AttachObjectSize() { return m_attachObject.size(); }

  void AddSuccessorSegment(LaneSegment *segment);

  void AddPredecessorSegment(LaneSegment *segment);

  //
  void SetSuccessorFeature(MapFeature *segment);

  void SetPredecessorFeature(MapFeature *segment);

  MapFeature *GetSuccessorFeature();

  MapFeature *GetPredecessorFeature();

  void AddAttachObject(MapFeature *pOjbect);

  void SetSignBoardObject(SignBoard *pOjbect);

  bool IsSuccessorSegment(LaneSegment *segment);

  bool IsPredecessorSegment(LaneSegment *segment);

  bool IsAttachObject(MapFeature *pOjbect);

  void ClearSuccessorSegment();

  void ClearPredecessorSegment();

  void ClearAttachObject();

  int GetHighlightPoint();

  void SetLeftBoundary(MapFeature *feat) {
    if (feat)
      m_property.leftBoundary = feat->GetUniqueID();
    else
      m_property.leftBoundary = 0;
    m_leftBoundary = feat;
  }

  void SetRightBoundary(MapFeature *feat) {
    if (feat)
      m_property.rightBoundary = feat->GetUniqueID();
    else
      m_property.rightBoundary = 0;
    m_rightBoundary = feat;
  }

  BoundSegment *GetLeftBoundary() { return (BoundSegment *)m_leftBoundary; }

  BoundSegment *GetRightBoundary() { return (BoundSegment *)m_rightBoundary; }
  void OverlapLaneVertex(int type);

  int InLaneNet();

  void CheckAttachObject();
  void CheckBoundary();
  void CheckCessor();
  void CheckParallelSegment();
  void CheckDeletedRealtion();

  int InLane(Geometry *geo);

 private:
  RoadSegment *m_linkRoad = NULL;
  SegmentProperty m_property;
  ParallelSegment m_parallel;

  // LaneSegment *m_leftSegment = NULL;
  // LaneSegment *m_rightSegment = NULL;

  std::vector<LaneSegment *> m_successor;
  std::vector<LaneSegment *> m_predecessor;

  MapFeature *m_successorFeature = NULL;
  MapFeature *m_predecessorFeature = NULL;

  std::vector<MapFeature *> m_attachObject;

  MapFeature *m_leftBoundary = NULL, *m_rightBoundary = NULL;
  MapFeature *m_leftLane = NULL, *m_rightLane = NULL, *m_leftLane_r = NULL,
             *m_rightLane_r = NULL;

 private:
  PolyLineSytle *m_lineStyle = NULL;
  bool m_Changed;
  int m_highlightPoint = 0;
};

}  // namespace geditor
