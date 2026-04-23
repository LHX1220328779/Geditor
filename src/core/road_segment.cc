
#include "core/road_segment.h"
#include "core/lane_segment.h"

namespace geditor {

RoadSegment::RoadSegment() : MapFeature(MFT_ROAD_SEG) {}

RoadSegment::~RoadSegment() {}

void RoadSegment::GetSuccessorRoad(std::vector<RoadSegment *> &segArray) {
  for (int i = 0; i < m_successor.size(); i++) {
    segArray.push_back(m_successor[i]);
  }
}

void RoadSegment::GetPredecessorRoad(std::vector<RoadSegment *> &segArray) {
  for (int i = 0; i < m_predecessor.size(); i++) {
    segArray.push_back(m_predecessor[i]);
  }
}

void RoadSegment::GetLaneSegment(std::vector<LaneSegment *> &segmentArray) {
  for (int i = 0; i < m_subLane.size(); i++) {
    segmentArray.push_back(m_subLane[i]);
  }
}

void RoadSegment::AddSuccessorRoad(RoadSegment *segment) {
  bool bNotFind = true;

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

void RoadSegment::AddPredecessorRoad(RoadSegment *segment) {
  bool bNotFind = true;

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

void RoadSegment::AddLaneSegment(LaneSegment *segment) {
  bool bNotFind = true;

  for (int i = 0; i < m_subLane.size(); i++) {
    if (m_subLane[i] == segment) {
      bNotFind = false;
      break;
    }
  }

  if (bNotFind) {
    m_subLane.push_back(segment);
  }
}

}  // namespace geditor
