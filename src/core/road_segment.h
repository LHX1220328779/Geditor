
#pragma once

#include "core/geo_polyline.h"
#include "core/map_feature.h"
#include "core/vector_style.h"

namespace geditor {

class LaneSegment;

class RoadSegment : public MapFeature {
 public:
  RoadSegment();

  virtual ~RoadSegment();

 public:
  void GetSuccessorRoad(std::vector<RoadSegment *> &segArray);

  void GetPredecessorRoad(std::vector<RoadSegment *> &segArray);

  void GetLaneSegment(std::vector<LaneSegment *> &segmentArray);

  void AddSuccessorRoad(RoadSegment *segment);

  void AddPredecessorRoad(RoadSegment *segment);

  void AddLaneSegment(LaneSegment *segment);

 private:
  std::vector<RoadSegment *> m_successor;
  std::vector<RoadSegment *> m_predecessor;
  std::vector<LaneSegment *> m_subLane;
};

}  // namespace geditor
