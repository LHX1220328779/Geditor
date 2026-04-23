
#pragma once

#include "core/geo_polyline.h"
#include "map/map_define.h"
#include "platform/date_time.h"

namespace SQLite {
class Database;

class Transaction;
}  // namespace SQLite

namespace geditor {

class GeoPolyline;

class AreaProperty;

class SegmentProperty;

class ParallelSegment;

class MapFeature;

struct SegmentNode;

struct Lane {
  int uniqueId;
  SegmentProperty pProperty;
  Geometry *polyline;

  Lane() {}

  ~Lane() {}
};

struct Job {
  int uniqueId;
  JobProperty pProperty;
  Geometry *polyline;

  Job() {}

  ~Job() {}
};

struct Area {
  int uniqueId;
  AreaProperty pProperty;
  Geometry *polyline;

  Area() {}

  ~Area() {}
};

struct Sign {
  int uniqueId;
  SignBoardProperty pProperty;
  Geometry *polyline;

  Sign() {}

  ~Sign() {}
};

class VDBManage {
 public:
  VDBManage();

  virtual ~VDBManage();

  bool Create(const char *filename);

  bool Open();

  void Close();

  void BeginTransaction();

  void EndTransaction();

  /*
  bool ReadLink(std::vector<LinkProperty*>& linkArray);
  bool ReadLinkNode(std::vector<LinkNode*>& linkNodeArray);

  bool ReadSegment(std::vector<SegmentProperty*>& segmentArray);

  bool ReadParallelSegment(std::vector<ParallelSegment*>& parallelSegmentArray);

  bool ReadLinkSegmentRelation(std::vector<LinkSegmentRelation*>&
  relationArray);
  */

  bool ReadSegNode(std::vector<SegmentNode *> &segmentNodeArray);

  bool ReadSegmentNode(int uniqueId,
                       std::vector<SegmentNode *> &segmentNodeArray);

  bool ClearRoadLink();

  bool SaveRoadLink(int uniqueId);

  bool ClearRoadNodeRelation();

  bool SaveRoadNodeRelation(int nodeId, SegmentNode *segNode);

  bool ClearLinkSegmentRelation();

  bool ReadLinkSegmentRelation(int uniqueId, int *plink);

  bool SaveLinkSegmentRelation(int uniqueId, int parentLink);

  bool ClearParallelSegment();

  bool SaveParallelSegment(int uniqueId, ParallelSegment *pParallel);

  bool ReadParallelSegment(int uniqueId, ParallelSegment *pParallel);

  bool ClearSegmentProperty();

  bool ReadSegmentProperty(std::vector<Lane *> &segmentArray);

  bool SaveSegmentProperty(int uniqueId, SegmentProperty *pProperty,
                           char *geoBuffer, int length);

  bool ClearBoundaryProperty();

  bool SaveBoundaryProperty(int uniqueId, BoundaryProperty *pProperty,
                            char *geoBuffer, int length);

  bool ReadBoundaryProperty(std::vector<Lane *> &segmentArray);

  bool ClearSegmentNodeRelation();

  bool SaveSegmentNodeRelation(int nodeId, SegmentNode *segNode);

  bool ClearAreaProperty();

  bool SaveAreaProperty(int uniqueId, AreaProperty *pProperty, char *geoBuffer,
                        int length);

  bool ReadAreaProperty(std::vector<Area *> &areaArray);

  bool ClearJobProperty();

  bool SaveJobProperty(int uniqueId, JobProperty *pProperty, char *geoBuffer,
                       int length);

  bool ReadJobProperty(std::vector<Job *> &areaArray);

  bool ClearTopologyProperty();

  bool SaveTopologyProperty(int uniqueId, char *geoBuffer, int length);

  bool ReadTopologyProperty(std::vector<Sign *> &areaArray);

  bool ClearTopologyEdge();

  bool SaveTopologyEdge(int oneId, int twoId);

  bool ReadTopologyEdge(std::vector<std::pair<int, int>> &edgeArray);

  bool ClearSignboardProperty();

  bool SaveSignboardProperty(int uniqueId, SignBoardProperty *pProperty,
                             char *geoBuffer, int length);

  bool ReadSignboardProperty(std::vector<Sign *> &areaArray);

  bool SaveAttachObject(int uniqueId,
                        const std::vector<MapFeature *> &attachObjects);

  bool ReadAttachObject2(int uniqueId, std::vector<int> &attachObjects);
  bool ReadAttachObject(int uniqueId,
                        std::vector<std::pair<int, int>> &attachObjects);

  bool SaveJobAttachObject(int uniqueId,
                           const std::vector<MapFeature *> &attachObjects);

  bool ReadJobAttachObject(int uniqueId, std::vector<int> &attachObjects);

  bool ClearJobSegmentRelation();

  bool SaveJobSegmentRelation(int uniqueId, int jobPredecessor,
                              int jobSuccessor);
  bool ReadJobSegmentRelation(int uniqueId, int &jobPredecessor,
                              int &jobSuccessor);

 private:
  bool HasColumn(const char *table, const char *column);
  bool EnsureSegmentPropertyColumns();
  bool ComputeUTMZone(const char *buffer, int bufSize);

  bool UnpackGeometry(const char *buffer, int bufSize, Geometry *&pointSet);

 private:
  SQLite::Transaction *m_transation;
  SQLite::Database *m_database;

  DateTime m_dateTime;
};

}  // namespace geditor
