
#include "map/vdb_manage.h"

#include <sqlite_cpp.h>
#include <transaction.h>

#include <cstdio>

#include "core/geo_arc_line.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_circular_arc.h"
#include "core/geo_polygon.h"
#include "core/geo_polygon_hole.h"
#include "core/geo_polyline.h"
#include "core/geo_rectangle.h"
#include "core/map_feature.h"
#include "map/projection_utm.h"
#include "platform/date_time.h"

namespace geditor {

VDBManage::VDBManage() : m_database(NULL), m_transation(NULL) {}

VDBManage::~VDBManage() {
  if (m_database != NULL) {
    delete m_database;
    m_database = NULL;
  }
}

bool VDBManage::Create(const char *filename) {
  m_database = new SQLite::Database(filename);
  if (m_database->open()) {
    const char pJobProperty[] =
        "create table if not exists job_property( areaID INTEGER NOT NULL, "
        "jobId INTEGER NOT NULL, areaType INTEGER NOT NULL, gpsParam INTEGER, "
        "proParam REAL,areaOrder INTEGER, name TEXT, geometry BLOB, flag "
        "INTEGER, date TEXT,PRIMARY KEY(areaID))";
    const char pSignBoardProperty[] =
        "create table if not exists signboard_property( signID INTEGER NOT "
        "NULL, signType INTEGER NOT NULL, signValue INTEGER,geometry TEXT, "
        "flag INTEGER, date TEXT,PRIMARY KEY(signID))";
    const char pAreaProperty[] =
        "create table if not exists area_property( areaID INTEGER NOT NULL, "
        "areaType INTEGER NOT NULL, gpsParam INTEGER, proParam REAL,  geometry "
        "TEXT, flag INTEGER, date TEXT,PRIMARY KEY(areaID))";
    const char pBoundaryProperty[] =
        "create table if not exists boundary_property (boundaryID INTEGER,  "
        "length INTEGER, geometry TEXT, flag INTEGER,PRIMARY KEY(boundaryID))";
    const char pSegmentProperty[] =
        "create table if not exists segment_property (segmentID INTEGER, speed "
        "REAL, length INTEGER, turnType INTEGER, laneType INTEGER, pitch REAL, "
        "leftBoundary INTEGER, rightBoundary INTEGER, leftBorderType INTEGER, "
        "rightBorderType INTEGER, mineSegmentIndex INTEGER, "
        "mineSegmentCode TEXT, name TEXT,geometry BLOB, flag, date "
        "TEXT,PRIMARY KEY(segmentID))";
    const char pSegmentParallel[] =
        "create table if not exists segment_parallel_relation (segmentID "
        "INTEGER, segmentLeft INTEGER, segmentRight INTEGER, "
        "segmentLeftReverse INTEGER, segmentRightReverse INTEGER, flag "
        "INTEGER, date TEXT, PRIMARY KEY(segmentID))";
    const char pSegmentNode[] =
        "create table if not exists segment_node_relation ( sequence INTEGER "
        "NOT NULL, nodeID INTEGER, segmentID INTEGER, segmentType INTEGER, "
        "date TEXT, PRIMARY KEY(sequence) )";
    const char pLinkSegment[] =
        "create table if not exists link_segment_relation ( segmentID INTEGER, "
        "parentLink INTEGER, flag INTEGER, date TEXT, primary key (segmentID, "
        "parentLink))";
    const char pLinkProperty[] =
        "create table if not exists link_property ( linkID INTEGER, speed "
        "INTEGER, length INTEGER, type INTEGER, geometry TEXT, flag INTEGER )";
    const char pLinkNode[] =
        "create table if not exists link_node_relation ( sequence INTEGER NOT "
        "NULL PRIMARY KEY AUTOINCREMENT, nodeID INTEGER NOT NULL, linkID "
        "INTEGER NOT NULL, linkType INTEGER, flag INTEGER, date TEXT )";
    const char pJobSegment[] =
        "create table if not exists job_segment_relation (segmentID INTEGER, "
        "jobSuccessor INTEGER, JobPredecessor INTEGER, date TEXT, primary key "
        "(segmentID))";
    const char pSegmentAttach[] =
        "create table if not exists segment_attach_object (segmentId INTEGER "
        "NOT NULL, objectId INTEGER NOT NULL,objectType INTEGER NOT "
        "NULL, date TEXT,primary key (segmentId, objectId, objectType))";
    const char pJobAttach[] =
        "create table if not exists job_attach_object (jobId INTEGER NOT NULL, "
        "objectId INTEGER NOT NULL, sequence INTEGER, date TEXT,primary key "
        "(jobId, objectId))";
    const char pTopologyProperty[] =
        "create table if not exists topology_property (pointId INTEGER, "
        "geometry TEXT, date TEXT, primary key (pointId))";
    const char pTopologyEdge[] =
        "create table if not exists topology_edge (pointOneId INTEGER NOT "
        "NULL, pointTwoId INTEGER NOT NULL, date TEXT,primary key (pointOneId, "
        "pointTwoId))";

    int ret0 = m_database->exec(pBoundaryProperty);
    int ret1 = m_database->exec(pSegmentProperty);
    int ret2 = m_database->exec(pSegmentParallel);
    int ret3 = m_database->exec(pSegmentNode);
    int ret4 = m_database->exec(pLinkSegment);
    int ret5 = m_database->exec(pLinkProperty);
    int ret6 = m_database->exec(pLinkNode);
    int ret7 = m_database->exec(pAreaProperty);
    int ret8 = m_database->exec(pSignBoardProperty);
    int ret9 = m_database->exec(pSegmentAttach);
    int ret10 = m_database->exec(pJobSegment);
    int ret11 = m_database->exec(pJobProperty);

    int ret12 = m_database->exec(pTopologyProperty);
    int ret13 = m_database->exec(pTopologyEdge);
    int ret14 = m_database->exec(pJobAttach);

    if ((ret0 | ret1 | ret2 | ret3 | ret4 | ret5 | ret6 | ret7 | ret8 | ret9 |
         ret10 | ret11 | ret12 | ret13 | ret14) == 0) {
      if (!EnsureSegmentPropertyColumns()) {
        return false;
      }
      return true;
    }
  }

  return false;
}

bool VDBManage::HasColumn(const char *table, const char *column) {
  std::string sql = "PRAGMA table_info(" + std::string(table) + ")";
  SQLite::Statement query(*m_database, sql.c_str());
  while (query.executeStep()) {
    const char *column_name = query.getColumn(1).getText();
    if (column_name != NULL && std::string(column_name) == column) {
      return true;
    }
  }
  return false;
}

bool VDBManage::EnsureSegmentPropertyColumns() {
  if (!HasColumn("segment_property", "mineSegmentIndex")) {
    const char *sql =
        "ALTER TABLE segment_property ADD COLUMN mineSegmentIndex INTEGER "
        "DEFAULT 0";
    if (m_database->exec(sql) != 0) {
      return false;
    }
  }
  if (!HasColumn("segment_property", "mineSegmentCode")) {
    const char *sql =
        "ALTER TABLE segment_property ADD COLUMN mineSegmentCode TEXT DEFAULT "
        "''";
    if (m_database->exec(sql) != 0) {
      return false;
    }
  }
  return true;
}

bool VDBManage::Open() {
  if (m_database != NULL) {
    if (m_database->open()) {
      return true;
    }
  }

  return false;
}

void VDBManage::Close() {
  if (m_database != NULL) {
    m_database->close();
  }
}

void VDBManage::BeginTransaction() {
  m_transation = new SQLite::Transaction(*m_database);
}

void VDBManage::EndTransaction() {
  if (m_transation) {
    m_transation->commit();

    delete m_transation;
  }
}

bool VDBManage::ClearTopologyEdge() {
  const char *szSql = "delete from topology_edge";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveTopologyEdge(int oneId, int twoId) {
  const char *szSql =
      "insert or replace into topology_edge(pointOneId, pointTwoId) values(? , "
      " ? )";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, oneId);
  query.bind(2, twoId);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::ReadTopologyEdge(std::vector<std::pair<int, int>> &edgeArray) {
  const char *szSql = "select pointOneId, pointTwoId from topology_edge";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int oneId = query.getColumn(0).getInt();
    int twoId = query.getColumn(1).getInt();

    edgeArray.push_back(std::pair<int, int>(oneId, twoId));
  }

  return true;
}

bool VDBManage::ClearTopologyProperty() {
  const char *szSql = "delete from topology_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveTopologyProperty(int uniqueId, char *geoBuffer,
                                     int length) {
  const char *szSql =
      "insert or replace into topology_property(pointId, geometry, date) "
      "values(? , ?,  ? )";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);

  if (geoBuffer != NULL && length > 0) {
    query.bind(2, geoBuffer, length);
  }

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(3, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::ReadTopologyProperty(std::vector<Sign *> &areaArray) {
  const char *szSql = "select pointId, geometry from topology_property";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int areaID = query.getColumn(0).getInt();

    //��ȡ�㴮
    const void *buffer = query.getColumn(1).getBlob();
    int bufSize = query.getColumn(1).getBytes();

    Sign *pLane = new Sign();

    Geometry *pointSet = NULL;
    if (bufSize > 0 && buffer != NULL) {
      if (!UnpackGeometry((const char *)buffer, bufSize, pointSet)) {
        delete pointSet;
        pointSet = NULL;

        continue;
      }
    }

    pLane->uniqueId = areaID;
    pLane->polyline = pointSet;

    areaArray.push_back(pLane);
  }

  return true;
}

bool VDBManage::ReadJobProperty(std::vector<Job *> &areaArray) {
  const char *szSql =
      "select areaID, jobId, areaType, gpsParam, proParam, areaOrder,name, "
      "geometry from job_property";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int areaID = query.getColumn(0).getInt();
    int jobId = query.getColumn(1).getInt();
    int areaType = query.getColumn(2).getInt();
    int gpsParam = query.getColumn(3).getInt();
    float proParam = (float)query.getColumn(4).getDouble();

    int areaOrder = query.getColumn(5).getInt();

    //����
    const char *nameID = query.getColumn(6).getText();

    //��ȡ�㴮
    const void *buffer = query.getColumn(7).getBlob();
    int bufSize = query.getColumn(7).getBytes();

    Job *pLane = new Job();

    Geometry *pointSet = NULL;
    if (bufSize > 0 && buffer != NULL) {
      if (!UnpackGeometry((const char *)buffer, bufSize, pointSet)) {
        delete pointSet;
        pointSet = NULL;

        continue;
      }
    }

    pLane->uniqueId = areaID;
    pLane->pProperty.jobId = jobId;
    pLane->pProperty.areaType = areaType;
    pLane->pProperty.gpsParam = gpsParam;
    pLane->pProperty.proParam = proParam;

    if (nameID != NULL) {
      strcpy(pLane->pProperty.name, nameID);
    } else {
      strcpy(pLane->pProperty.name, "");
    }
    pLane->polyline = pointSet;

    areaArray.push_back(pLane);
  }

  return true;
}

bool VDBManage::ReadAreaProperty(std::vector<Area *> &areaArray) {
  const char *szSql =
      "select areaID, areaType, gpsParam, proParam, geometry from "
      "area_property";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int areaID = query.getColumn(0).getInt();
    float areaType = (float)query.getColumn(1).getDouble();
    int gpsParam = query.getColumn(2).getInt();
    float proParam = (float)query.getColumn(3).getDouble();

    //��ȡ�㴮
    const void *buffer = query.getColumn(4).getBlob();
    int bufSize = query.getColumn(4).getBytes();

    Area *pLane = new Area();

    Geometry *pointSet = NULL;
    if (bufSize > 0 && buffer != NULL) {
      if (!UnpackGeometry((const char *)buffer, bufSize, pointSet)) {
        delete pointSet;
        pointSet = NULL;
      }
    }

    pLane->uniqueId = areaID;
    pLane->pProperty.areaType = areaType;
    pLane->pProperty.gpsParam = gpsParam;
    pLane->pProperty.proParam = proParam;
    pLane->polyline = pointSet;

    areaArray.push_back(pLane);
  }

  return true;
}

bool VDBManage::ClearSignboardProperty() {
  const char *szSql = "delete from signboard_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveSignboardProperty(int uniqueId,
                                      SignBoardProperty *pProperty,
                                      char *geoBuffer, int length) {
  const char *szSql =
      "insert or replace into signboard_property(signID, signType, geometry, "
      "flag, date) values(? ,? ,?, ?, ? )";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);

  if (pProperty != NULL) {
    query.bind(2, pProperty->areaType);
  }

  if (geoBuffer != NULL && length > 0) {
    query.bind(3, geoBuffer, length);
  }
  if (pProperty != NULL) {
    query.bind(4, pProperty->stopline);
  }

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(5, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::ReadSignboardProperty(std::vector<Sign *> &areaArray) {
  const char *szSql =
      "select signID, signType, geometry, flag from signboard_property";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int areaID = query.getColumn(0).getInt();
    float areaType = (float)query.getColumn(1).getDouble();

    //��ȡ�㴮
    const void *buffer = query.getColumn(2).getBlob();
    int bufSize = query.getColumn(2).getBytes();

    int stopline = query.getColumn(3).getInt();

    Sign *pLane = new Sign();

    Geometry *pointSet = NULL;
    if (bufSize > 0 && buffer != NULL) {
      if (!UnpackGeometry((const char *)buffer, bufSize, pointSet)) {
        delete pointSet;
        pointSet = NULL;
      }
    }

    pLane->uniqueId = areaID;
    pLane->pProperty.stopline = stopline;
    pLane->pProperty.areaType = areaType;
    pLane->polyline = pointSet;

    areaArray.push_back(pLane);
  }

  return true;
}

bool VDBManage::ClearAreaProperty() {
  const char *szSql = "delete from area_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ClearJobProperty() {
  const char *szSql = "delete from job_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveJobProperty(int uniqueId, JobProperty *pProperty,
                                char *geoBuffer, int length) {
  const char *szSql =
      "insert or replace into job_property(areaID, jobId, areaType, gpsParam, "
      "proParam, areaOrder, name, geometry, flag, date) values(?,?,?,? ,?,?,? "
      ",?, ?, ? )";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);

  if (pProperty != NULL) {
    query.bind(2, pProperty->jobId);
    query.bind(3, pProperty->areaType);
    query.bind(4, pProperty->gpsParam);
    query.bind(5, pProperty->proParam);
    query.bind(6, pProperty->order);
    query.bind(7, pProperty->name);
  }

  if (geoBuffer != NULL && length > 0) {
    query.bind(8, geoBuffer, length);
  }

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(9, 0);
  query.bind(10, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::SaveAreaProperty(int uniqueId, AreaProperty *pProperty,
                                 char *geoBuffer, int length) {
  const char *szSql =
      "insert or replace into area_property(areaID, areaType, gpsParam, "
      "proParam, geometry, flag, date) values(? ,? ,?,?,?, ?, ? )";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);

  if (pProperty != NULL) {
    query.bind(2, pProperty->areaType);
    query.bind(3, pProperty->gpsParam);
    query.bind(4, pProperty->proParam);
  }

  if (geoBuffer != NULL && length > 0) {
    query.bind(5, geoBuffer, length);
  }

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(6, 0);
  query.bind(7, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::SaveSegmentProperty(int uniqueId, SegmentProperty *pProperty,
                                    char *geoBuffer, int length) {
  const char *szSql =
      "insert or replace into segment_property(segmentID, speed, length, "
      "turnType, laneType, pitch, "
      "leftBoundary,rightBoundary,leftBorderType,rightBorderType, "
      "mineSegmentIndex, mineSegmentCode, name, geometry, flag, date) "
      "values(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);

  if (pProperty != NULL) {
    query.bind(2, pProperty->speed);
    query.bind(3, pProperty->length);
    query.bind(4, pProperty->turnType);
    query.bind(5, pProperty->laneType);
    query.bind(6, pProperty->lanePitch);

    query.bind(7, pProperty->leftBoundary);
    query.bind(8, pProperty->rightBoundary);

    query.bind(9, pProperty->leftBorderType);
    query.bind(10, pProperty->rightBorderType);
    query.bind(11, pProperty->mineSegmentIndex);
    query.bind(12, pProperty->mineSegmentCode);
    query.bind(13, pProperty->name);
  }

  if (geoBuffer != NULL && length > 0) {
    query.bind(14, geoBuffer, length);
  }

  query.bind(15, 0);

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(16, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::ClearBoundaryProperty() {
  const char *szSql = "delete from boundary_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveBoundaryProperty(int uniqueId, BoundaryProperty *pProperty,
                                     char *geoBuffer, int length) {
  const char *szSql =
      "insert or replace into boundary_property(boundaryID,  length,  "
      "geometry, flag) values(? , ? ,?, ? )";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);

  if (pProperty != NULL) {
    query.bind(2, pProperty->length);
  }

  if (geoBuffer != NULL && length > 0) {
    query.bind(3, geoBuffer, length);
  }

  if (pProperty != NULL) {
    query.bind(4, pProperty->boundType);
  }

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ClearParallelSegment() {
  const char *szSql = "delete from segment_parallel_relation";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveParallelSegment(int uniqueId, ParallelSegment *pParallel) {
  const char *szSql =
      "insert or replace into segment_parallel_relation(segmentID, "
      "segmentLeft, segmentRight, segmentLeftReverse, segmentRightReverse, "
      "flag, date) values(?, ?, ?, ?, ?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);
  if (pParallel != NULL) {
    query.bind(2, pParallel->leftSegment);
    query.bind(3, pParallel->rightSegment);
    query.bind(4, pParallel->leftReverseSegment);
    query.bind(5, pParallel->rightReverseSegment);
  }
  query.bind(6, 0);

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(7, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ClearJobSegmentRelation() { return true; }

bool VDBManage::SaveJobSegmentRelation(int uniqueId, int jobPredecessor,
                                       int jobSuccessor) {
  const char *szSql =
      "insert or replace into job_segment_relation (segmentID, jobSuccessor, "
      "JobPredecessor) values(?, ?, ?)";

  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);
  query.bind(2, jobSuccessor);
  query.bind(3, jobPredecessor);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ReadJobSegmentRelation(int uniqueId, int &jobPredecessor,
                                       int &jobSuccessor) {
  const char *szSql =
      "select jobSuccessor, JobPredecessor from job_segment_relation where "
      "segmentID = ?";

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, uniqueId);

  if (query.executeStep()) {
    jobSuccessor = query.getColumn(0).getInt();
    jobPredecessor = query.getColumn(1).getInt();
    return true;
  }

  return false;
}

bool VDBManage::ReadAttachObject2(int uniqueId,
                                  std::vector<int> &attachObjects) {
  const char *szSql =
      "select segmentId, objectId from segment_attach_object where objectId = "
      "?";

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, uniqueId);

  while (query.executeStep()) {
    int segmentId = query.getColumn(0).getInt();
    int objectId = query.getColumn(1).getInt();

    attachObjects.push_back(segmentId);
  }
  return true;
}

bool VDBManage::ReadAttachObject(
    int uniqueId, std::vector<std::pair<int, int>> &attachObjects) {
  const char *szSql =
      "select segmentId, objectId ,objectType from segment_attach_object "
      "where segmentId = ?";

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, uniqueId);

  while (query.executeStep()) {
    int segmentId = query.getColumn(0).getInt();
    int objectId = query.getColumn(1).getInt();
    int objectType = query.getColumn(2).getInt();

    attachObjects.push_back({objectId, objectType});
  }
  return true;
}

bool VDBManage::ReadJobAttachObject(int uniqueId,
                                    std::vector<int> &attachObjects) {
  const char *szSql =
      "select objectId from job_attach_object where objectId = ?";

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, uniqueId);

  while (query.executeStep()) {
    int objectId = query.getColumn(0).getInt();

    attachObjects.push_back(objectId);
  }
  return true;
}

bool VDBManage::SaveJobAttachObject(
    int uniqueId, const std::vector<MapFeature *> &attachObjects) {
  const char *szSql =
      "insert or replace into job_attach_object(jobId, objectId, sequence) "
      "values(?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  for (int i = 0; i < attachObjects.size(); i++) {
    int unId = attachObjects[i]->GetUniqueID();

    query.bind(1, uniqueId);
    query.bind(2, unId);
    query.bind(3, i + 1);

    int ret = query.exec();
    if (ret <= 0) {
      return false;
    }
    query.reset();
  }

  return true;
}

bool VDBManage::SaveAttachObject(
    int uniqueId, const std::vector<MapFeature *> &attachObjects) {
  const char *szSql =
      "insert or replace into segment_attach_object (segmentId, "
      "objectId, objectType, date) values(?, ?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  for (int i = 0; i < attachObjects.size(); i++) {
    query.bind(1, uniqueId);

    query.bind(2, attachObjects[i]->GetUniqueID());
    query.bind(3, attachObjects[i]->GetType());

    char tmBuffer[64];
    m_dateTime.SetToNow();
    m_dateTime.Format(tmBuffer, 64);

    query.bind(4, tmBuffer);

    int ret = query.exec();
    if (ret <= 0) {
      return false;
    }
    query.reset();
  }
  return true;
}

bool VDBManage::SaveLinkSegmentRelation(int uniqueId, int parentLink) {
  const char *szSql =
      "insert or replace into link_segment_relation (segmentID, parentLink, "
      "flag, date) values(?, ?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);
  query.bind(2, parentLink);
  query.bind(3, 3);

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(4, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ClearSegmentNodeRelation() {
  const char *szSql = "delete from segment_node_relation";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveSegmentNodeRelation(int nodeId, SegmentNode *segNode) {
  const char *szSql =
      "insert or replace into segment_node_relation (nodeID, segmentID, "
      "segmentType, date) values(?, ?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, nodeId);
  query.bind(2, segNode->segment);
  query.bind(3, segNode->type);

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(4, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ClearRoadNodeRelation() {
  const char *szSql = "delete from link_node_relation";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveRoadNodeRelation(int nodeId, SegmentNode *segNode) {
  const char *szSql =
      "insert or replace into link_node_relation (nodeID, linkID, linkType, "
      "date) values(?, ?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, nodeId);
  query.bind(2, segNode->segment);
  query.bind(3, segNode->type);

  char tmBuffer[64];
  m_dateTime.SetToNow();
  m_dateTime.Format(tmBuffer, 64);

  query.bind(4, tmBuffer);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ReadSegmentNode(int uniqueId,
                                std::vector<SegmentNode *> &segmentNodeArray) {
  const char *szSql =
      "select nodeID, segmentID, segmentType from segment_node_relation where "
      "segmentID = ? ";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int nodeID = query.getColumn(0).getInt();
    int segmentID = query.getColumn(1).getInt();
    int segmentType = query.getColumn(2).getInt();

    SegmentNode *pSeg = new SegmentNode();

    // pSeg->segNode = nodeID;
    pSeg->segment = segmentID;
    pSeg->type = (SegmentType)segmentType;

    segmentNodeArray.push_back(pSeg);
  }
  return true;
}

bool VDBManage::ClearRoadLink() {
  const char *szSql = "delete from link_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::SaveRoadLink(int uniqueId) {
  const char *szSql =
      "insert or replace into link_property (linkID, speed, length, flag) "
      "values(?, ?, ?, ?)";
  SQLite::Statement query(*m_database, szSql);

  query.bind(1, uniqueId);
  query.bind(2, 0.0);
  query.bind(3, 0.0);
  query.bind(4, 0);

  int ret = query.exec();
  if (ret > 0) {
    return true;
  }

  return false;
}

bool VDBManage::ClearLinkSegmentRelation() {
  const char *szSql = "delete from link_segment_relation";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ReadLinkSegmentRelation(int uniqueId, int *plink) {
  const char *szSql =
      "select parentLink, flag from link_segment_relation where segmentID = ?";

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, uniqueId);

  if (query.executeStep()) {
    int parentLink = query.getColumn(0).getInt();
    int flag = query.getColumn(1).getInt();

    *plink = parentLink;

    return true;
  }

  return false;
}

bool VDBManage::ReadParallelSegment(int uniqueId, ParallelSegment *pParallel) {
  const char *szSql =
      "select segmentLeft, segmentRight, segmentLeftReverse, "
      "segmentRightReverse from segment_parallel_relation where "
      "segmentID = ?";

  SQLite::Statement query(*m_database, szSql);
  query.bind(1, uniqueId);

  if (query.executeStep()) {
    int segmentLeft = query.getColumn(0).getInt();
    int segmentRight = query.getColumn(1).getInt();
    int segmentLeftReverse = query.getColumn(2).getInt();
    int segmentRightReverse = query.getColumn(3).getInt();

    pParallel->leftSegment = segmentLeft;
    pParallel->rightSegment = segmentRight;
    pParallel->leftReverseSegment = segmentLeftReverse;
    pParallel->rightReverseSegment = segmentRightReverse;

    return true;
  }

  return false;
}

bool VDBManage::ReadBoundaryProperty(std::vector<Lane *> &segmentArray) {
  const char *szSql =
      "select boundaryID, length, geometry, flag from boundary_property";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int segmentID = query.getColumn(0).getInt();
    float length = (float)query.getColumn(1).getDouble();
    int bound_type = query.getColumn(3).getInt();

    //��ȡ�㴮
    const void *buffer = query.getColumn(2).getBlob();
    int bufSize = query.getColumn(2).getBytes();

    Lane *pLane = new Lane();

    Geometry *pointSet = NULL;
    if (bufSize > 0 && buffer != NULL) {
      if (!UnpackGeometry((const char *)buffer, bufSize, pointSet)) {
        delete pointSet;
        pointSet = NULL;
      }
    }

    pLane->uniqueId = segmentID;
    pLane->pProperty.speed = 0;
    pLane->pProperty.length = length;
    pLane->pProperty.turnType = 0;
    pLane->pProperty.rightBorderType = bound_type;
    pLane->polyline = pointSet;

    segmentArray.push_back(pLane);
  }

  return true;
}

bool VDBManage::ClearSegmentProperty() {
  const char *szSql = "delete from segment_property";
  SQLite::Statement query(*m_database, szSql);

  int ret = query.exec();
  if (ret > 0) {
    return false;
  }

  return true;
}

bool VDBManage::ReadSegmentProperty(std::vector<Lane *> &segmentArray) {
  const char *szSql =
      "select segmentID, speed, length, turnType, laneType, pitch, "
      "leftBoundary , rightBoundary, leftBorderType, rightBorderType, "
      "mineSegmentIndex, mineSegmentCode, name, geometry from "
      "segment_property";

  SQLite::Statement query(*m_database, szSql);

  bool flag = false;

  while (query.executeStep()) {
    int segmentID = query.getColumn(0).getInt();
    float speed = (float)query.getColumn(1).getDouble();
    float length = (float)query.getColumn(2).getDouble();
    int turnType = query.getColumn(3).getInt();
    int laneType = query.getColumn(4).getInt();
    float pitch = query.getColumn(5).getDouble();
    int leftBoundary = query.getColumn(6).getInt();
    int rightBoundary = query.getColumn(7).getInt();
    int leftBorderType = query.getColumn(8).getInt();
    int rightBorderType = query.getColumn(9).getInt();
    int mineSegmentIndex = query.getColumn(10).getInt();
    const char *mineSegmentCode = query.getColumn(11).getText();

    const char *nameID = query.getColumn(12).getText();

    //��ȡ�㴮
    const void *buffer = query.getColumn(13).getBlob();
    int bufSize = query.getColumn(13).getBytes();

    if (!flag) {
      if (bufSize > 0 && buffer != NULL) {
        flag = ComputeUTMZone((const char *)buffer, bufSize);
      }
    }

    Lane *pLane = new Lane();

    Geometry *pointSet = NULL;
    if (bufSize > 0 && buffer != NULL) {
      if (!UnpackGeometry((const char *)buffer, bufSize, pointSet)) {
        delete pointSet;
        pointSet = NULL;
      }
    }

    pLane->uniqueId = segmentID;
    pLane->pProperty.speed = speed;
    pLane->pProperty.length = length;
    pLane->pProperty.turnType = turnType;
    pLane->pProperty.laneType = laneType;
    pLane->pProperty.lanePitch = pitch;
    pLane->pProperty.leftBoundary = leftBoundary;
    pLane->pProperty.rightBoundary = rightBoundary;

    pLane->pProperty.leftBorderType = leftBorderType;
    pLane->pProperty.rightBorderType = rightBorderType;
    pLane->pProperty.mineSegmentIndex = mineSegmentIndex;
    if (mineSegmentCode != NULL) {
      strncpy(pLane->pProperty.mineSegmentCode, mineSegmentCode,
              sizeof(pLane->pProperty.mineSegmentCode) - 1);
    }

    pLane->polyline = pointSet;
    if (nameID != NULL) {
      strcpy(pLane->pProperty.name, nameID);
    } else {
      strcpy(pLane->pProperty.name, "");
    }

    segmentArray.push_back(pLane);
  }

  return true;
}

bool VDBManage::ReadSegNode(std::vector<SegmentNode *> &segmentArray) {
  const char *szSql =
      "select nodeID, segmentID, segmentType from segment_node_relation ";

  SQLite::Statement query(*m_database, szSql);

  while (query.executeStep()) {
    int nodeID = query.getColumn(0).getInt();
    int segmentID = query.getColumn(1).getInt();
    int segmentType = query.getColumn(2).getInt();

    SegmentNode *pSeg = new SegmentNode();

    // pSeg->segNode = nodeID;
    pSeg->segment = segmentID;
    pSeg->type = (SegmentType)segmentType;

    segmentArray.push_back(pSeg);
  }

  return true;
}

/*
bool VDBManage::ReadLink(std::vector<LinkProperty*>& linkArray)
{
        const char* szSql = "select linkID, speed, length, type, geometry from
link_property";


        SQLite::Statement query(*m_database, szSql);

        while (query.executeStep())
        {

                //��ȡ��������
                int    linkID = query.getColumn(0).getInt();
                float  speed = (float)query.getColumn(1).getDouble();
                float  length = (float)query.getColumn(2).getDouble();
                int    type = query.getColumn(3).getInt();

                //��ȡ�㴮
                const void* buffer = query.getColumn(4).getBlob();
                int bufSize = query.getColumn(4).getBytes();

                GeometryLine*  pointSet = NULL;
                if (bufSize > 0 && buffer != NULL)
                {
                        pointSet = new GeometryLine();
                        if (!ParsePointSet((const char*)buffer, bufSize,
pointSet))
                        {
                                delete pointSet;
                                pointSet = NULL;
                        }
                }

                //����
                LinkProperty* pLink = new LinkProperty();
                pLink->link = linkID;
                pLink->length = length;
                pLink->speed = speed;
                pLink->type = type;
                pLink->pointSet = pointSet;

                linkArray.push_back(pLink);
        }

        if (linkArray.size() <= 0)
        {
                return false;
        }

        return true;

}


bool VDBManage::ReadLinkNode(std::vector<LinkNode*>& linkNodeArray)
{
        const char* szSql = "select nodeID, linkID, linkType from
link_node_relation";

        SQLite::Statement query(*m_database, szSql);

        while (query.executeStep())
        {
                int  nodeID = query.getColumn(0).getInt();
                int  linkID = query.getColumn(1).getInt();
                int  linkType = query.getColumn(2).getInt();

                LinkNode* pSeg = new LinkNode();

                pSeg->linkNode = nodeID;
                pSeg->link = linkID;
                pSeg->linkType = linkType;

                linkNodeArray.push_back(pSeg);
        }

        return true;
}



bool VDBManage::ReadParallelSegment(std::vector<ParallelSegment*>& segmentArray)
{
        const char* szSql = "select segmentID, segmentRight, segmentLeft from
segment_parallel_relation";

        SQLite::Statement query(*m_database, szSql);

        while (query.executeStep())
        {
                int  segment = query.getColumn(0).getInt();
                int  segmentright = query.getColumn(1).getInt();
                int  segmentleft = query.getColumn(2).getInt();

                ParallelSegment* pSeg = new ParallelSegment();

                pSeg->segment = segment;
                pSeg->right = segmentright;
                pSeg->left = segmentleft;

                segmentArray.push_back(pSeg);
        }

        return true;
}

bool VDBManage::ReadLinkSegmentRelation(std::vector<LinkSegmentRelation*>&
relationArray)
{
        const char* szSql = "select segmentID, parentLink from
link_segment_relation";

        SQLite::Statement query(*m_database, szSql);

        while (query.executeStep())
        {
                int  segmentID = query.getColumn(0).getInt();
                int  parentLink = query.getColumn(1).getInt();

                LinkSegmentRelation* pSeg = new LinkSegmentRelation();

                pSeg->segment = segmentID;
                pSeg->parentLink = parentLink;

                relationArray.push_back(pSeg);
        }

        return true;
}


*/

/*
bool VDBManage::ParsePointSet(const char* buffer, int bufSize, GeoPolyline*
pointSet)
{
        do{

                double x = atof(buffer);
                while (*buffer++ != ',');
                double y = atof(buffer);
                while (*buffer++ != '\n');

                pointSet->AppendVertex(x, y, 0.0f);

        } while (*buffer != 0);

        return true;
}
*/

struct GEO_HDR {
  unsigned int magic;
  unsigned short version;
  unsigned short reserve;
  unsigned int crc32;
};

struct GEO_INFO {
  int geotype;  //ͼ�����
  int count;    //������
  double minLat;
  double maxLat;
  double maxLon;
  double minLon;
  float minAlt;
  float maxAlt;
};

bool VDBManage::ComputeUTMZone(const char *buffer, int bufSize) {
  if (bufSize < 12) {
    return false;
  }

  GEO_HDR hdr;
  GEO_INFO info;

  int pos = 0;

  memcpy(&hdr, buffer + pos, sizeof(GEO_HDR));
  pos += sizeof(GEO_HDR);

  if (hdr.magic != 0x00306567 || hdr.version != 1) {
    return false;
  }

  //---------------------------------------------
  memcpy(&info, buffer + pos, sizeof(GEO_INFO));
  pos += sizeof(GEO_INFO);

  GPSPoint *ptrPnt = (GPSPoint *)(buffer + pos);

  if (ptrPnt != NULL) {
    double lat = ptrPnt->latlon.lon;
    int nzone = (int)(lat / 6.0 + 31);
    ProjectionUTM::zone = nzone;
    return true;
  }

  return false;
}

bool VDBManage::UnpackGeometry(const char *buffer, int bufSize,
                               Geometry *&pointSet) {
  if (bufSize < sizeof(GEO_HDR) + sizeof(GEO_INFO)) {
    return false;
  }

  GEO_HDR hdr;
  GEO_INFO info;

  int pos = 0;

  memcpy(&hdr, buffer + pos, sizeof(GEO_HDR));
  pos += sizeof(GEO_HDR);

  if (hdr.magic != 0x00306567) {
    return false;
  }

  if (hdr.version != 1) {
    return false;
  }

  //---------------------------------------------
  memcpy(&info, buffer + pos, sizeof(GEO_INFO));
  pos += sizeof(GEO_INFO);

  Geometry *pGeometry = NULL;
  if (info.geotype == Geometry::GT_POLYLINE) {
    pGeometry = new GeoPolyline();
  } else if (info.geotype == Geometry::GT_BEZIER_CURVE) {
    pGeometry = new GeoBezierCurve3();
  } else if (info.geotype == Geometry::GT_POLYGON) {
    pGeometry = new GeoPolygon();
  } else if (info.geotype == Geometry::GT_BSPLINE_CURVE) {
    pGeometry = new GeoBSplineCurve3();
  } else if (info.geotype == Geometry::GT_CIRCULAR_ARC) {
    pGeometry = new GeoCircularArc();
  } else if (info.geotype == Geometry::GT_HERMITE_CURVE) {
    // pGeometry = new GeoHermiteCurve3();
  } else if (info.geotype == Geometry::GT_RECTANGLE) {
    pGeometry = new GeoRectangle();
  } else if (info.geotype == Geometry::GT_POLYGON_HOLE) {
    pGeometry = new GeoPolygonHole();
  } else if (info.geotype == Geometry::GT_ARC_LINE) {
    pGeometry = new GeoArcLine();
  } else {
    return false;
  }

  int nCount = 0;
  pos += info.count * sizeof(GPSPoint);
  nCount = hdr.reserve;
  // if (info.geotype == Geometry::GT_POLYLINE) {
  //   nCount = info.count;
  // } else {
  //   pos += info.count * sizeof(GPSPoint);
  //   nCount = hdr.reserve;
  // }

  ProjectionUTM projectionUTM;
  //------------------------------------------------
  GPSPoint *ptrPnt = (GPSPoint *)(buffer + pos);

  if (info.geotype == Geometry::GT_POLYGON_HOLE) {
    //��������
    std::vector<Point3d> utmPoints;
    for (int i = 0; i < nCount; i++) {
      UTMPoint utmxy;
      projectionUTM.LatLonToCartesian((ptrPnt + i)->latlon.lat,
                                      (ptrPnt + i)->latlon.lon, utmxy);
      utmPoints.push_back(
          Point3d(utmxy.x, utmxy.y, ptrPnt->altitude, (ptrPnt + i)->id));
    }

    //��������
    std::vector<std::pair<int, int>> vxPair;

    pos += nCount * sizeof(GPSPoint);
    for (int position = pos; position < bufSize; position += 8) {
      int *pair = (int *)(buffer + position);
      vxPair.push_back(std::pair<int, int>(*pair, *(pair + 1)));
    }

    //�����
    GeoPolygonHole *pPolygon = (GeoPolygonHole *)pGeometry;

    if (vxPair.size() > 0) {
      int istart = vxPair[0].first;
      int iend = vxPair[0].second;
      for (int i = istart; i < iend; i++) {
        pPolygon->AppendVertex(utmPoints[i]);
      }

      for (int x = 1; x < vxPair.size(); x++) {
        GeoPolygon *pHole = new GeoPolygon();

        int istart = vxPair[x].first;
        int iend = vxPair[x].second;
        for (int i = istart; i < iend; i++) {
          pHole->AppendVertex(utmPoints[i]);
        }

        pPolygon->AppendHole(pHole);
      }
    }
  } else {
    for (int i = 0; i < nCount; i++) {
      UTMPoint utmxy;
      projectionUTM.LatLonToCartesian((ptrPnt + i)->latlon.lat,
                                      (ptrPnt + i)->latlon.lon, utmxy);
      pGeometry->AppendVertex(
          Point3d(utmxy.x, utmxy.y, ptrPnt->altitude, (ptrPnt + i)->id));
    }
  }

  pointSet = pGeometry;

  return true;
}

}  // namespace geditor
