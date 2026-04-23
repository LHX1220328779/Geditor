
#pragma once

#include <cstring>

#include "core/geometry.h"

namespace geditor
{
  enum SegmentType
  {
    ENTER_SEGMENT = 1,
    EXIT_SEGMENT = 2,
  };

  class SegmentProperty
  {
  public:
    float speed;
    float length;
    int turnType;
    int laneType;
    int laneSeq;
    float lanePitch;
    int leftBoundary;
    int rightBoundary;
    int leftBorderType;
    int rightBorderType;
    // Stable business index assigned from mapping.txt.
    int mineSegmentIndex;
    // Stable business code used by planning to index the lane directly.
    char mineSegmentCode[64];
    // Legacy free-form name kept for backward compatibility.
    char name[32];
    // 路段横向均匀碾压位：0=存在某处边界与中心线横向距离<=阈值A；
    // 1=整条路段处处边界与中心线横向距离>阈值A (均匀碾压)。
    int road_right;
    // 行驶方向：0=未设定，1=上山，2=下山
    int direction;
    SegmentProperty()
        : leftBoundary(0),
          rightBoundary(0),
          laneType(0),
          laneSeq(0),
          mineSegmentIndex(0),
          road_right(0),
          direction(0)
    {
      memset(mineSegmentCode, 0, sizeof(mineSegmentCode));
      memset(name, 0, 32);
      // name[0] = 'm';
    }
  };

  class BoundaryProperty
  {
  public:
    float speed;
    float length;
    int turnType;
    int laneSeq = 0xffffff;
    float lanePitch;
    int leftBoundary;
    int rightBoundary;
    int leftBorderType;
    int rightBorderType;
    float ndtParam;
    int boundType;
    // Stable index from mapping.txt, mirrors SegmentProperty::mineSegmentIndex.
    int mineSegmentIndex;
    // Stable code from mapping.txt filename, mirrors SegmentProperty::mineSegmentCode.
    char mineSegmentCode[64];

    BoundaryProperty()
        : leftBoundary(0), rightBoundary(0), boundType(1), laneSeq(0),
          mineSegmentIndex(0)
    {
      memset(mineSegmentCode, 0, sizeof(mineSegmentCode));
    }
  };

  class JobProperty
  {
  public:
    int jobId;
    int areaType;
    int gpsParam;
    float proParam;
    int order;
    char name[32];

    JobProperty() : jobId(0), areaType(0), gpsParam(0), proParam(0.0f), order(0)
    {
      memset(name, 0, 32);
    }
  };

  class AreaProperty
  {
  public:
    int areaType;
    int order;
    char name[32];
    float proParam;
    int gpsParam;
  };

  class SignBoardProperty
  {
  public:
    int areaType;
    int stopline;
  };

  class ParallelSegment
  {
  public:
    int leftSegment;
    int rightSegment;
    int leftReverseSegment;
    int rightReverseSegment;
  };

  struct SegmentNode
  {
    int segment;
    SegmentType type;
  };

  class GeoPolyline : public Geometry
  {
  public:
    GeoPolyline();

    virtual ~GeoPolyline();

  public:
    void Hermite(std::vector<Point3d> &items) override;

    void MoveGeometry(double x, double y, double z);

    int OnPoint(const Point3d &Q, double tolerance);

    const void *GetDataPtr() const;

    const int GetDataSize() const;
  };

} // namespace geditor
