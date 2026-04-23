
#pragma once

#include <map>
#include <vector>

#include "core/bound_segment.h"
#include "core/camera.h"
#include "core/geo_arc_line.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_polyline.h"
#include "core/layer.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "core/state_set.h"
#include "core/vector_style.h"

namespace geditor {

class VDBManage;

struct Lane;

//车道线图层
class BoundaryLayer : public Layer {
 public:
  BoundaryLayer();

  virtual ~BoundaryLayer();

 public:
  //添加地图要素
  void AddMapFeature(MapFeature *feature);

  //删除地图要素
  bool DeleteMapFeature(MapFeature *polyline);

  bool GetBoundary(V3d &vMin, V3d &vMax);

  MapFeature *GetFeature(int idx) override;

  void GetBoundaryArea(std::vector<Geometry *> &boundaryArea);

  //拾取对象
  bool PickupObject(const Point3d &mousePoint, double tolerance,
                    PickupResult &result);

  Geometry *MergeGeometry(Geometry *pLine1, Geometry *pLine2);

  void MergeBoundary(std::vector<BoundSegment *> arraySegment);

  void BreakPolyline(MapFeature *pObject, int index, const Point3d &nearPnt3d);

  void Save(VDBManage *vdb);

  void Read(VDBManage *vdb);

  void AddMapObject(std::vector<Lane *> segmentArray);

  void ClearLayer();

  const std::vector<BoundSegment *> &GetAllBoundary() const {
    return lane_segment_;
  }

 public:
  virtual void Cull(double minX, double minY, double maxX, double maxY);

  virtual void Update(const Matrix4x4f &svMatrix, Camera *pCamera);

  virtual void Draw(RenderInfo &rendinfo);

 private:
  int OnPoint(Geometry *geometry, const Point3d &Q, double tolerance);

 private:
  int PackGeometry(Geometry *pPolyline, char *&pMem);

 private:
  //渲染对象
  std::map<Geometry *, PositionTransformNode *> render_leaf_;

  //几何线条
  std::vector<BoundSegment *> lane_segment_;

  //拓扑关系
  std::map<int, std::vector<SegmentNode *>> segment_node_;
};

}  // namespace geditor
