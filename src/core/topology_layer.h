
#pragma once

#include "core/camera.h"
#include "core/geo_polyline.h"
#include "core/layer.h"
#include "core/point_element.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "core/state_set.h"
#include "core/vector_style.h"
#include "map/color_area_map.h"

#include <map>
#include <vector>

namespace geditor {

class SignBoard;

class VDBManage;

class SegmentLayer;

//车道线图层
class TopologyLayer : public Layer {
 public:
  TopologyLayer();

  virtual ~TopologyLayer();

 public:
  void AddMapFeature(MapFeature *feature);

  bool DeleteMapFeature(MapFeature *feature);

  bool PickupObject(const Point3d &mousePoint, double tolerance,
                    PickupResult &reslut);

  void ClearLayer();

  void Save(VDBManage *vdb);

  void Read(VDBManage *vdb);

  void AddTopology(int indx1, int indx2);

  void DeleteTopology(int indx1, int indx2);

 public:
  virtual void Cull(double minX, double minY, double maxX, double maxY);

  virtual void Update(const Matrix4x4f &svMatrix, Camera *pCamera);

  virtual void Draw(RenderInfo &rendinfo);

 private:
  Drawable *ConvertNodeDrawable(Geometry *pPolyline, const V3d &vCnt);

  Drawable *ConvertLineDrawable(Geometry *pPolyline, int select,
                                const V3d &vCnt);

  int OnPoint(Geometry *geometry, const Point3d &Q, double tolerance);

 private:
  int PackGeometry(Geometry *pPolyline, char *&pMem);

 private:
  //几何拓扑
  std::vector<std::pair<int, int>> path_topology_;
  std::map<int, PointElement *> path_points_;

  bool path_update_;

  //渲染对象
  PositionTransformNode *transform_node_;
};

}  // namespace geditor
