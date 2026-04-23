
#pragma once

#include "core/camera.h"
#include "core/geo_polyline.h"
#include "core/layer.h"
#include "core/map_feature.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "core/state_set.h"
#include "core/vector_style.h"
#include "map/color_area_map.h"

#include <map>
#include <vector>

namespace geditor {

class RoadArea;

class VDBManage;

class GeoPolygon;

//车道线图层
class AreaLayer : public Layer {
 public:
  AreaLayer();

  virtual ~AreaLayer();

 public:
  //添加地图要素
  void AddMapFeature(MapFeature *feature);

  //删除地图要素
  bool DeleteMapFeature(MapFeature *feature);

  bool GetBoundary(V3d &vMin, V3d &vMax);

  //拾取对象
  bool PickupObject(const Point3d &mousePoint, double tolerance,
                    PickupResult &reslut);

  MapFeature *GetFeature(int idx) override;

  void GetAllMapFeature(std::vector<MapFeature *> &objects);
  std::vector<RoadArea *> GetAllRoadarea() { return m_LaneSegment; }

  bool CliperLayer(GeoPolyline *pClipLine);

  //清空图层
  void ClearLayer();

  void Save(VDBManage *vdb);

  void Read(VDBManage *vdb);

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
  std::map<Geometry *, PositionTransformNode *> m_RenderLeaf;

  //几何线条
  std::vector<RoadArea *> m_LaneSegment;

  ColorAreaMap m_colorAreaMap;
};

}  // namespace geditor
