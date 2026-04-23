
#pragma once

#include <map>
#include <vector>

#include "core/camera.h"
#include "core/geo_polyline.h"
#include "core/job_area.h"
#include "core/layer.h"
#include "core/map_feature.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "core/state_set.h"
#include "core/vector_style.h"
#include "map/color_area_map.h"

namespace geditor {

class JobArea;

class VDBManage;

class BoundaryLayer;

//作业区图层
class JobLayer : public Layer {
 public:
  JobLayer();

  virtual ~JobLayer();

 public:
  MapFeature *GetFeature(int idx);
  void AddMapObject(std::vector<JobArea *> segmentArray);

  //添加地图要素
  void AddMapFeature(MapFeature *feature);

  //删除地图要素
  bool DeleteMapFeature(MapFeature *feature);

  bool GetBoundary(V3d &vMin, V3d &vMax);

  void GetAllMapFeature(std::vector<MapFeature *> &objects);

  //拾取对象
  bool PickupObject(const Point3d &mousePoint, double tolerance,
                    PickupResult &reslut);

  int GetAllJobArea(std::vector<JobArea *> &jobArea);

  //合并作业区
  void MergeObject(const std::vector<JobArea *> &jobArea);

  bool CliperLayer(GeoPolyline *pClipLine);

  //清空图层
  void ClearLayer();

  void Save(VDBManage *vdb);

  void Read(VDBManage *vdb, BoundaryLayer *boundary);

  void SetPointSize(double x);

  double GetPointSize() { return m_dPointSize; }

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
  std::vector<JobArea *> m_LaneSegment;

  ColorAreaMap m_colorAreaMap;

  double m_dPointSize;
};

}  // namespace geditor
