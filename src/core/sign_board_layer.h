
#pragma once

#include "core/camera.h"
#include "core/geo_polyline.h"
#include "core/layer.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "core/sign_board.h"
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
class PointLayer : public Layer {
 public:
  PointLayer();

  virtual ~PointLayer();

 public:
  void AddMapFeature(MapFeature *feature);

  bool DeleteMapFeature(MapFeature *feature);

  bool PickupObject(const Point3d &mousePoint, double tolerance,
                    PickupResult &reslut);
  MapFeature *GetFeature(int idx) override;

  void ClearLayer();

  void Save(VDBManage *vdb);

  void Read(VDBManage *vdb, SegmentLayer *pLayer);
  std::vector<SignBoard *> GetAllSign() { return m_LaneSegment; };

 public:
  virtual void Cull(double minX, double minY, double maxX, double maxY);

  virtual void Update(const Matrix4x4f &svMatrix, Camera *pCamera);

  virtual void Draw(RenderInfo &rendinfo);

 private:
  Drawable *ConvertNodeDrawable(Geometry *pPolyline, int select,
                                const V3d &vCnt, int areaType);

  int OnPoint(Geometry *geometry, const Point3d &Q, double tolerance);

 private:
  int PackGeometry(Geometry *pPolyline, char *&pMem);

 private:
  //渲染对象
  std::map<Geometry *, PositionTransformNode *> m_RenderLeaf;

  //几何线条
  std::vector<SignBoard *> m_LaneSegment;
};

}  // namespace geditor
