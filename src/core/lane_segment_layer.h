
#pragma once

#include <map>
#include <unordered_set>
#include <vector>

#include "core/geo_arc_line.h"
#include "core/geo_bezier_curve3.h"
#include "core/geo_bspline_curve3.h"
#include "core/geo_polyline.h"
#include "core/job_area.h"
#include "core/job_layer.h"
#include "core/lane_segment.h"
#include "core/layer.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "core/road_segment.h"
#include "core/state_set.h"
#include "core/vector_style.h"
#include "map/vdb_manage.h"

namespace geditor {

//车道线图层
class SegmentLayer : public Layer {
 public:
  SegmentLayer();

  virtual ~SegmentLayer();

 public:
  //添加地图要素
  void AddMapFeature(MapFeature *feature);

  //删除地图要素
  bool DeleteMapFeature(MapFeature *feature);

  //拾取对象
  bool PickupObject(const Point3d &mousePoint, double tolerance,
                    PickupResult &reslut);

  bool GetBoundary(V3d &minVec, V3d &maxVec);

  std::vector<LaneSegment *> GetAllLane() { return lane_segment_; }

  //清空图层
  void ClearLayer();

  void SetLineWidth(double x);

  double GetLineWidth() const { return line_width_; }

  void SetPointSize(double x);

  double GetPointSize() const { return point_size_; }

  void SetCarBodyCheck(bool bCheck);

  bool GetCarBodyCheck() const { return check_; }

  // road_right=1 的中心线以浅绿色区域填充代替默认浅白色
  void SetVizRoadRight(bool on);

  bool GetVizRoadRight() const { return viz_road_right_; }

  // 连通性校验后，将异常中心线的左右边界蒙版显示为浅黄色。
  // 每次调用都完整替换上一次结果；传入空集合即清除高亮。
  void SetConnectivityWarnings(const std::vector<int> &lane_ids);

  bool IsConnectivityWarning(int lane_id) const {
    return connectivity_warning_lane_ids_.count(lane_id) > 0;
  }

  // 设定行驶方向时：对 direction!=0 的中心线叠加半透明色带高亮
  // (上山 浅蓝 / 下山 黄色)
  void SetShowDirectionOverlay(bool on);

  bool GetShowDirectionOverlay() const { return show_direction_overlay_; }

  void BreakPolyline(MapFeature *pObject, int index, const Point3d &nearPnt3d);

  void SetParallelSegment(std::vector<LaneSegment *> selectSegment);
  void SetReverseSegment(std::vector<LaneSegment *> selectSegment);

  void GenerateLaneTopology(LaneSegment *lane = nullptr);
  void GenerateJobTopology(std::vector<JobArea *> &jobs);
  void GenerateTopology();

  void GenerateSegmentNodeID(
      const std::vector<LaneSegment *> &laneSegment,
      std::map<int, std::vector<SegmentNode *>> &segmentNode);

  void GenerateRoadNodeID(
      const std::vector<RoadSegment *> &laneSegment,
      std::map<int, std::vector<SegmentNode *>> &segmentNode);

  void MergeObject(std::vector<LaneSegment *> selectSegment);

  LaneSegment *GetAllParallelSegment(LaneSegment *pSegment,
                                     std::vector<LaneSegment *> &laneSegment);

  int GetAllSegment(std::vector<LaneSegment *> &laneSegment);

  RoadSegment *FindRoadSegment(int linkId);

  MapFeature *GetFeature(int idx) override;

  void Save(VDBManage *vdb);

  void Read(VDBManage *vdb);

  void AddMapObject(std::vector<Lane *> segmentArray);

  void SetLayers(LayerHelper layers) { layers_ = layers; }

  void CheckDeletedRealtion();

 public:
  virtual void Cull(double minX, double minY, double maxX, double maxY);

  virtual void Update(const Matrix4x4f &viewMatrix, Camera *pCamera);

  virtual void Draw(RenderInfo &rendinfo);

 private:
  void GenerateSegmentRelation(std::vector<LaneSegment *> &laneSegment,
                               LaneSegment *lane = nullptr);

  void GenerateRoadRelation(std::vector<LaneSegment *> &laneSegment);

  int OnPoint(Geometry *geometry, const Point3d &Q, double tolerance);

  void OverlapLanesVertex();

  Geometry *MergeGeometry(Geometry *pLine1, Geometry *pLine2);

 private:
  int PackGeometry(Geometry *pPolyline, char *&pMem);

 private:
  //渲染对象
  std::map<Geometry *, PositionTransformNode *> render_leaf_;

  //车道
  std::vector<LaneSegment *> lane_segment_;

  //道路
  std::vector<RoadSegment *> road_link_;

  bool check_;
  bool viz_road_right_ = false;
  std::unordered_set<int> connectivity_warning_lane_ids_;
  bool show_direction_overlay_ = false;
  double line_width_;
  double point_size_;

  LayerHelper layers_;
};

}  // namespace geditor
