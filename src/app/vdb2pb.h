#include <string>
#include <vector>

#include "app/attribute_calc.hpp"
#include "common_msgs/map_msgs/map.pb.h"
#include "common_msgs/map_msgs/map_area.pb.h"
#include "common_msgs/map_msgs/map_geometry.pb.h"
#include "common_msgs/map_msgs/map_junction.pb.h"
#include "common_msgs/map_msgs/map_lane.pb.h"
#include "common_msgs/map_msgs/map_overlap.pb.h"
#include "common_msgs/map_msgs/map_road.pb.h"
#include "core/boundary_layer.h"
#include "core/job_layer.h"
#include "core/lane_segment_layer.h"
#include "core/log.h"
#include "core/road_area_layer.h"
#include "core/sign_board_layer.h"

namespace geditor {
using PbLaneBoundary = map_engine::hdmap::LaneBoundary;
// vdb2pb.cc 会直接构造这些 protobuf 对象，因此这里要包含具体定义头，
// 不能只依赖 map.pb.h / map_lane.pb.h 里的前向声明。
using PbCurve = map_engine::hdmap::Curve;
using PbPolygon = map_engine::hdmap::Polygon;
using PbOverlap = map_engine::hdmap::Overlap;
using PbLane = map_engine::hdmap::Lane;
using PbJunction = map_engine::hdmap::Junction;
using PbArea = map_engine::hdmap::Area;
class VDB2PB {
 public:
  VDB2PB();
  ~VDB2PB();
  int Run(std::string filedb, std::string filepb, std::string version);
  int ReadVDB(std::string file);
  int ConvFeature2Pb(std::string path, std::string version);

 private:
  std::vector<Point3d> GetLaneInterpolateLane(const LaneSegment* l,
                                              double maxDist);
  int ExportLanePoints(std::string path, int lane_id = 0);
  int GetBouPoints(std::string lane_id, const BoundSegment* lane_left_bou,
                   const BoundSegment* lane_right_bou,
                   const BoundSegment* road_left_bou,
                   const BoundSegment* road_right_bou,
                   const BoundSegment* road_left_bou_forward,
                   bool is_left_road_bound_reverse,
                   std::vector<Point3d>& l_lane_points,
                   std::vector<Point3d>& r_lane_points,
                   std::vector<Point3d>& l_road_points,
                   std::vector<Point3d>& r_road_points,
                   std::vector<Point3d>& l_road_forward_points);
  double FillPbCurve(const std::vector<Point3d>& pts, PbCurve* curve);
  int FillPbPolygon(const std::vector<Point3d>& pts, PbPolygon* polygon);
  int FillPbBoundary(const std::vector<Point3d>& pts, PbLaneBoundary* boundary,
                     std::string from_lane_id);
  int CalcOverlapInfo(const std::vector<LaneSegment*>& lanes,
                      const std::vector<JobArea*>& areas,
                      std::unordered_map<std::string, PbOverlap>& pb_overlaps,
                      std::unordered_map<std::string, PbLane>& pb_lanes,
                      std::unordered_map<std::string, PbJunction>& pb_junctions,
                      std::unordered_map<std::string, PbArea>& pb_areas);
  int OutputPbFile(const std::string& pb_file_name,
                   const std::string& test_file_path);

 private:
  BoundaryLayer* layer_boundary_;
  SegmentLayer* layer_lane_;
  AreaLayer* layer_roadarea_;
  PointLayer* layer_sign_;
  JobLayer* layer_job_;
};

}  // namespace geditor
