#include "vdb2pb.h"

#include <fcntl.h>

#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#include "pbfileopra.hpp"

std::string CreateOverlapId() {
  static int count = 0;
  ++count;
  return ("overlap_" + std::to_string(count));
}

namespace fs = std::filesystem;
namespace geditor {
using PbHeader = map_engine::hdmap::Header;
using PbRoad = map_engine::hdmap::Road;
using PbMap = map_engine::hdmap::Map;
using PbLineSegment = map_engine::hdmap::LineSegment;
using PbCurveSegment = map_engine::hdmap::CurveSegment;
using PbLaneBoundaryType = map_engine::hdmap::LaneBoundaryType;
using PbID = map_engine::hdmap::Id;
using PbBoundaryEdge = map_engine::hdmap::BoundaryEdge;
using PbObjectOverlapInfo = map_engine::hdmap::ObjectOverlapInfo;
using PbArea = map_engine::hdmap::Area;

const double DefaultLaneWidth = 3.5;  // 默认车道宽度

VDB2PB::VDB2PB() {
  layer_roadarea_ = new AreaLayer();
  layer_lane_ = new SegmentLayer();
  layer_sign_ = new PointLayer();
  layer_boundary_ = new BoundaryLayer();
  layer_job_ = new JobLayer();
  LayerHelper layer_helper;
  layer_helper.SetLayer(layer_roadarea_, LT_ROADAREA);
  layer_helper.SetLayer(layer_lane_, LT_LANE);
  layer_helper.SetLayer(layer_sign_, LT_SIGN);
  layer_helper.SetLayer(layer_boundary_, LT_BOUNDARY);
  layer_helper.SetLayer(layer_job_, LT_FUNAREA);
  layer_lane_->SetLayers(layer_helper);
}
VDB2PB::~VDB2PB() {
  delete layer_boundary_;
  delete layer_lane_;
  delete layer_roadarea_;
  delete layer_sign_;
  delete layer_job_;
}
int VDB2PB::Run(std::string filedb, std::string filepb, std::string version) {
  RETURN_VAL_IF(ReadVDB(filedb) < 0, -1);
  RETURN_VAL_IF(ConvFeature2Pb(filepb, version) < 0, -1);
  return 0;
}
int VDB2PB::ReadVDB(std::string file) {
  VDBManage* vdb = new VDBManage();
  if (vdb->Create(file.c_str())) {
    layer_boundary_->Read(vdb);
    layer_roadarea_->Read(vdb);
    layer_sign_->Read(vdb, layer_lane_);
    layer_job_->Read(vdb, layer_boundary_);
    layer_lane_->Read(vdb);
    layer_lane_->GenerateTopology();

    vdb->Close();
    delete vdb;
    return 0;
  }
  delete vdb;
  return -1;
}
int VDB2PB::ConvFeature2Pb(std::string path, std::string version) {
  auto p_seg = path + "/" + version;
  file_opra::ReCreateDirectory(p_seg);
  auto lanes = layer_lane_->GetAllLane();
  // ExportLanePoints(path);  // 测试code，输出车道原始坐标

  std::unordered_map<std::string, PbLane> pb_lanes;
  std::unordered_map<std::string, std::string> lane_id2road_id;
  std::unordered_map<std::string, std::vector<std::string>> road_id2lane_ids;
  std::unordered_map<std::string, bool> road_id2has_reverse;
  int road_cnt = 1;
  for (auto lane : lanes) {
    // if (lane->GetUniqueID() == 352104879) {
    //   int test = 9999;
    // }
    // 测试代码
    // std::cout << "lane id=" << lane->GetUniqueID() << std::endl;
    // if(lane->GetUniqueID() == 351856514) {
    //   std::vector<LaneSegment*> paralle_lanesTTTTTTT;
    //   auto leftmost = layer_lane_->GetAllParallelSegment(lane,
    //   paralle_lanesTTTTTTT); for (int i = 0;i <
    //   paralle_lanesTTTTTTT.size(); ++i) {
    //     std::cout << "paralle_lane id[" << i << "]" <<
    //     paralle_lanesTTTTTTT[i]->GetUniqueID() <<std::endl;
    //   }
    // } else {
    //   continue;
    // }
    // return 0;

    auto seg_property = lane->GetProperty();
     if (seg_property->laneType != 0) {  // 过滤掉参考线类型
      LOG(INFO) << "lane id=" << lane->GetUniqueID() << " is reference line";
      continue;
    }

    PbLane pb_lane;
    std::string lane_id = "lane_" + std::to_string(lane->GetUniqueID());
    // 赋值车道ID，道路类型，车道类型，车道方向，车道限速，车道长度
    pb_lane.mutable_id()->set_id(lane_id);
    if (seg_property->mineSegmentIndex > 0) {
      pb_lane.set_mine_segment_index(seg_property->mineSegmentIndex);
    }
    if (strlen(seg_property->mineSegmentCode) > 0) {
      pb_lane.set_mine_segment_code(seg_property->mineSegmentCode);
    }
    auto lanetype = PbLane::CITY_DRIVING;
    if (seg_property->laneType == 11) {
      lanetype = PbLane::ENTRY_REFERENCE_LINE;
    } else if (seg_property->laneType == 12) {
      lanetype = PbLane::BACK_REFERENCE_LINE;
    } else if (seg_property->laneType == 13) {
      lanetype = PbLane::EXIT_REFERENCE_LINE;
    } else {
      lanetype = PbLane::CITY_DRIVING;
    }
    pb_lane.set_type(lanetype);
    pb_lane.set_turn(PbLane::NO_TURN);
    pb_lane.set_direction(PbLane::FORWARD);
    pb_lane.set_speed_limit(attribute_calc::kmph2mps(seg_property->speed));
    pb_lane.set_length(seg_property->length);

    auto plist = GetLaneInterpolateLane(lane, 1000);  // 插值车道点，1m插值
    // 赋值车道线几何形状点
    auto lane_curve = pb_lane.mutable_central_curve();
    FillPbCurve(plist, lane_curve);
    // 左右临接车道
    auto* pNeighbors = lane->GetParallelSegment();
    if (pNeighbors->leftSegment > 0) {
      PbID* left_neighbor_forward_lane_id =
          pb_lane.add_left_neighbor_forward_lane_id();
      left_neighbor_forward_lane_id->set_id(
          "lane_" + std::to_string(pNeighbors->leftSegment));
    }
    if (pNeighbors->rightSegment > 0) {
      PbID* right_neighbor_forward_lane_id =
          pb_lane.add_right_neighbor_forward_lane_id();
      right_neighbor_forward_lane_id->set_id(
          "lane_" + std::to_string(pNeighbors->rightSegment));
    }
    if (pNeighbors->leftReverseSegment > 0) {
      PbID* left_neighbor_reverse_lane_id =
          pb_lane.add_left_neighbor_reverse_lane_id();
      left_neighbor_reverse_lane_id->set_id(
          "lane_" + std::to_string(pNeighbors->leftReverseSegment));
    }
    if (pNeighbors->rightReverseSegment > 0) {
      PbID* right_neighbor_reverse_lane_id =
          pb_lane.add_right_neighbor_reverse_lane_id();
      right_neighbor_reverse_lane_id->set_id(
          "lane_" + std::to_string(pNeighbors->rightReverseSegment));
    }
    // 赋值后继车道ID
    std::vector<int> succ_segArray;
    lane->GetSuccessorSegment(succ_segArray);
    for (auto succ_seg : succ_segArray) {
      auto succ_lane = (LaneSegment*)(layer_lane_->GetFeature(succ_seg));
      if (succ_lane->GetProperty()->laneType != 0) {  // 过滤掉参考线类型
        LOG(INFO) << "lane id=" << lane->GetUniqueID()
                  << " successor lane id=" << succ_seg
                  << " is reference line";
        continue;
      }
      PbID* s_lane_id = pb_lane.add_successor_id();
      s_lane_id->set_id("lane_" + std::to_string(succ_seg));
    }
    // 赋值前驱车道ID
    std::vector<int> pred_segArray;
    lane->GetPredecessorSegment(pred_segArray);
    for (auto pred_seg : pred_segArray) {
      auto pred_lane = (LaneSegment*)(layer_lane_->GetFeature(pred_seg));
      if (pred_lane->GetProperty()->laneType != 0) {  // 过滤掉参考线类型
        LOG(INFO) << "lane id=" << lane->GetUniqueID()
                  << " predecessor lane id=" << pred_seg
                  << " is reference line";
        continue;
      }
      PbID* p_lane_id = pb_lane.add_predecessor_id();
      p_lane_id->set_id("lane_" + std::to_string(pred_seg));
    }
    // 计算车道边线和道路边线
    std::vector<LaneSegment*> paralle_lanes;
    auto leftmost = layer_lane_->GetAllParallelSegment(lane, paralle_lanes);
    auto rightmost = lane->GetParallelSegment()->rightSegment > 0
                         ? paralle_lanes.back()
                         : lane;
    // std::cout << "laneid = " << lane->GetUniqueID()
    //           << ";name = " << lane->GetProperty()->name << std::endl;
    // std::cout << "rightmost = " << rightmost->GetUniqueID()
    //           << ";name = " << rightmost->GetProperty()->name << std::endl;
    auto road_right_bou = rightmost->GetRightBoundary();
    auto road_left_bou = leftmost->GetLeftBoundary();
    auto road_left_bou_forward = road_left_bou;
    // bool is_left_road_bound_reverse = false;
    bool has_left_reverse = false;
    // 此处如果最左侧车道还有对向车道关系，则需要找到对向车道的最右侧车道的右边界
    if (leftmost->GetParallelSegment()->leftReverseSegment > 0) {
      int id = leftmost->GetParallelSegment()->leftReverseSegment;
      auto reverse_lane = (LaneSegment*)(layer_lane_->GetFeature(id));
      std::vector<LaneSegment*> reverse_parallel_lanes;
      layer_lane_->GetAllParallelSegment(reverse_lane, reverse_parallel_lanes);
      auto reverse_most = reverse_lane->GetParallelSegment()->rightSegment > 0
                              ? reverse_parallel_lanes.back()
                              : reverse_lane;
      road_left_bou = reverse_most->GetRightBoundary();
      has_left_reverse = true;
      // std::cout << "reverse_most = " << reverse_most->GetUniqueID()
      //           << ";name = " << reverse_most->GetProperty()->name
      //           << std::endl;
    }
    // else {
    //   std::cout << "no reverse_most" << std::endl;
    // }
    auto lane_left_bou = lane->GetLeftBoundary();
    auto lane_right_bou = lane->GetRightBoundary();

    if (lane_left_bou && lane_right_bou) {
      std::vector<Point3d> l_lane_points;
      std::vector<Point3d> r_lane_points;
      std::vector<Point3d> l_road_points;
      std::vector<Point3d> r_road_points;
      std::vector<Point3d> l_road_forward_points;
      GetBouPoints(lane_id, lane_left_bou, lane_right_bou, road_left_bou,
                   road_right_bou, road_left_bou_forward, has_left_reverse,
                   l_lane_points, r_lane_points, l_road_points, r_road_points,
                   l_road_forward_points);
      // 赋值车道boundary信息
      auto lane_left_boudary = pb_lane.mutable_left_boundary();
      FillPbBoundary(l_lane_points, lane_left_boudary, lane_id);

      auto lane_right_boudary = pb_lane.mutable_right_boundary();
      FillPbBoundary(r_lane_points, lane_right_boudary, lane_id);

      // 计算车道左边界偏移
      auto lane_left_offsets = attribute_calc::calculateLateralOffsets(
          lane->GetUniqueID(), plist, l_lane_points, true);
      // 计算车道右边界偏移
      auto lane_right_offsets = attribute_calc::calculateLateralOffsets(
          lane->GetUniqueID(), plist, r_lane_points, false);
      // 计算同向道路左边界偏移
      auto road_left_offsets = attribute_calc::calculateLateralOffsets(
          lane->GetUniqueID(), plist, l_road_forward_points, true);
      // 计算同向道路右边界偏移
      auto road_right_offsets = attribute_calc::calculateLateralOffsets(
          lane->GetUniqueID(), plist, r_road_points, false);

      double s = 0.0;
      Point3d pre_pt;
      for (size_t i = 0; i < plist.size(); ++i) {
        if (i > 0) {
          s += attribute_calc::distance(plist[i], pre_pt);
        }
        auto pl = pb_lane.add_left_sample();
        pl->set_s(s);
        pl->set_width(lane_left_offsets[i]);
        auto pr = pb_lane.add_right_sample();
        pr->set_s(s);
        pr->set_width(lane_right_offsets[i]);

        auto pl_road = pb_lane.add_left_road_sample();
        pl_road->set_s(s);
        pl_road->set_width(road_left_offsets[i]);
        auto pr_road = pb_lane.add_right_road_sample();
        pr_road->set_s(s);
        pr_road->set_width(road_right_offsets[i]);
        pre_pt = plist[i];
      }
    } else {
      if (lanetype == PbLane::CITY_DRIVING) {
        LOG(INFO) << "lane id=" << lane->GetUniqueID()
                  << " lane type=" << seg_property->laneType
                  << " has no left or right boundary";
      }
      double s = 0.0;
      Point3d pre_pt;
      for (size_t i = 0; i < plist.size(); ++i) {
        if (i > 0) {
          s += attribute_calc::distance(plist[i], pre_pt);
        }
        auto pl = pb_lane.add_left_sample();
        pl->set_s(s);
        pl->set_width(DefaultLaneWidth);
        auto pr = pb_lane.add_right_sample();
        pr->set_s(s);
        pr->set_width(DefaultLaneWidth);

        auto pl_road = pb_lane.add_left_road_sample();
        pl_road->set_s(s);
        pl_road->set_width(DefaultLaneWidth);
        auto pr_road = pb_lane.add_right_road_sample();
        pr_road->set_s(s);
        pr_road->set_width(DefaultLaneWidth);
        pre_pt = plist[i];
      }
    }

    // 输出结果
    // std::cout << "Center Point | Left Offset | Right Offset" << std::endl;
    // for (size_t i = 0; i < central_line.size(); ++i) {
    //     printf("(%.1f, %.1f) | %.2fm | %.2fm\n",
    //           central_line[i].x, central_line[i].y,
    //           left_offsets[i], right_offsets[i]);
    // }
    pb_lanes[lane_id] = pb_lane;
    if (!lane_id2road_id.count(lane_id)) {
      // 没有生成道路ID，此时需要新建一个roadID，同时将平行车道一起关联到road
      std::string road_id = "road_" + std::to_string(road_cnt++);
      road_id2has_reverse[road_id] = false;

      if (has_left_reverse) {
        road_id2has_reverse[road_id] = true;
        // 先将反向的车道从右到左加入road
        std::vector<LaneSegment*> reverse_paralle_lanes;
        int left_rev_id = leftmost->GetParallelSegment()->leftReverseSegment;
        auto reverse_lane =
            (LaneSegment*)(layer_lane_->GetFeature(left_rev_id));
        layer_lane_->GetAllParallelSegment(reverse_lane, reverse_paralle_lanes);
        if (reverse_paralle_lanes.size() > 0) {  // 有多条，从右到左加入
          for (int i = reverse_paralle_lanes.size() - 1; i >= 0; --i) {
            auto revid = reverse_paralle_lanes[i]->GetUniqueID();
            std::string id = "lane_" + std::to_string(revid);
            lane_id2road_id[id] = road_id;
            road_id2lane_ids[road_id].push_back(id);
          }
        } else {  // 只有一条反向车道，直接加入
          std::string id =
              "lane_" + std::to_string(reverse_lane->GetUniqueID());
          lane_id2road_id[id] = road_id;
          road_id2lane_ids[road_id].push_back(id);
        }
      }

      if (lane->GetParallelSegment()->leftSegment > 0) {
        auto left_id = lane->GetParallelSegment()->leftSegment;
        auto left_lane = (LaneSegment*)(layer_lane_->GetFeature(left_id));
        if (left_lane->GetParallelSegment()->leftReverseSegment >
            0) {  // 存在左侧反向车道
          road_id2has_reverse[road_id] = true;
        }
        std::string id = "lane_" + std::to_string(left_id);
        lane_id2road_id[id] = road_id;
        road_id2lane_ids[road_id].push_back(id);
      }
      lane_id2road_id[lane_id] = road_id;
      road_id2lane_ids[road_id].push_back(lane_id);
      if (lane->GetParallelSegment()->rightSegment > 0) {
        std::string id =
            "lane_" + std::to_string(lane->GetParallelSegment()->rightSegment);
        lane_id2road_id[id] = road_id;
        road_id2lane_ids[road_id].push_back(id);
      }
    }
  }

  std::unordered_map<std::string, PbRoad> pb_roads;
  for (auto& rec_pair : road_id2lane_ids) {
    auto road_id = rec_pair.first;
    auto lane_ids = rec_pair.second;
    PbRoad pb_road;
    pb_road.mutable_id()->set_id(road_id);
    pb_road.set_type(PbRoad::UNKNOWN);
    auto* section = pb_road.add_section();
    section->mutable_id()->set_id("section_1");
    for (auto& lane_id : lane_ids) {
      PbID* id = section->add_lane_id();
      id->set_id(lane_id);
    }
    auto lane_type = pb_lanes[lane_ids[0]].type();

    auto road_bou = section->mutable_boundary();
    // left boudary
    auto l_lane = pb_lanes[lane_ids[0]];
    if (l_lane.has_right_boundary() && !l_lane.has_left_boundary()) {
      auto* l_edge = road_bou->mutable_outer_polygon()->add_edge();
      if (road_id2has_reverse[road_id]) {  // 存在反向车道
        l_edge->mutable_curve()->CopyFrom(l_lane.right_boundary().curve());
      } else {
        l_edge->mutable_curve()->CopyFrom(l_lane.left_boundary().curve());
      }
      l_edge->set_type(PbBoundaryEdge::LEFT_BOUNDARY);
    }
    // right boudary
    auto r_lane = pb_lanes[lane_ids.back()];
    if (r_lane.has_right_boundary()) {
      auto* r_edge = road_bou->mutable_outer_polygon()->add_edge();
      r_edge->mutable_curve()->CopyFrom(r_lane.right_boundary().curve());
      r_edge->set_type(PbBoundaryEdge::RIGHT_BOUNDARY);
    }

    pb_roads[road_id] = pb_road;
  }

  std::unordered_map<std::string, PbJunction> pb_junctions;
  std::unordered_map<std::string, PbArea> pb_areas;
  std::vector<JobArea*> jobArea;
  int count = layer_job_->GetAllJobArea(jobArea);
  LOG(INFO) << "job area count: " << count;
  for (auto& job : jobArea) {
    auto jobid = job->GetUniqueID();
    auto pro = job->GetProperty();
    /*
     {"铲装区", 0}, {"破碎站", 1},        {"排土区", 2},
    {"路口", 3},   {"停车区", 4},        {"加油区", 5},
    {"检修区", 6}, {"功能点", jobpoint}, {"停车位", 101}};
    */
    PbArea::Type type;
    bool bFlag = true;
    switch (pro->areaType) {
      case 3:  // 路口
      {
        std::string junc_id = "junction_" + std::to_string(jobid);
        PbJunction pb_junc;
        pb_junc.mutable_id()->set_id(junc_id);
        pb_junc.set_type(PbJunction::CROSS_ROAD);
        std::vector<Point3d> pts;
        job->GetGeometry()->Hermite(pts);
        FillPbPolygon(pts, pb_junc.mutable_polygon());
        pb_junctions[junc_id] = pb_junc;
      } break;
      case 0:  // 铲装区
        type = PbArea::SHOVEL_LOADING_AREA;
        break;
      case 1:  // 破碎站
        type = PbArea::CRUSHING_STATION;
        break;
      case 2:  // 排土区
        type = PbArea::DISPOSAL_AREA;
        break;
      case 4:  // 停车区
        type = PbArea::PARKING_AREA;
        break;
      case 5:  // 加油区->改为障碍区
        // type = PbArea::GAS_STATION;
        type = PbArea::OBSTACLE_AREA;
        break;
      case 6:  // 检修区->改为可行驶区
        // type = PbArea::MAINTENANCE_AREA;
        type = PbArea::FREE_SPACE_AREA;
        break;
      case 100:  // 功能点
        type = PbArea::JOP_POINT;
        break;
      case 101:  // 停车位
        type = PbArea::PARKING_SPACE;
        break;
      default:
        bFlag = false;
        LOG(INFO) << "unknow area type: " << pro->areaType;
        break;
    }
    if (pro->areaType != 3 && bFlag) {
      std::string area_id = "area_" + std::to_string(jobid);
      PbArea pb_area;
      pb_area.mutable_id()->set_id(area_id);
      pb_area.set_type(type);
      std::vector<Point3d> pts;
      job->GetGeometry()->Hermite(pts);
      FillPbPolygon(pts, pb_area.mutable_polygon());
      pb_areas[area_id] = pb_area;
    }
  }
  std::unordered_map<std::string, PbOverlap> pb_overlaps;
  CalcOverlapInfo(lanes, jobArea, pb_overlaps, pb_lanes, pb_junctions, pb_areas);

  PbMap map;
  PbHeader* map_header = map.mutable_header();
  map_header->set_version(version);
  // 获取当前系统时间的时间戳
  auto currentTimeStamp = std::chrono::system_clock::now();
  // 转换为当前时区的时间
  std::time_t currentTime =
      std::chrono::system_clock::to_time_t(currentTimeStamp);
  // 将时间转换为struct tm结构体
  struct std::tm* currentTimeInfo = std::localtime(&currentTime);
  std::ostringstream oss;
  oss << std::put_time(currentTimeInfo, "%Y-%m-%d %H:%M:%S");
  map_header->set_date(oss.str());
  map_header->mutable_projection()->set_proj(
      "+proj=utm +zone=50 +ellps=WGS84 +datum=WGS84 +units=m +no_defs");

  for (auto& road_pair : pb_roads) {
    *(map.add_road()) = road_pair.second;
  }
  for (auto& lane_pair : pb_lanes) {
    *(map.add_lane()) = lane_pair.second;
  }
  for (auto& junc_pair : pb_junctions) {
    *(map.add_junction()) = junc_pair.second;
  }
  for (auto& area_pair : pb_areas) {
    *(map.add_ad_area()) = area_pair.second;
  }
  for (auto& overlap_pair : pb_overlaps) {
    *(map.add_overlap()) = overlap_pair.second;
  }
  std::ofstream output(p_seg + "/map.bin",
                       std::ios::out | std::ios::trunc | std::ios::binary);
  if (!output) {
    std::cerr << "无法打开文件 " << p_seg << "/map.bin 进行写入" << std::endl;
    return 1;
  }
  if (!map.SerializeToOstream(&output)) {
    std::cerr << "序列化失败" << std::endl;
    return false;
  }
  file_opra::SetProtoToASCIIFile(map, p_seg + "/map.txt");
  return 0;
}

std::vector<Point3d> VDB2PB::GetLaneInterpolateLane(const LaneSegment* l,
                                                    double maxDist) {
  auto& geo = *l->GetGeometry();
  // size_t size = geo.GetVertexCount();
  std::vector<Point3d> raw_plist;
  // raw_plist.reserve(size);
  // for (size_t i = 0; i < size; ++i) {
  //   raw_plist.emplace_back(geo.GetVertex(i));
  // }
  geo.Hermite(raw_plist);
  return attribute_calc::InterpolateLane(raw_plist, maxDist);
}
int VDB2PB::ExportLanePoints(std::string path, int lane_id) {
  // 打印车道原始坐标信息
  auto lanes = layer_lane_->GetAllLane();
  for (auto lane : lanes) {
    if (lane_id > 0) {
      if (lane->GetUniqueID() != lane_id) {
        continue;
      }
    }
    auto filename = path + "/raw_" + lane->GetProperty()->name + ".csv";
    LINFO << filename;
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to create file: " << filename << std::endl;
      return 1;
    }
    auto& lane_geo = *lane->GetGeometry();
    std::vector<Point3d> pts;
    lane_geo.Hermite(pts);
    for (const auto& vertex : pts) {
      file << std::fixed << std::setprecision(6) << vertex.x << "," << vertex.y
           << std::endl;
    }
    // size_t lane_size = lane_geo.GetVertexCount();
    // for (size_t i = 0; i < lane_size; i++) {
    //   const auto& vertex = lane_geo.GetVertex(i);
    //   file << std::fixed << std::setprecision(6) << vertex.x << "," <<
    //   vertex.y
    //        << std::endl;
    // }
    file.close();
    if (lane_id > 0) {
      break;
    }
  }
  return 0;
}
int VDB2PB::GetBouPoints(
    std::string lane_id, const BoundSegment* lane_left_bou,
    const BoundSegment* lane_right_bou, const BoundSegment* road_left_bou,
    const BoundSegment* road_right_bou,
    const BoundSegment* road_left_bou_forward, bool is_left_road_bound_reverse,
    std::vector<Point3d>& l_lane_points, std::vector<Point3d>& r_lane_points,
    std::vector<Point3d>& l_road_points, std::vector<Point3d>& r_road_points,
    std::vector<Point3d>& l_road_forward_points) {
  if (lane_left_bou == nullptr || lane_right_bou == nullptr ||
      road_left_bou == nullptr || road_right_bou == nullptr ||
      road_left_bou_forward == nullptr) {
    LOG(ERROR) << "GetBouPoints: lane_id=" << lane_id
               << ",lane_left_bou or lane_right_bou or "
                  "road_left_bou or road_right_bou is nullptr";
    return 1;
  }
  auto& l_lane_geo = *lane_left_bou->GetGeometry();
  l_lane_geo.Hermite(l_lane_points);
  auto& r_lane_geo = *lane_right_bou->GetGeometry();
  r_lane_geo.Hermite(r_lane_points);
  auto& r_road_geo = *road_right_bou->GetGeometry();
  r_road_geo.Hermite(r_road_points);
  auto& l_road_geo = *road_left_bou->GetGeometry();
  l_road_geo.Hermite(l_road_points);
  if (is_left_road_bound_reverse) {
    std::reverse(l_road_points.begin(), l_road_points.end());
  }
  auto& l_road_forward_geo = *road_left_bou_forward->GetGeometry();
  l_road_forward_geo.Hermite(l_road_forward_points);
  return 0;
}
int VDB2PB::FillPbPolygon(const std::vector<Point3d>& pts, PbPolygon* polygon) {
  for (unsigned i = 0; i < pts.size(); i++) {
    auto pt = polygon->add_point();
    pt->set_x(pts[i].x);
    pt->set_y(pts[i].y);
  }
  return 0;
}
double VDB2PB::FillPbCurve(const std::vector<Point3d>& pts, PbCurve* curve) {
  double s = 0.0;
  // for (unsigned i = 0; i < pts.size() - 1; i++) {
  //   PbCurveSegment* curve_segment = curve->add_segment();
  //   curve_segment->set_s(s);
  //   auto p = curve_segment->mutable_start_position();
  //   p->set_x(pts[i].x);
  //   p->set_y(pts[i].y);
  //   double length = attribute_calc::distance2d(pts[i], pts[i + 1]);
  //   curve_segment->set_length(length);
  //   double heading = attribute_calc::calculateHeading(pts[i], pts[i + 1]);
  //   curve_segment->set_heading(heading);
  //   s += length;
  // }
  PbCurveSegment* curve_segment = curve->add_segment();
  curve_segment->set_s(s);
  if (pts.size() > 1) {
    auto p = curve_segment->mutable_start_position();

    // 输出经过原点转换的坐标
    ProjectionUTM utm;
    LatLon ll;
    utm.CartesianToLatLon(pts[0].x, pts[0].y, ProjectionUTM::zone, false, ll);
    Point3d pp = attribute_calc::TransGps2Pt(ll.lon, ll.lat, pts[0].z);
    p->set_x(pp.x);
    p->set_y(pp.y);

    // 原始输出
    // p->set_x(pts[0].x);
    // p->set_y(pts[0].y);
    double heading = attribute_calc::calculateHeading(pts[0], pts[1]);
    curve_segment->set_heading(heading);
  }
  auto line = curve_segment->mutable_line_segment();
  double length = 0;
  for (unsigned i = 0; i < pts.size(); i++) {
    auto line_pt = line->add_point();
    
    // 输出经过原点转换的坐标
    ProjectionUTM utm;
    LatLon ll;
    utm.CartesianToLatLon(pts[i].x, pts[i].y, ProjectionUTM::zone, false, ll);
    Point3d pp = attribute_calc::TransGps2Pt(ll.lon, ll.lat, pts[i].z);
    line_pt->set_x(pp.x);
    line_pt->set_y(pp.y);

    // 原始输出
    // line_pt->set_x(pts[i].x);
    // line_pt->set_y(pts[i].y);
    if (i > 0) {
      length += attribute_calc::distance2d(pts[i], pts[i - 1]);
    }
  }
  curve_segment->set_length(length);
  return length;
}
int VDB2PB::FillPbBoundary(const std::vector<Point3d>& pts,
                           PbLaneBoundary* boundary, std::string from_lane_id) {
  boundary->set_virtual_(true);
  auto b_type = boundary->add_boundary_type();
  b_type->set_s(0.0);
  b_type->add_types(PbLaneBoundaryType::DOTTED_WHITE);
  double l_lane_b_length = FillPbCurve(pts, boundary->mutable_curve());
  boundary->set_length(l_lane_b_length);
  boundary->mutable_from_lane_id()->set_id(from_lane_id);
  return 0;
}

int VDB2PB::OutputPbFile(const std::string& pb_file_name,
                         const std::string& test_file_path) {
  PbMap map;
  std::fstream input(pb_file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    std::cerr << "Failed to open file " << pb_file_name << " in binary mode.";
    return -1;
  }
  if (!map.ParseFromIstream(&input)) {
    std::cerr << "Failed to parse file " << pb_file_name << " as binary proto.";
    return -2;
  }
  // output lane
  int idx = 0;
  for (const auto& lane : map.lane()) {
    std::ofstream file(test_file_path + "/" + lane.id().id() + ".txt");
    if (!file.is_open()) {
      std::cerr << "Failed to open file " << test_file_path << lane.id().id()
                << ".txt for writing.";
      return -3;
    }
    file << "x,y,s,l_width,r_width,id,type,length" << std::endl;
    file << "id " << lane.id().id() << "\n";
    const auto& cen_curve = lane.central_curve();
    for (const auto& curve : cen_curve.segment()) {
      if (curve.has_line_segment()) {
        int pt_idx = 0;
        for (const auto& point : curve.line_segment().point()) {
          file << point.x() << "," << point.y() << ",";
          file << lane.left_sample()[pt_idx].s() << ",";
          file << lane.left_sample()[pt_idx].width() << ",";
          file << lane.right_sample()[pt_idx].width() << ",";
        }
      } else {
        std::cerr << "Can not handle curve line_segment.";
      }
    }
    if (idx == 0) {
      file << lane.id().id() << "," << lane.type() << "," << lane.length();
    }
    idx++;
  }
  return 0;
}
int VDB2PB::CalcOverlapInfo(
    const std::vector<LaneSegment*>& lanes,
    const std::vector<JobArea*>& areas,
    std::unordered_map<std::string, PbOverlap>& pb_overlaps,
    std::unordered_map<std::string, PbLane>& pb_lanes,
    std::unordered_map<std::string, PbJunction>& pb_junctions,
    std::unordered_map<std::string, PbArea>& pb_areas) {
  for (const auto& lane : lanes) {    // 遍历所有车道计算压盖信息
    auto seg_property = lane->GetProperty();
     if (seg_property->laneType != 0) {  // 过滤掉参考线类型
      LOG(INFO) << "lane id=" << lane->GetUniqueID() << " is reference line";
      continue;
    }
    auto lane_id = "lane_" + std::to_string(lane->GetUniqueID());
    auto suc_object = lane->GetSuccessorFeature();
    auto pre_object = lane->GetPredecessorFeature();

    if (suc_object) {
      if (suc_object->GetType() != MapFeature::MFT_JOB_AREA) {
        LOG(INFO) << "lane id: " << lane_id
                  << " SuccessorFeature type: " << suc_object->GetType()
                  << " not MFT_JOB_AREA!";
        suc_object = nullptr;
      }
    }
    if (pre_object) {
      if (pre_object->GetType() != MapFeature::MFT_JOB_AREA) {
        LOG(INFO) << "lane id: " << lane_id
                  << " PredecessorFeature type: " << pre_object->GetType()
                  << " not MFT_JOB_AREA!";
        pre_object = nullptr;
      }
    }
    bool calc_finished = false;
    if (suc_object && pre_object) {  // 前序后继都找到压盖区域
      JobArea* p_suc = (JobArea*)suc_object;
      auto pro_suc = p_suc->GetProperty();
      JobArea* p_pre = (JobArea*)pre_object;
      auto pro_pre = p_pre->GetProperty();
      if (suc_object->GetUniqueID() ==
          pre_object->GetUniqueID()) {  // 车道在区域内部
        if (pro_suc->areaType ==
            3) {  // 是junction内车道，不需要制作overlap，只要关联junction到lane就行
          std::string junc_id =
              "junction_" + std::to_string(pre_object->GetUniqueID());
          if (pb_lanes.count(lane_id) > 0) {
            pb_lanes[lane_id].mutable_junction_id()->set_id(junc_id);
          }
        } else {  // 构造一个overlap 对象
          PbOverlap overlap;
          std::string overlap_id = CreateOverlapId();
          overlap.mutable_id()->set_id(overlap_id);
          overlap.add_object()->mutable_id()->set_id(lane_id);
          std::string area_id =
              "area_" + std::to_string(pre_object->GetUniqueID());
          overlap.add_object()->mutable_id()->set_id(area_id);
          pb_overlaps[overlap_id] = overlap;

          pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
          pb_areas[area_id].add_overlap_id()->set_id(overlap_id);
        }
        calc_finished = true;
      }
    }
    if (!calc_finished && pre_object) {  // 有前序压盖
      JobArea* p_pre = (JobArea*)pre_object;
      auto pro_pre = p_pre->GetProperty();
      PbOverlap overlap;
      std::string overlap_id = CreateOverlapId();
      overlap.mutable_id()->set_id(overlap_id);
      overlap.add_object()->mutable_id()->set_id(lane_id);
      if (pro_pre->areaType == 3) {
        std::string junc_id =
            "junction_" + std::to_string(pre_object->GetUniqueID());
        overlap.add_object()->mutable_id()->set_id(junc_id);
        pb_junctions[junc_id].add_overlap_id()->set_id(overlap_id);
      } else {
        std::string area_id =
            "area_" + std::to_string(pre_object->GetUniqueID());
        overlap.add_object()->mutable_id()->set_id(area_id);
        pb_areas[area_id].add_overlap_id()->set_id(overlap_id);
      }
      pb_overlaps[overlap_id] = overlap;
      pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
    }
    if (!calc_finished && suc_object) {  // 有后续压盖
      JobArea* p_suc = (JobArea*)suc_object;
      auto pro_suc = p_suc->GetProperty();
      PbOverlap overlap;
      std::string overlap_id = CreateOverlapId();
      overlap.mutable_id()->set_id(overlap_id);
      overlap.add_object()->mutable_id()->set_id(lane_id);
      if (pro_suc->areaType == 3) {
        std::string junc_id =
            "junction_" + std::to_string(suc_object->GetUniqueID());
        overlap.add_object()->mutable_id()->set_id(junc_id);
        pb_junctions[junc_id].add_overlap_id()->set_id(overlap_id);
      } else {
        std::string area_id =
            "area_" + std::to_string(suc_object->GetUniqueID());
        overlap.add_object()->mutable_id()->set_id(area_id);
        pb_areas[area_id].add_overlap_id()->set_id(overlap_id);
      }
      pb_overlaps[overlap_id] = overlap;
      pb_lanes[lane_id].add_overlap_id()->set_id(overlap_id);
    }
  }
  for (const auto& job : areas) {
    auto jobid = job->GetUniqueID();
    std::string area_id = "area_" + std::to_string(jobid);
    std::vector<MapFeature *> attachOjbects;
    job->GetAttachObject(attachOjbects);
    std::cout << "area_id =" << area_id << "; attachOjbects count=" << attachOjbects.size() << std::endl;
    for (auto attachOjbect : attachOjbects) {
      std::cout << "attachOjbect->GetType()=" << attachOjbect->GetType() << std::endl;
      if (attachOjbect->GetType() == MapFeature::MFT_JOB_AREA) { // 只制作功能点与功能区的压盖关系
        JobArea* job_point = (JobArea*)attachOjbect;
        auto pro = job_point->GetProperty();
        if (pro->areaType == 100) {
          auto job_point_id = "area_" + std::to_string(job_point->GetUniqueID());
          PbOverlap overlap;
          std::string overlap_id = CreateOverlapId();
          overlap.mutable_id()->set_id(overlap_id);
          overlap.add_object()->mutable_id()->set_id(job_point_id);
          overlap.add_object()->mutable_id()->set_id(area_id);
          pb_areas[area_id].add_overlap_id()->set_id(overlap_id);
          pb_overlaps[overlap_id] = overlap;
          pb_areas[job_point_id].add_overlap_id()->set_id(overlap_id);
        }
      }
    }
  }
  return 0;
}
}  // namespace geditor

int main(int argc, char const* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;
  if (argc < 3) return -1;
  std::string filedb = argv[1];
  std::string filepb = argv[2];
  std::string version = argv[3];
  geditor::VDB2PB v2pb;
  v2pb.Run(filedb, filepb, version);
  std::cout << "vdb to pb trans ok!" << std::endl;
  return 0;
}
