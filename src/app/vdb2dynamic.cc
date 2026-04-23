#include "vdb2dynamic.h"

#include <fcntl.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#include "pbfileopra.hpp"

namespace fs = std::filesystem;
namespace geditor {
using PbDynamicType = map_engine::hdmap::DynamicData_Type;
using PbGeoType = map_engine::hdmap::DynamicData_GeoType;
using PbReg = map_engine::hdmap::DynamicData_BelongingRegion;

VDB2Dynamic::VDB2Dynamic() {
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
VDB2Dynamic::~VDB2Dynamic() {
  delete layer_boundary_;
  delete layer_lane_;
  delete layer_roadarea_;
  delete layer_sign_;
  delete layer_job_;
}
int VDB2Dynamic::Run(std::string filedb, std::string filepb,
                     std::string version) {
  RETURN_VAL_IF(ReadVDB(filedb) < 0, -1);
  RETURN_VAL_IF(ConvFeature2Pb(filepb, version) < 0, -1);
  return 0;
}
int VDB2Dynamic::ReadVDB(std::string file) {
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

int VDB2Dynamic::ConvFeature2Pb(std::string path, std::string version) {
  auto p_seg = path + "/" + version;
  file_opra::CreateDirectory(p_seg);
  std::vector<LaneSegment*> lanes = layer_lane_->GetAllLane();
  PbDynamic dynamics;

  for (auto& lane : lanes) {
    auto seg_property = lane->GetProperty();
    if (seg_property->laneType != 11 && seg_property->laneType != 12 &&
        seg_property->laneType != 13) {  // 过滤掉非参考线类型
      // LOG(INFO) << "lane id=" << lane->GetUniqueID() << " is not reference
      // line";
      continue;
    }
    PbDynamicData pb_dynamic;
    if (seg_property->laneType == 11) {
      pb_dynamic.set_type(PbDynamicData::ENTRY_LINE);
    } else if (seg_property->laneType == 12) {
      pb_dynamic.set_type(PbDynamicData::BACK_LINE);
    } else if (seg_property->laneType == 13) {
      pb_dynamic.set_type(PbDynamicData::EXIT_LINE);
    } else {
      LOG(ERROR) << "lane id=" << lane->GetUniqueID()
                 << " is not reference line";
    }

    auto obj_id = "obj_" + std::to_string(lane->GetUniqueID());
    pb_dynamic.set_id(obj_id);
    auto suc_object = lane->GetSuccessorFeature();
    auto pre_object = lane->GetPredecessorFeature();

    if (suc_object) {
      if (suc_object->GetType() != MapFeature::MFT_JOB_AREA) {
        LOG(INFO) << "obj id: " << obj_id
                  << " SuccessorFeature type: " << suc_object->GetType()
                  << " not MFT_JOB_AREA!";
        suc_object = nullptr;
      }
    }
    if (pre_object) {
      if (pre_object->GetType() != MapFeature::MFT_JOB_AREA) {
        LOG(INFO) << "obj id: " << obj_id
                  << " PredecessorFeature type: " << pre_object->GetType()
                  << " not MFT_JOB_AREA!";
        pre_object = nullptr;
      }
    }
    if (suc_object && pre_object) {  // 前序后继都找到压盖区域
      JobArea* p_suc = (JobArea*)suc_object;
      auto pro_suc = p_suc->GetProperty();
      if (suc_object->GetUniqueID() ==
          pre_object->GetUniqueID()) {  // 车道在区域内部
        if (pro_suc->areaType != 3) {   // 不是junction内车道，需要处理
          switch (pro_suc->areaType) {
            case 0:  // 铲装区
              pb_dynamic.set_reg(PbDynamicData::SHOVEL_LOADING);
              break;
            case 1:  // 破碎站
              pb_dynamic.set_reg(PbDynamicData::CRUSHING_STATION);
              break;
            case 2:  // 排土区
              pb_dynamic.set_reg(PbDynamicData::DISPOSAL_AREA);
              break;
            default:
              pb_dynamic.set_reg(PbDynamicData::UNKNOW);
              break;
          }
        }
      }
    }
    std::vector<Point3d> pts;
    auto geo = lane->GetGeometry();
    geo->Hermite(pts);
    FillDynamicPbPoints(pts, pb_dynamic);
    pb_dynamic.set_geo_type(PbDynamicData::GEO_LINE);
    pb_dynamic.set_info("");
    dynamics.mutable_dynamic_data()->Add()->CopyFrom(pb_dynamic);
  }

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
    PbDynamicType type;
    PbGeoType geo_type = PbDynamicData::GEO_POLYGON;
    PbReg reg = PbDynamicData::MAIN_ROASD;
    bool bFlag = true;
    switch (pro->areaType) {
      case 7:  // 翻浆区
        type = PbDynamicData::BOILING_AREA;
        break;
      case 8:  // 不可通行区
        type = PbDynamicData::IMPASSABLE_AREA;
        break;
      case 9:  // 洒落区
        type = PbDynamicData::SCATTERED_AREA;
        break;
      case 102:  // “人”字形倒车起点
        type = PbDynamicData::BACK_START_POINT;
        geo_type = PbDynamicData::GEO_POSE;
        reg = PbDynamicData::SHOVEL_LOADING;
        break;
      case 103:  // “人”字形倒车终点
        type = PbDynamicData::BACK_END_POINT;
        geo_type = PbDynamicData::GEO_POSE;
        reg = PbDynamicData::SHOVEL_LOADING;
        break;
      default:
        bFlag = false;
        break;
    }
    if (bFlag) {
      PbDynamicData pb_dynamic;
      std::string obj_id = "obj_" + std::to_string(jobid);
      pb_dynamic.set_id(obj_id);
      pb_dynamic.set_type(type);
      pb_dynamic.set_reg(reg);
      std::vector<Point3d> pts;
      job->GetGeometry()->Hermite(pts);
      FillDynamicPbPoints(pts, pb_dynamic);
      pb_dynamic.set_geo_type(geo_type);
      pb_dynamic.set_info("");
      dynamics.mutable_dynamic_data()->Add()->CopyFrom(pb_dynamic);
    }
  }
  dynamics.set_report_time(std::to_string(std::time(nullptr)));
  dynamics.set_lowest_supported_static_data_verison("0");
  std::ofstream output(p_seg + "/dynamic.bin",
                       std::ios::out | std::ios::trunc | std::ios::binary);
  if (!output) {
    std::cerr << "无法打开文件 " << p_seg << "/dynamic.bin 进行写入"
              << std::endl;
    return 1;
  }
  if (!dynamics.SerializeToOstream(&output)) {
    std::cerr << "序列化失败" << std::endl;
    return false;
  }
  file_opra::SetProtoToASCIIFile(dynamics, p_seg + "/dynamic.txt");
  return 0;
}

int VDB2Dynamic::FillDynamicPbPoints(const std::vector<Point3d>& pts,
                                     PbDynamicData& pb_dynamic) {
  for (const auto& pt : pts) {
    auto pb_pt = pb_dynamic.mutable_pts()->Add();
    pb_pt->set_x(pt.x);
    pb_pt->set_y(pt.y);
    pb_pt->set_z(pt.z);
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
  geditor::VDB2Dynamic v2pb;
  v2pb.Run(filedb, filepb, version);
  std::cout << "vdb to dynamic trans ok!" << std::endl;
  return 0;
}
