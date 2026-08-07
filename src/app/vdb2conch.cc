#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>

#include "app/attribute_calc.hpp"
#include "core/boundary_layer.h"
#include "core/job_layer.h"
#include "core/lane_segment_layer.h"
#include "core/log.h"
#include "core/road_area_layer.h"
#include "core/sign_board_layer.h"
#include "fileopra.hpp"
#include "map/pdb_manage.h"
#include "map/mine_origin_config.h"
namespace fs = std::filesystem;
namespace geditor {

namespace {

std::string ResolveSegmentOutputName(
    const SegmentProperty* seg_property, const LaneSegment* lane,
    const std::map<std::string, std::string>& index2name) {
  if (seg_property != nullptr) {
    if (strlen(seg_property->mineSegmentCode) > 0) {
      return seg_property->mineSegmentCode;
    }
    if (seg_property->mineSegmentIndex > 0) {
      auto iter = index2name.find(std::to_string(seg_property->mineSegmentIndex));
      if (iter != index2name.end()) {
        return iter->second;
      }
      return std::to_string(seg_property->mineSegmentIndex);
    }
    if (strlen(seg_property->name) > 0) {
      auto iter = index2name.find(seg_property->name);
      if (iter != index2name.end()) {
        return iter->second;
      }
      return seg_property->name;
    }
  }
  return std::to_string(lane->GetUniqueID());
}

}  // namespace

// class NoneProjector : public Projector {
//  public:
//   explicit NoneProjector() : Projector(Origin(lanelet::GPSPoint{0, 0, 0})) {}

//   BasicPoint3d forward(const lanelet::GPSPoint& gps) const override {
//     return BasicPoint3d{gps.lat, gps.lon, gps.ele};
//   }

//   lanelet::GPSPoint reverse(const BasicPoint3d& utm) const override {
//     return lanelet::GPSPoint{utm.x(), utm.y(), utm.z()};
//   }
// };

class VDB2Conch {
 public:
  VDB2Conch() {
    layer_roadarea_ = new AreaLayer();
    layer_lane_ = new SegmentLayer();
    layer_sign_ = new PointLayer();
    layer_boundary_ = new BoundaryLayer();
    LayerHelper layer_helper;
    layer_helper.SetLayer(layer_roadarea_, LT_ROADAREA);
    layer_helper.SetLayer(layer_lane_, LT_LANE);
    layer_helper.SetLayer(layer_sign_, LT_SIGN);
    layer_helper.SetLayer(layer_boundary_, LT_BOUNDARY);
    layer_lane_->SetLayers(layer_helper);
  }
  ~VDB2Conch() {
    delete layer_boundary_;
    delete layer_lane_;
    delete layer_roadarea_;
    delete layer_sign_;
  }
  int Run(std::string filedb, std::string name_mapping_file, std::string fileconch) {
    RETURN_VAL_IF(ReadVDB(filedb) < 0, -1);
    RETURN_VAL_IF(ConvFeatureConchSeg(name_mapping_file, fileconch) < 0, -1);
    return 0;
  }
  int ReadVDB(std::string file) {
    VDBManage* vdb = new VDBManage();
    if (vdb->Create(file.c_str())) {
      layer_boundary_->Read(vdb);
      layer_roadarea_->Read(vdb);
      layer_sign_->Read(vdb, layer_lane_);
      layer_lane_->Read(vdb);
      vdb->Close();
      delete vdb;
      return 0;
    }
    delete vdb;
    return -1;
  }
  int ConvFeatureConchSeg(std::string name_mapping_file, std::string path) {
    auto p_seg = path + "/segment_map";
    auto p_bou = path + "/boundary_map";
    file_opra::ReCreateDirectory(p_seg);
    file_opra::ReCreateDirectory(p_bou);
    std::map<std::string, std::string> index2name;
    if (name_mapping_file.size() > 0) {
      ReadNameMapping(name_mapping_file, index2name);
    }
    auto lanes = layer_lane_->GetAllLane();
    // ExportLanePoints(path);  // 测试code，输出车道原始坐标
    for (auto lane : lanes) {
      auto plist = GetLaneInterpolateLane(lane, 0.5);
      std::vector<std::vector<std::string>> conch_seg_data;
      std::vector<std::vector<std::string>> conch_gps_data;
      ConvLine2ConchSeg(lane, plist, conch_seg_data, conch_gps_data);
      auto seg_property = lane->GetProperty();
      std::string s_name =
          ResolveSegmentOutputName(seg_property, lane, index2name);
      // 写车道表
      auto f_seg = p_seg + "/" + s_name + ".csv";
      WriteConchSegmentMap(f_seg, conch_seg_data);

      // // 写gps车道表
      // auto f_gps = p_seg + "/" + s_name +
      // "_gps.csv"; WriteConchSegmentMap(f_gps, conch_gps_data);

      // 写边界表
      auto f_bou = p_bou + "/" + s_name + "_boundary.csv";
      auto lane_left_bou = lane->GetLeftBoundary();
      auto lane_right_bou = lane->GetRightBoundary();
      if (!lane_left_bou || !lane_left_bou) {  // 没有边线，不生成边界表
        continue;
      }
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
      bool is_left_road_bound_reverse = false;
      // 此处如果最左侧车道还有对向车道关系，则需要找到对向车道的最右侧车道的右边界
      if (leftmost->GetParallelSegment()->leftReverseSegment > 0) {
        int id = leftmost->GetParallelSegment()->leftReverseSegment;
        auto reverse_lane = (LaneSegment*)(layer_lane_->GetFeature(id));
        std::vector<LaneSegment*> reverse_parallel_lanes;
        layer_lane_->GetAllParallelSegment(reverse_lane,
                                           reverse_parallel_lanes);
        auto reverse_most = reverse_lane->GetParallelSegment()->rightSegment > 0
                                ? reverse_parallel_lanes.back()
                                : reverse_lane;
        road_left_bou = reverse_most->GetRightBoundary();
        is_left_road_bound_reverse = true;
        // std::cout << "reverse_most = " << reverse_most->GetUniqueID()
        //           << ";name = " << reverse_most->GetProperty()->name
        //           << std::endl;
      }
      // else {
      //   std::cout << "no reverse_most" << std::endl;
      // }
      WriteConchBoundaryMap(f_bou, lane_left_bou, lane_right_bou, road_left_bou,
                            road_right_bou, is_left_road_bound_reverse);
    }
    // write road_right.csv
    bool write_road_right = false;
    if (write_road_right) {
      auto filename = path + "/road_right.csv";
      WriteRoadRightFile(filename);
    }
    return 0;
  }

 private:
  bool ReadNameMapping(std::string name_mapping_file, std::map<std::string, std::string>& index2name) {
    std::ifstream file(name_mapping_file);
    if (!file.is_open()) {
      std::cerr << "Failed to open name mapping file: " << name_mapping_file << std::endl;
      return false;
    }

    std::string line;
    // Skip header line
    std::getline(file, line);

    while (std::getline(file, line)) {
      // Remove any whitespace
      line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
      
      // Skip empty lines
      if (line.empty()) {
        continue;
      }

      // Find the comma separator
      size_t comma_pos = line.find(',');
      if (comma_pos == std::string::npos) {
        std::cerr << "Invalid line format in name mapping file: " << line << std::endl;
        continue;
      }

      // Extract filename and index
      std::string filename = line.substr(0, comma_pos);
      std::string index_str = line.substr(comma_pos + 1);

      try {
        // int index = std::stoi(index_str);
        index2name[index_str] = filename;
      } catch (const std::exception& e) {
        std::cerr << "Failed to parse index_str in line: " << line << std::endl;
      }
    }

    file.close();
    return true;
  }
  std::vector<Point3d> GetLaneInterpolateLane(const LaneSegment* l,
                                              double maxDist) {
    auto geo = l->GetGeometry();
    // size_t size = geo.GetVertexCount();
    // std::vector<Point3d> raw_plist;
    // raw_plist.reserve(size);
    // for (size_t i = 0; i < size; ++i) {
    //   raw_plist.emplace_back(geo.GetVertex(i));
    // }
    // return attribute_calc::InterpolateLane(raw_plist, maxDist);
    return GetGeoInterpolate(geo, maxDist);
  }

   std::vector<Point3d> GetGeoInterpolate(Geometry * geo,
                                              double maxDist) {
    size_t size = geo->GetVertexCount();
    std::vector<Point3d> raw_plist;
    raw_plist.reserve(size);
    for (size_t i = 0; i < size; ++i) {
      raw_plist.emplace_back(geo->GetVertex(i));
    }
    return attribute_calc::InterpolateLane(raw_plist, maxDist);
  }
  int ExportLanePoints(std::string path, int lane_id = 0) {
    // 打印车道原始坐标信息
    auto lanes = layer_lane_->GetAllLane();
    for (auto lane : lanes) {
      if (lane_id > 0) {
        if (lane->GetUniqueID() != lane_id) {
          continue;
        }
      }
      auto filename = path + "/raw_" +
                      ResolveSegmentOutputName(lane->GetProperty(), lane, {}) +
                      ".csv";
      LINFO << filename;
      std::ofstream file(filename);
      if (!file.is_open()) {
        std::cerr << "Failed to create file: " << filename << std::endl;
        return 1;
      }
      auto& lane_geo = *lane->GetGeometry();
      size_t lane_size = lane_geo.GetVertexCount();
      for (size_t i = 0; i < lane_size; i++) {
        const auto& vertex = lane_geo.GetVertex(i);
        file << std::fixed << std::setprecision(6) << vertex.x << ","
             << vertex.y << std::endl;
      }
      file.close();
      if (lane_id > 0) {
        break;
      }
    }
    return 0;
  }
  Point3d Utm2Local(const Point3d& p) {
    ProjectionUTM utm;
    LatLon ll;
    utm.CartesianToLatLon(p.x, p.y, ProjectionUTM::zone, false, ll);
    Point3d pp = attribute_calc::TransGps2Pt(ll.lon, ll.lat, p.z);
    return pp;
  }
  void ConvLine2ConchSeg(const LaneSegment* l,
                         const std::vector<Point3d>& interpolate_plist,
                         std::vector<std::vector<std::string>>& seg_data,
                         std::vector<std::vector<std::string>>& gps_data) {
    size_t size = interpolate_plist.size();
    seg_data.clear();
    gps_data.clear();
    float last_heading = 0;
    ProjectionUTM utm;
    for (size_t i = 0; i < size; ++i) {
      auto& p = interpolate_plist[i];
      float heading = last_heading;
      if (i + 1 < size) {
        heading = attribute_calc::calculateHeading(
            p.x, p.y, interpolate_plist[i + 1].x, interpolate_plist[i + 1].y);
      }

      LatLon ll;
      utm.CartesianToLatLon(p.x, p.y, ProjectionUTM::zone, false, ll);
      gps_data.push_back({std::to_string(ll.lon), std::to_string(ll.lat),
                          std::to_string(heading), "10"});

      Point3d pp = attribute_calc::TransGps2Pt(ll.lon, ll.lat, p.z);
      seg_data.push_back({std::to_string(pp.x), std::to_string(pp.y),
                          std::to_string(heading), "10", "1", "0"});
      // auto gps_p = attribute_calc::TransPt2Gps(p);

      last_heading = heading;
    }
  }
  int WriteConchSegmentMap(const std::string& filename,
                           const std::vector<std::vector<std::string>>& data) {
    LINFO << filename;
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to create file: " << filename << std::endl;
      return 1;
    }
    file << "x,y,heading,v,gear,kk";
    file << std::endl;
    for (const auto& row : data) {
      for (size_t i = 0; i < row.size(); ++i) {
        file << row[i];
        if (i < row.size() - 1) {
          file << ",";
        }
      }
      file << std::endl;
    }
    file.close();
    return 0;
  }
  std::string Utm2LocalStr(const Point3d& p) {
    Point3d pp = Utm2Local(p);
    return std::to_string(pp.x) + "," + std::to_string(pp.y);
    // return std::to_string(p.x-538206.889604231) + "," + std::to_string(p.y-3401992.722156150);
  }
  int WriteConchBoundaryMap(const std::string& filename,
                            const BoundSegment* lane_left_bou,
                            const BoundSegment* lane_right_bou,
                            const BoundSegment* road_left_bou,
                            const BoundSegment* road_right_bou,
                            bool is_left_road_bound_reverse) {
    LINFO << filename;
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to create file: " << filename << std::endl;
      return 1;
    }
    file << "lane_left_boundary_x,lane_left_boundary_y,lane_right_boundary_x,"
            "lane_right_boundary_y,road_left_boundary_x,road_left_boundary_y,"
            "road_right_boundary_x,road_right_boundary_y";
    file << std::endl;
    auto l_lane_geo = lane_left_bou->GetGeometry();
    auto l_lane_pts = GetGeoInterpolate(l_lane_geo, 0.5);
    size_t l_lane_size = l_lane_pts.size();  // l_lane_geo.GetVertexCount();

    auto r_lane_geo = lane_right_bou->GetGeometry();
    auto r_lane_pts = GetGeoInterpolate(r_lane_geo, 0.5);
    size_t r_lane_size = r_lane_pts.size();  // r_lane_geo.GetVertexCount();

    auto l_road_geo = road_left_bou->GetGeometry();
    auto l_road_pts = GetGeoInterpolate(l_road_geo, 0.5);
    size_t l_road_size = l_road_pts.size();  // l_road_geo.GetVertexCount();

    auto r_road_geo = road_right_bou->GetGeometry();
    auto r_road_pts = GetGeoInterpolate(r_road_geo, 0.5);
    size_t r_road_size = r_road_pts.size();  // r_road_geo.GetVertexCount();

    size_t lane_max_size = std::max(l_lane_size, r_lane_size);
    size_t road_max_size = std::max(l_road_size, r_road_size);
    size_t max_size = std::max(lane_max_size, road_max_size);

    // std::vector<Point3d> l_road_points;
    // l_road_points.reserve(l_road_size);
    // if (is_left_road_bound_reverse) {
    //   for (int i = l_road_size - 1; i >= 0; --i) {
    //     l_road_points.push_back(l_road_pts[i]);
    //   }
    // } else {
    //   for (size_t i = 0; i < l_road_size; ++i) {
    //     l_road_points.push_back(l_road_geo.GetVertex(i));
    //   }
    // }
    if (is_left_road_bound_reverse) {
      std::reverse(l_road_pts.begin(), l_road_pts.end());
    }
    for (size_t i = 0; i < max_size; ++i) {
      i >= l_lane_size ? file << ",,"
                       : file << Utm2LocalStr(l_lane_pts[i]) << ",";
      i >= r_lane_size ? file << ",,"
                       : file << Utm2LocalStr(r_lane_pts[i]) << ",";
      i >= l_road_size ? file << ",,"
                       : file << Utm2LocalStr(l_road_pts[i]) << ",";
      i >= r_road_size ? file << ","
                       : file << Utm2LocalStr(r_road_pts[i]);
      file << std::endl;
    }
    file.close();
    return 0;
  }

  int WriteRoadRightFile(std::string filename) {
    LINFO << filename;
    std::ofstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Failed to create file: " << filename << std::endl;
      return 1;
    }
    file << "路段编码,路段长度,通行数量,影响路段列表,前继路段,后继路段,"
            "申请路权距离,是否为铲装区路段,路线区域,env1,env2,env3,env4,"
            "env5,env6,env7,env8,释放路权距离,风险路段,所属区域,route_"
            "attribute,均匀碾压,起点目标,终点目标,是否为卸装区路段,"
            "平台编码,平台主路编码,绕障,车道影响列表";
    file << std::endl;
    auto lanes = layer_lane_->GetAllLane();
    for (auto lane : lanes) {
      file << lane->GetUniqueID() << ",";
      file << lane->GetProperty()->length << ",";
      int pass_num = 0;
      file << pass_num << ",";
    }
    file.close();
    return 0;
  }

 private:
  BoundaryLayer* layer_boundary_;
  SegmentLayer* layer_lane_;
  AreaLayer* layer_roadarea_;
  PointLayer* layer_sign_;
};
}  // namespace geditor
int main(int argc, char const* argv[]) {
  // geditor::TestCalcHeading();
  // return 0;
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;
  if (argc < 4) return -1;
  std::string filedb = argv[1];
  std::string name_mapping_file = argv[2];
  std::string fileconch = argv[3];
  geditor::MineOrigin origin;
  if (argc >= 7) {
    origin.longitude = atof(argv[4]);
    origin.latitude = atof(argv[5]);
    origin.z = atof(argv[6]);
  } else {
    std::vector<geditor::MineOrigin> origins;
    std::string error;
    const auto config = fs::absolute(argv[0]).parent_path().parent_path() /
                        "mine_origins.yaml";
    if (!geditor::MineOriginConfig::Load(config.string(), origins, &error)) {
      std::cerr << "Mine origin is required: " << error << std::endl;
      return -1;
    }
    origin = origins.front();
  }
  geditor::attribute_calc::InitGlobalOrigin(origin.latitude, origin.longitude,
                                             origin.z);
  geditor::VDB2Conch v2c;
  v2c.Run(filedb, name_mapping_file, fileconch);
  return 0;
}
