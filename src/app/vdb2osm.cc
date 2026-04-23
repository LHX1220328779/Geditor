#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <memory>

#include "core/boundary_layer.h"
#include "core/job_layer.h"
#include "core/lane_segment_layer.h"
#include "core/log.h"
#include "core/road_area_layer.h"
#include "core/sign_board_layer.h"

using namespace lanelet;
namespace geditor {

class NoneProjector : public Projector {
 public:
  explicit NoneProjector() : Projector(Origin(lanelet::GPSPoint{0, 0, 0})) {}

  BasicPoint3d forward(const lanelet::GPSPoint& gps) const override {
    return BasicPoint3d{gps.lat, gps.lon, gps.ele};
  }

  lanelet::GPSPoint reverse(const BasicPoint3d& utm) const override {
    return lanelet::GPSPoint{utm.x(), utm.y(), utm.z()};
  }
};

class VDB2OSM {
 public:
  VDB2OSM() : osm_(new LaneletMap()) {
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
  ~VDB2OSM() {
    delete layer_boundary_;
    delete layer_lane_;
    delete layer_roadarea_;
    delete layer_sign_;
  }
  int Run(std::string filedb, std::string fileosm) {
    RETURN_VAL_IF(ReadVDB(filedb) < 0, -1);
    RETURN_VAL_IF(ConvFeature() < 0, -1);
    return WriteOsm(fileosm);
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
  int ConvFeature() {
    auto ss = layer_sign_->GetAllSign();
    for (auto& s : ss) {
      auto line = ConvLine(s);
      line.attributes()[AttributeName::Type] =
          AttributeValueString::TrafficLight;
      osm_->add(line);
    }
    auto rs = layer_roadarea_->GetAllRoadarea();
    for (auto& r : rs) {
      auto line = ConvLine((MapFeature*)r);
      line.attributes()[AttributeName::Type] = AttributeValueString::StopLine;
      osm_->add(line);
    }

    auto lanes = layer_lane_->GetAllLane();
    for (auto& lane : lanes) {
      // auto cl = ConvLine(lane);
      auto ll = ConvLine(lane->GetLeftBoundary());
      ll.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
      ll.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;
      auto rl = ConvLine(lane->GetRightBoundary());
      rl.attributes()[AttributeName::Type] = AttributeValueString::LineThin;
      rl.attributes()[AttributeName::Subtype] = AttributeValueString::Dashed;
      Lanelet lanelet(lane->GetUniqueID(), ll, rl);
      lanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
      lanelet.attributes()["turn_direction"] = "straight";
      auto rg = ConvRelation(lane->GetAttachObject());
      if (rg) lanelet.addRegulatoryElement(rg);
      osm_->add(lanelet);
    }
    return 0;
  }
  int WriteOsm(std::string file) {
    LINFO << file;

    lanelet::write(file, *osm_, NoneProjector());
    return 0;
  }
  int ReadOsm(std::string file) {
    auto map = lanelet::load(file, NoneProjector());
    return 0;
  }

 private:
  LineString3d ConvLine(MapFeature* feat) {
    auto id = feat->GetUniqueID();
    auto& geo = *feat->GetGeometry();
    int size = geo.GetVertexCount();
    Points3d ps;
    ProjectionUTM utm;
    for (int i = 0; i < size; ++i) {
      auto p = geo.GetVertex(i);
      LatLon ll;
      utm.CartesianToLatLon(p.x, p.y, ProjectionUTM::zone, false, ll);
      ps.push_back(lanelet::Point3d(p.Id(), ll.lat, ll.lon, p.z));
    }
    auto ls = LineString3d(id, ps);
    return ls;
  }
  RegulatoryElementPtr ConvRelation(std::vector<MapFeature*> feats) {
    LineStringsOrPolygons3d ls;
    LineString3d sl;
    for (auto feat : feats) {
      if (feat->GetType() == MapFeature::MFT_SIGNBORAD) {
        auto l = ConvLine(feat);
        ls.push_back(LineString3d(feat->GetUniqueID(), {}));
      } else if (feat->GetType() == MapFeature::MFT_ROAD_AREA) {
        sl = LineString3d(feat->GetUniqueID(), {});
      }
    }
    if (ls.empty())
      return nullptr;
    else
      return lanelet::TrafficLight::make(GenerateFeatureID(), {}, ls, sl);
  }

 private:
  BoundaryLayer* layer_boundary_;
  SegmentLayer* layer_lane_;
  AreaLayer* layer_roadarea_;
  PointLayer* layer_sign_;
  std::unique_ptr<LaneletMap> osm_;
};
}  // namespace geditor
int main(int argc, char const* argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;
  if (argc < 3) return -1;
  std::string filedb = argv[1];
  std::string fileosm = argv[2];
  geditor::VDB2OSM v2o;
  v2o.Run(filedb, fileosm);
  v2o.ReadOsm(fileosm);
  return 0;
}
