#include <string>
#include <vector>

#include "app/attribute_calc.hpp"
#include "common_msgs/map_msgs/map_dynamic.pb.h"
#include "core/boundary_layer.h"
#include "core/job_layer.h"
#include "core/lane_segment_layer.h"
#include "core/log.h"
#include "core/road_area_layer.h"
#include "core/sign_board_layer.h"

namespace geditor {
using PbDynamic = map_engine::hdmap::Dynamic;
using PbDynamicData = map_engine::hdmap::DynamicData;
class VDB2Dynamic {
 public:
  VDB2Dynamic();
  ~VDB2Dynamic();
  int Run(std::string filedb, std::string filepb, std::string version);
  int ReadVDB(std::string file);
  int ConvFeature2Pb(std::string path, std::string version);

 private:
  int FillDynamicPbPoints(const std::vector<Point3d>& pts, PbDynamicData& pb_dynamic);

 private:
  BoundaryLayer* layer_boundary_;
  SegmentLayer* layer_lane_;
  AreaLayer* layer_roadarea_;
  PointLayer* layer_sign_;
  JobLayer* layer_job_;
};

}  // namespace geditor
