
#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <memory>

#include "algorithm/common.h"
#include "core/log.h"
#include "map/pdb_manage.h"
#include "map/tile_pdb.h"
#include "pcd/db_read_write.h"
#include "pcd/pcd_split.h"
#include "pcd/voxel_grid.h"

using namespace geditor;

DEFINE_string(pcd_path, "", "dir of info.txt");
DEFINE_string(pdb_path, "", "path to name.pdb");

class PCD2Pdb {
 public:
  int Build(std::string dir, std::string txt, std::string vdb) {
    std::string file = dir + "/" + txt;
    std::fstream fs(file);
    if (!fs.is_open()) {
      LOG(ERROR) << "Failed to open " << file;
      return -1;
    }
    std::string line;
    int skip = 0, skiprow = 1;
    bool has_record = false;
    while (std::getline(fs, line)) {
      if (skip++ < skiprow) continue;
      std::stringstream ss(line);
      std::string pcd;
      double x, y, z;
      int zone;
      if (!(ss >> pcd >> x >> y >> z >> zone)) {
        LOG(ERROR) << "Invalid info.txt line: " << line;
        return -1;
      }
      if (zone < 1 || zone > 60) {
        LOG(ERROR) << "Invalid UTM zone in info.txt: " << zone;
        return -1;
      }
      pcd = dir + "/" + pcd;
      V3d origin(x, y, z);
      tra_[pcd] = origin;
      if (!has_record) {
        zone_ = zone;
        origin_ = origin;
        has_record = true;
      } else if (zone_ != zone) {
        LOG(ERROR) << "All info.txt records must use the same UTM zone: "
                   << zone_ << " vs " << zone;
        return -1;
      }
    }
    if (!has_record) {
      LOG(ERROR) << "No point-cloud records in " << file;
      return -1;
    }
    pdb_path_ = vdb;
    split_task();
    return 0;
  }

 private:
  void split_task() {
    PointCloud<PCLPoint> outCloud;
    std::shared_ptr<PCDSplitter> splitter(new PCDSplitter());
    splitter->SetOriginPoint(origin_[0], origin_[1], zone_);

    if (merge_pcd2pcd(pcd_path_, outCloud)) {
      //切分pcd
      std::vector<TilePDB *> tileArray;
      if (splitter->SplitData(outCloud, tileArray)) {
        outCloud.Clear();
        //保存到数据库
        PDBManage database;
        if (database.Open(pdb_path_.c_str())) {
          if (!database.SetUTMZone(zone_)) {
            LOG(ERROR) << "Failed to save UTM zone metadata";
            return;
          }
          if (!database.SetPointColorModeRGB(false)) {
            LOG(ERROR) << "Failed to save point color metadata";
            return;
          }
          size_t isize = tileArray.size();
          for (size_t pos = 0; pos < isize; pos++) {
            TilePDB *pTile = tileArray[pos];
            database.Save(pTile);
            //释放内存
            delete pTile;
          }
          database.Close();
          return;
        }
      }
    } else {
      outCloud.Clear();
    }
    return;
  }
  bool merge_pcd2pcd(const std::string &db_file_name,
                     geditor::PointCloud<geditor::PCLPoint> &map) {
    for (auto &node : tra_) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr ps(
          new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(node.first, *ps) == -1) continue;
      PointCloud<PCLPoint> cloud(ps->points.size());
      auto offset = node.second;
      for (auto &p : ps->points) {
        PCLPoint pp;
        pp.x = p.x + offset.x - origin_.x;
        pp.y = p.y + offset.y - origin_.y;
        // PDB altitude is absolute. X/Y are temporarily made relative because
        // PCDSplitter adds the selected UTM origin back during projection.
        pp.z = p.z + offset.z;
        pp.w = p.intensity;
        cloud.PushBack(pp);
      }
      PointCloud<PCLPoint> out;
      VoxelGrid<PCLPoint> sor;
      sor.setInputCloud(&cloud);
      sor.setLeafSize(0.1, 0.1, 0.1);
      sor.filter(out);
      map.MergePointCloud(out);
    }
    return true;
  }

  geditor::V3d origin_;
  int zone_ = 0;
  std::string pcd_path_;
  std::string pdb_path_;
  std::map<std::string, geditor::V3d> tra_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_pcd_path.empty()) {
    LOG(ERROR) << "Please input db path;";
    return -1;
  }

  if (FLAGS_pdb_path.empty()) {
    LOG(ERROR) << "Please input pdb path;";
    return -1;
  }
  PCD2Pdb pdb;
  return pdb.Build(FLAGS_pcd_path, "info.txt", FLAGS_pdb_path);
}
