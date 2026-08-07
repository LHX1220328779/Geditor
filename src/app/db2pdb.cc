
#include <gflags/gflags.h>

#include <memory>

#include "algorithm/common.h"
#include "core/log.h"
#include "map/pdb_manage.h"
#include "map/tile_pdb.h"
#include "pcd/pcd_split.h"
#include "pcd/sqlite_rwer.h"
#include "pcd/voxel_grid.h"

using namespace geditor;

DEFINE_string(db_path, "", "path ot db");
DEFINE_string(pdb_path, "", "path ot pdb");

bool merge_db2pcd(const std::string &db_file_name,
                  geditor::PointCloud<geditor::PCLPoint> &map);

std::vector<DBTraPoint> db_tra;
int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (FLAGS_db_path.empty()) {
    LWARN << "Please input db path;";
    return 1;
  }

  if (FLAGS_pdb_path.empty()) {
    LWARN << "Please input pdb path;";
    return 1;
  }

  V3d origin(0, 0, 0);
  int zone = 0;
  bool east = true;
  SqliteRWer db(FLAGS_db_path);
  if (db.IsOpen()) {
    db_tra = db.ReadTra();
    if (!db_tra.empty()) {
      db.CloseDB();
    } else {
      db.CloseDB();
      LWARN << "DB格式错误";
      return 2;
    }
    origin = V3d(db_tra[0].x, db_tra[0].y, db_tra[0].z);
    zone = db_tra[0].zone;
  } else {
    LWARN << "无法读取DB";
    return 3;
  }

  PointCloud<PCLPoint> outCloud;
  std::shared_ptr<PCDSplitter> splitter(new PCDSplitter());
  splitter->SetOriginPoint(origin[0], origin[1], zone);
  LINFO << origin[0] << " " << origin[1] << " " << zone;

  if (merge_db2pcd(FLAGS_db_path, outCloud)) {
    std::vector<TilePDB *> tileArray;
    if (splitter->SplitData(outCloud, tileArray)) {
      outCloud.Clear();
      //保存到数据库
      PDBManage database;
      if (database.Open(FLAGS_pdb_path.c_str())) {
        if (!database.SetUTMZone(zone)) {
          LERROR << "无法保存UTM带区元数据";
          return 4;
        }
        size_t isize = tileArray.size();
        for (size_t pos = 0; pos < isize; pos++) {
          TilePDB *pTile = tileArray[pos];
          database.Save(pTile);
          //释放内存
          delete pTile;
        }
        database.Close();
        return 0;
      }
    }
  } else {
    outCloud.Clear();
  }
  LINFO << "done.";
  return 100;
}

bool merge_db2pcd(const std::string &db_file_name,
                  geditor::PointCloud<geditor::PCLPoint> &map) {
  SqliteRWer db(db_file_name);
  if (!db.IsOpen()) return false;
  if (db_tra.empty()) return false;
  double offx = db_tra[0].x;
  double offy = db_tra[0].y;
  double offz = db_tra[0].z;
  for (auto &node : db_tra) {
    DBPoints ps = db.ReadFrameByTime(node.time);
    if (ps.points.empty()) continue;
    PointCloud<PCLPoint> cloud(ps.points.size());
    for (auto &p : ps.points) {
      PCLPoint pp;
      pp.x = ps.offset_x + p.x - offx;
      pp.y = ps.offset_y + p.y - offy;
      pp.z = ps.offset_z + p.z - offz;
      pp.w = p.i;
      cloud.PushBack(pp);
    }
    PointCloud<PCLPoint> out;
    VoxelGrid<PCLPoint> sor;
    sor.setInputCloud(&cloud);
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.filter(out);
    map.MergePointCloud(out);
  }
  db.CloseDB();
  return true;
}
