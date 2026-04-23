
#pragma once

#include "map/tile_pdb.h"
#include "pcd/async_task.h"
#include "pcd/pointcloud.h"
#include "pcd/read_pcl.h"

namespace geditor {

class AsyncTask;

class PCDSplitter {
 public:
  PCDSplitter();

  ~PCDSplitter();

  bool LoadPCDFile(const char *szPDBfile, const char *filename);

  void SetOriginPoint(double originX, double originY, int zone);

  bool SplitData(const PointCloud<PCLPoint> &cloud,
                 std::vector<TilePDB *> &tileArray);

 private:
  double move_orgin_x;
  double move_orgin_y;
  int orgin_zone;
};

}  // namespace geditor
