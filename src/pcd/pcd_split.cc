
#include <minilzo.h>
#include <sqlite_cpp.h>

#include "map/pdb_manage.h"
#include "map/projection_utm.h"
#include "map/tile_pdb.h"
#include "map/tile_tool.h"
#include "pcd/async_task.h"
#include "pcd/pcd_split.h"
#include "pcd/read_pcl.h"

#include "platform/date_time.h"

namespace geditor {

PCDSplitter::PCDSplitter() {
  move_orgin_x = 0.0;
  move_orgin_y = 0.0;
  orgin_zone = 0;
}

PCDSplitter::~PCDSplitter() {}

void PCDSplitter::SetOriginPoint(double originX, double originY, int zone) {
  move_orgin_x = originX;
  move_orgin_y = originY;
  orgin_zone = zone;
}

void DivideLayer(const PointCloud<PCLPoint> &cloud,
                 std::vector<PointCloud<PCLPoint>> &layerCloud) {
  const int seg_num[] = {4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  const int seg_size = sizeof(seg_num) / sizeof(int);
  layerCloud.resize(5);

  int seg_idx = 0;
  int start_layer = 0;
  size_t start_index = 0;

  size_t iSize = cloud.GetPointCount();
  while (start_index < iSize) {
    PCLPoint point = cloud.GetPoint(start_index++);
    layerCloud[start_layer++].PushBack(point);

    if (start_layer > seg_num[seg_idx]) {
      start_layer = 0;

      if (seg_idx + 1 < seg_size) {
        seg_idx++;
      } else {
        seg_idx = 0;
      }
    }
  }
}

bool PCDSplitter::SplitData(const PointCloud<PCLPoint> &cloud,
                            std::vector<TilePDB *> &tileArray) {
  if (orgin_zone < 1 || orgin_zone > 60) {
    LOG(ERROR) << "UTM origin is not configured";
    return false;
  }
  std::vector<PointCloud<PCLPoint>> layerCloud;
  DivideLayer(cloud, layerCloud);

  ProjectionUTM project;
  for (size_t zoom = 0; zoom < layerCloud.size(); zoom++) {
    PointCloud<PCLPoint> &cloud = layerCloud[zoom];

    size_t nsize = cloud.GetPointCount();
    for (size_t pos = 0; pos < nsize; pos++) {
      PCLPoint point = cloud.GetPoint(pos);

      float altitude = point.z;
      float intensity = point.w;

      LatLon latlon;

      double x = move_orgin_x + point.x;
      double y = move_orgin_y + point.y;
      project.CartesianToLatLon(x, y, orgin_zone, false, latlon);

      //------------------------------

      TileGrid pointGrid(zoom);
      TileTools::LatLon2Grid(latlon.lat, latlon.lon, pointGrid);

      //--------------------------------------
      TilePDB *pTile = NULL;

      int iSize = tileArray.size();
      for (int i = 0; i < iSize; i++) {
        TileGrid grid = tileArray[i]->GetTileGrid();

        if (grid == pointGrid) {
          pTile = tileArray[i];
          break;
        }
      }

      if (pTile != NULL) {
        pTile->Add(latlon, altitude, intensity, intensity);
      } else {
        TilePDB *tile = new TilePDB(pointGrid);
        tile->Add(latlon, altitude, intensity, intensity);

        tileArray.push_back(tile);
      }
    }
  }
  return true;
}

bool PCDSplitter::SplitRGBData(const PointCloud<PCLRGBPoint> &source,
                               std::vector<TilePDB *> &tileArray) {
  if (orgin_zone < 1 || orgin_zone > 60) {
    LOG(ERROR) << "UTM origin is not configured";
    return false;
  }
  std::vector<PointCloud<PCLRGBPoint>> layer_cloud(5);
  const int seg_num[] = {4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  int seg_idx = 0;
  int layer = 0;
  for (unsigned int i = 0; i < source.GetPointCount(); ++i) {
    layer_cloud[layer++].PushBack(source.GetPoint(i));
    if (layer > seg_num[seg_idx]) {
      layer = 0;
      seg_idx = (seg_idx + 1) % (sizeof(seg_num) / sizeof(seg_num[0]));
    }
  }

  ProjectionUTM project;
  for (size_t zoom = 0; zoom < layer_cloud.size(); ++zoom) {
    const auto &cloud = layer_cloud[zoom];
    for (unsigned int pos = 0; pos < cloud.GetPointCount(); ++pos) {
      const PCLRGBPoint point = cloud.GetPoint(pos);
      LatLon latlon;
      project.CartesianToLatLon(move_orgin_x + point.x,
                               move_orgin_y + point.y, orgin_zone, false,
                               latlon);
      TileGrid grid(zoom);
      TileTools::LatLon2Grid(latlon.lat, latlon.lon, grid);
      TilePDB *tile = nullptr;
      for (TilePDB *candidate : tileArray) {
        if (candidate->GetTileGrid() == grid) {
          tile = candidate;
          break;
        }
      }
      if (tile == nullptr) {
        tile = new TilePDB(grid);
        tileArray.push_back(tile);
      }
      tile->Add(latlon, point.z, point.w, point.w, point.rgb);
    }
  }
  return true;
}

bool PCDSplitter::LoadPCDFile(const char *szPDBfile, const char *filename) {
  if (lzo_init() != LZO_E_OK) {
    return false;
  }

  PCDReader<PCLPoint> pcdReader;

  if (pcdReader.loadFile(filename)) {
    std::vector<TilePDB *> tileArray;
    if (SplitData(pcdReader.cloud_, tileArray)) {
      pcdReader.Destroy();

      PDBManage database;
      if (database.Open(szPDBfile)) {
        if (!database.SetUTMZone(orgin_zone)) {
          return false;
        }
        size_t isize = tileArray.size();
        for (size_t pos = 0; pos < isize; pos++) {
          TilePDB *pTile = tileArray[pos];
          database.Save(pTile);
          delete pTile;
        }
        database.Close();
        return true;
      }
    } else {
      pcdReader.Destroy();
    }
  }

  return false;
}

}  // namespace geditor
