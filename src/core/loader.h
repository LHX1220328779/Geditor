
#pragma once

#include <algorithm>
#include <atomic>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "core/point_drawable.h"
#include "core/position_transform_node.h"
#include "core/render_leaf.h"
#include "map/pdb_manage.h"
#include "map/projection_utm.h"
#include "map/tile_tool.h"
#include "utils/color_interpolation.h"

namespace geditor {

struct Callback {
  Callback(std::map<TileGrid, PositionTransformNode *> *RenderLeaf) {
    m_RenderLeaf = RenderLeaf;
  }

  virtual void ReturnData(const TileGrid &grid,
                          PositionTransformNode *pTransformNode) const {
    if (pTransformNode) {
      //保存数据
      m_RenderLeaf->insert(
          std::pair<TileGrid, PositionTransformNode *>(grid, pTransformNode));
    }
  }
  void Clear() {
    for (auto &mr : *m_RenderLeaf) {
      delete mr.second;
    }
    m_RenderLeaf->clear();
  }

 private:
  std::map<TileGrid, PositionTransformNode *> *m_RenderLeaf;
};

class Loader {
 public:
  Loader() : m_callback(NULL), m_database(NULL) {}

  ~Loader() {
    if (m_callback) {
      delete m_callback;
      m_callback = NULL;
    }
  }

  void SetDataFile(PDBManage *database) { m_database = database; }

  void setCallback(Callback *callback) { m_callback = callback; }

  void start() {
    running_.store(true);
    thread_ = std::make_shared<std::thread>(std::bind(&Loader::run, this));
  }

  int cancel() {
    if (running_) {
      running_.store(false);
      thread_->join();
      thread_ = nullptr;
      if (m_callback) {
        m_callback->Clear();
      }
      ClearCache();
    }
    return 0;
  }

  //---------------------------------------------------

  PointDrawable *load(TilePDB *tile, V3d &vCenter) {
    ProjectionUTM projectionUTM;

    if (m_database->Read(tile)) {
      UTMPoint utmCenter;
      GPSPoint gpsCenter = tile->GetCenterPoint();
      projectionUTM.LatLonToCartesian(gpsCenter.latlon.lat,
                                      gpsCenter.latlon.lon, utmCenter);

      vCenter = V3d(utmCenter.x, utmCenter.y, 0.0);

      PointDrawable *pPtDrawable = new PointDrawable();
      pPtDrawable->SetSourceColorMode(use_source_rgb_);
      pPtDrawable->SetColorType(color_type_, color_r_);

      for (int i = 0; i < tile->point_cloud_.size(); i++) {
        LatLon latlon = tile->point_cloud_[i].latlon;
        float altitude = tile->point_cloud_[i].altitude;
        float intensity = tile->point_cloud_[i].intensity;

        UTMPoint utmPt;
        projectionUTM.LatLonToCartesian(latlon.lat, latlon.lon, utmPt);

        double x = (utmPt.x - utmCenter.x);
        double y = (utmPt.y - utmCenter.y);
        float z = altitude - gpsCenter.altitude;

        V4f clrf = use_source_rgb_
                       ? PointColor::FromRGB(tile->point_cloud_[i].rgb,
                                             intensity)
                       : PointColor::GetColor(intensity, z, color_type_,
                                              color_r_);

        pPtDrawable->add((float)x, (float)y, (float)z, clrf);
      }

      return pPtDrawable;
    }
    return NULL;
  }

  //--------------------------------------

  void Clear() {
    std::unique_lock<std::mutex> lock(m_threadMutex);
    m_listTile.clear();
  }

  void Add(const std::vector<TileGrid> tileArray) {
    std::unique_lock<std::mutex> lock(m_threadMutex);
    for (int i = 0; i < tileArray.size(); i++) {
      std::list<TileGrid>::iterator it_find =
          std::find(m_listTile.begin(), m_listTile.end(), tileArray[i]);
      if (it_find == m_listTile.end()) {
        m_listTile.push_back(tileArray[i]);
      }
    }
  }

  void Add(const TileGrid &grid) {
    std::unique_lock<std::mutex> lock(m_threadMutex);
    m_listTile.push_front(grid);
  }

  void run() {
    while (running_) {
      TileGrid grid;
      {
        std::unique_lock<std::mutex> lock(m_threadMutex);
        if (!m_listTile.empty()) {
          grid = m_listTile.front();
          m_listTile.pop_front();
        }
      }

      if (grid.x == 0 && grid.y == 0) {
        usleep(100);
        continue;
      }

      std::map<TileGrid, PointDrawable *>::iterator iter = m_cache.find(grid);
      if (iter == m_cache.end()) {
        V3d vCenter;
        TilePDB tile(grid);

        PointDrawable *pDrawable = load(&tile, vCenter);
        if (pDrawable) {
          m_cache.insert(std::pair<TileGrid, PointDrawable *>(grid, pDrawable));
          PositionTransformNode *pTransformNode = new PositionTransformNode();
          pTransformNode->SetPosition(vCenter);
          RenderLeaf *pRenderLeaf = new RenderLeaf();
          pRenderLeaf->SetDrawable(pDrawable);
          pTransformNode->AddChild(pRenderLeaf);
          if (m_callback) {
            m_callback->ReturnData(grid, pTransformNode);
          }
        } else {
          m_cache.insert(std::pair<TileGrid, PointDrawable *>(grid, NULL));
        }
      }
    }
    running_.store(false);
  }
  void SetColorType(int type, int r) {
    color_type_ = type;
    color_r_ = r;
  }
  void SetUseSourceRGB(bool enabled) { use_source_rgb_ = enabled; }
  void ClearCache() {
    // for (auto &mr : m_cache) {
    //   if (mr.second) delete mr.second;
    // }
    m_cache.clear();
  }

 private:
  PDBManage *m_database = nullptr;
  Callback *m_callback = nullptr;
  std::mutex m_threadMutex;  //互斥对象
  std::list<TileGrid> m_listTile;
  std::map<TileGrid, PointDrawable *> m_cache;

  std::shared_ptr<std::thread> thread_ = nullptr;
  std::atomic<bool> running_;
  int color_type_ = 2;
  int color_r_ = 6;
  bool use_source_rgb_ = false;
};

}  // namespace geditor
