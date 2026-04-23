
#pragma once

#include "algorithm/matrix44.h"
#include "core/camera.h"
#include "core/layer.h"
#include "core/position_transform_node.h"
#include "core/render_info.h"
#include "loader.h"
#include "map/tile_tool.h"

#include <map>
#include <vector>

namespace geditor {

class RenderLeaf;

class Drawable;

class Loader;

class PDBManage;

class PDBLayer : public Layer {
 public:
  PDBLayer();

  virtual ~PDBLayer();

 public:
  bool GetBoundary(V3d &minVec, V3d &maxVec);

  void SetDataSource(const char *szName);

  void CloseDataSource();

  void SetPointCloudFilter(float minVal, float maxVal);

  void GetPointCloudFilter(float &minVal, float &maxVal);

  void SetPointHighFilter(float minVal, float maxVal);

  void GetPointHighFilter(float &minVal, float &maxVal);

  void SetColorType(int type, int r);
  void GetColorType(int &type, int &r);

  //��ӵ�ͼҪ��
  void AddMapFeature(MapFeature *feature){};

  bool DeleteMapFeature(MapFeature *polyline) { return true; };

  void ClearLayer(){};

 public:
  void Cull(double minX, double minY, double maxX, double maxY, float scale);

  void Update(const Matrix4x4f &svMatrix, Camera *pCamera);

  void Draw(RenderInfo &rendinfo);

 private:
  float m_minValue;
  float m_maxValue;
  float m_minHigh;
  float m_maxHigh;
  int color_type_ = 0;
  int color_r_ = 6;

  // Loader *m_loader;
  Loader *m_loader;
  PDBManage *m_database;

  std::map<TileGrid, PositionTransformNode *> m_RenderLeaf;

  //��ǰ�ɼ���tile
  std::vector<TileGrid> m_tileArray;

  //��ǰ�ɼ���tile
  std::vector<PositionTransformNode *> m_RenderTile;
};
}  // namespace geditor
