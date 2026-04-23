
#pragma once

#include "core/map_feature.h"
#include "core/point3d.h"
#include "core/point_color.h"

namespace geditor {

class MapFeature;

struct PickupResult {
  MapFeature *pFeatureObject;
  Point3d ptNearPoint;
  int nSegmentIdx;
  int nKeyPoint;
  double dDistance;

  PickupResult() {
    pFeatureObject = 0;
    ptNearPoint;
    nSegmentIdx = -1;
    nKeyPoint = -1;
    dDistance = 10000.0;
  }
};
enum LayerType {
  LT_PDB = 0,
  LT_BOUNDARY,
  LT_LANE,
  LT_ROADAREA,
  LT_SIGN,
  LT_FUNAREA,
  LT_TOPO,
  LT_TRA,
  ////////
  LT_END
};
class Layer;
class LayerHelper {
 public:
  LayerHelper() : layers_(LT_END + 1, nullptr) {}
  void SetLayer(Layer *layer, LayerType type) { layers_[type] = layer; }
  Layer *GetLayer(LayerType type) { return layers_[type]; }
  Layer *SignLayer() { return layers_[LT_SIGN]; }
  Layer *BoundaryLayer() { return layers_[LT_BOUNDARY]; }
  Layer *RoadareaLayer() { return layers_[LT_ROADAREA]; }
  Layer *FunareaLayer() { return layers_[LT_FUNAREA]; }
  std::vector<Layer *> &layers() { return layers_; }
  Layer *GetLayerByFeature(int feat_type) {
    auto type = MapFeature::MapFeatureType(feat_type);
    switch (type) {
      case MapFeature::MFT_LANE_SEG:
        return layers_[LT_LANE];
      case MapFeature::MFT_BOUNDARY:
        return layers_[LT_BOUNDARY];
      case MapFeature::MFT_ROAD_AREA:
        return layers_[LT_ROADAREA];
      case MapFeature::MFT_SIGNBORAD:
        return layers_[LT_SIGN];
      case MapFeature::MFT_JOB_AREA:
        return layers_[LT_FUNAREA];
      case MapFeature::MFT_POINT_ELEMENT:
        return layers_[LT_TOPO];
      default:
        return nullptr;
    }
  }

 private:
  std::vector<Layer *> layers_;
};

template <typename C>
MapFeature *GetFeatureVec(int idx, const C &vec) {
  for (int i = 0; i < vec.size(); i++)
    if (vec[i]->GetUniqueID() == idx)
      if (vec[i]->deleted())
        return nullptr;
      else
        return vec[i];
  return nullptr;
}

class Layer {
 public:
  Layer(LayerType id);

  virtual ~Layer();

 public:
  virtual void ClearLayer() = 0;

  virtual void AddMapFeature(MapFeature *feature) = 0;

  virtual bool DeleteMapFeature(MapFeature *feature) = 0;

  virtual bool PickupObject(const Point3d &mousePoint, double tolerance,
                            PickupResult &result) {
    return false;
  };
  virtual MapFeature *GetFeature(int idx) { return nullptr; }

  void SetVisible(bool bVisible);

  bool IsVisible() const;

  bool IsEnableEdit() const;
  LayerType GetLayerType() { return layer_id_; }

 protected:
  bool m_bVisible;
  bool m_bEnableEdit;
  LayerType layer_id_;
};

}  // namespace geditor
