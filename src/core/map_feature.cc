
#include "core/map_feature.h"
#include "core/geometry.h"

namespace geditor {

MapFeature::MapFeature(int type)
    : m_uniqueId(0),
      m_pGeometry(nullptr),
      m_mapLayer(nullptr),
      m_type(type),
      m_bSelected(false) {}

MapFeature::~MapFeature() {
  if (m_pGeometry) {
    delete m_pGeometry;
    m_pGeometry = nullptr;
  }
}

int MapFeature::GetType() const { return m_type; }

void MapFeature::SetUniqueID(int uniqueId) { m_uniqueId = uniqueId; }

int MapFeature::GetUniqueID() const { return m_uniqueId; }

void MapFeature::SetGeometry(Geometry *pGeometry) {
  if (m_pGeometry != nullptr) {
    delete m_pGeometry;
  }
  m_pGeometry = pGeometry;
}

Geometry *MapFeature::GetGeometry() const { return m_pGeometry; }

void MapFeature::SetMapLayer(Layer *pMapLayer) { m_mapLayer = pMapLayer; }

Layer *MapFeature::GetMapLayer() const { return m_mapLayer; }

}  // namespace geditor