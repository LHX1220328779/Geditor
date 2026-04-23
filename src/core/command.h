
#pragma once

#include "core/layer.h"
#include "core/map_feature.h"

namespace geditor {

class IMapCommand {
 public:
  virtual void Execute() = 0;

  virtual void Undo() = 0;
};

class DeleteCommand : public IMapCommand {
 public:
  DeleteCommand(const std::vector<MapFeature *> &listObject) {
    m_listObject = listObject;
  }

  virtual void Execute() {
    for (int i = 0; i < m_listObject.size(); i++) {
      MapFeature *pFeature = m_listObject[i];

      Layer *pLayer = pFeature->GetMapLayer();
      pLayer->DeleteMapFeature(pFeature);
      pFeature->Delete(true);
    }
  }

  virtual void Undo() {
    for (int i = 0; i < m_listObject.size(); i++) {
      MapFeature *pFeature = m_listObject[i];

      Layer *pLayer = pFeature->GetMapLayer();
      pLayer->AddMapFeature(pFeature);
      pFeature->Delete(false);
    }
  }

 private:
  std::vector<MapFeature *> m_listObject;
};

class ReverseCommand : public IMapCommand {
 public:
  ReverseCommand(const std::vector<MapFeature *> &listObject) {
    m_listObject = listObject;
  }

  virtual void Execute() {
    for (int i = 0; i < m_listObject.size(); i++) {
      MapFeature *pFeature = m_listObject[i];

      Geometry *pPolyline = pFeature->GetGeometry();
      pPolyline->ReverseVertex();
    }
  }

  virtual void Undo() {
    for (int i = 0; i < m_listObject.size(); i++) {
      MapFeature *pFeature = m_listObject[i];

      Geometry *pPolyline = pFeature->GetGeometry();
      pPolyline->ReverseVertex();
    }
  }

 private:
  std::vector<MapFeature *> m_listObject;
};

class InsertPointCommand : public IMapCommand {
 public:
  InsertPointCommand(MapFeature *feature, int index, const Point3d &point)
      : m_feature(feature), m_index(index), m_point(point) {}

  virtual void Execute() {
    Geometry *pline = m_feature->GetGeometry();
    pline->InsertVertex(m_index, m_point);
  }

  virtual void Undo() {
    Geometry *pline = m_feature->GetGeometry();
    pline->RemoveVertex(m_index);
  }

 private:
  MapFeature *m_feature;
  int m_index;
  Point3d m_point;
};

//------------------------------------------------------

class DrawCommand : public IMapCommand {
 public:
  DrawCommand(Layer *mapLayer, MapFeature *feature)
      : m_feature(feature), m_mapLayer(mapLayer) {}

  virtual void Execute() { m_mapLayer->AddMapFeature(m_feature); }

  virtual void Undo() { m_mapLayer->DeleteMapFeature(m_feature); }

 private:
  MapFeature *m_feature;
  Layer *m_mapLayer;
};

//-------------------------------------------------------

class MoveObjectCommand : public IMapCommand {
 public:
  MoveObjectCommand(const std::vector<MapFeature *> &listObject,
                    const Point3d &orginPoint) {
    m_listObject = listObject;
    m_orginPoint = orginPoint;
  }

  void Update(const Point3d &newPoint) {
    m_vecMove.x = newPoint.x - m_orginPoint.x;
    m_vecMove.y = newPoint.y - m_orginPoint.y;
    m_vecMove.z = newPoint.z - m_orginPoint.z;

    m_sumMove.x -= m_vecMove.x;
    m_sumMove.y -= m_vecMove.y;
    m_sumMove.z -= m_vecMove.z;

    m_orginPoint = newPoint;
  }

  virtual void Execute() {
    for (int i = 0; i < m_listObject.size(); i++) {
      MapFeature *pFeature = m_listObject[i];

      Geometry *pPolyline = pFeature->GetGeometry();
      pPolyline->MoveGeometry(m_vecMove);
    }
  }

  virtual void Undo() {
    for (int i = 0; i < m_listObject.size(); i++) {
      MapFeature *pFeature = m_listObject[i];

      Geometry *pPolyline = pFeature->GetGeometry();
      pPolyline->MoveGeometry(m_sumMove);
    }
    m_vecMove.x = -m_sumMove.x;
    m_vecMove.y = -m_sumMove.y;
    m_vecMove.z = -m_sumMove.z;
  }

 private:
  std::vector<MapFeature *> m_listObject;
  Point3d m_orginPoint;

  Point3d m_vecMove;
  Point3d m_sumMove;
};

//-------------------------------------------------------

class MovePointCommand : public IMapCommand {
 public:
  MovePointCommand(MapFeature *feature, int keyPoint) {
    m_feature = feature;
    m_keyPoint = keyPoint;

    Geometry *pPolyline = m_feature->GetGeometry();
    m_orginPoint = pPolyline->GetVertex(keyPoint);
  }

  void Update(const Point3d &newPoint) { m_newPoint = newPoint; }

  virtual void Execute() {
    Geometry *pPolyline = m_feature->GetGeometry();
    pPolyline->MoveVertex(m_newPoint, m_keyPoint);
  }

  virtual void Undo() {
    Geometry *pPolyline = m_feature->GetGeometry();
    pPolyline->MoveVertex(m_orginPoint, m_keyPoint);
  }

 private:
  MapFeature *m_feature;
  int m_keyPoint;

  Point3d m_orginPoint;
  Point3d m_newPoint;
};
}  // namespace geditor
