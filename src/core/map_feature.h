
#pragma once

namespace geditor {

class Geometry;

class Layer;

class MapFeature {
 public:
  enum MapFeatureType {
    MFT_UNKOWN_TYPE = 0,
    MFT_LANE_SEG = 1,
    MFT_ROAD_SEG = 2,
    MFT_BOUNDARY = 3,
    MFT_ROAD_AREA = 4,
    MFT_SIGNBORAD = 5,
    MFT_JOB_AREA = 6,
    MFT_POINT_ELEMENT = 7,
  };

 public:
  MapFeature(int type);

  virtual ~MapFeature();

 public:
  int GetType() const;

  void SetUniqueID(int uniqueId);

  int GetUniqueID() const;

  void SetGeometry(Geometry *pGeometry);

  Geometry *GetGeometry() const;

  void SetMapLayer(Layer *pMapLayer);

  Layer *GetMapLayer() const;

  virtual void SetSelectedState(bool selected) { m_bSelected = selected; };

  virtual void SetHighlightState(bool high) { m_bHighlight = high; };

  virtual void SetHighlightPoint(int index){};
  virtual int GetHighlightPoint() { return -1; }

  bool IsSelectedState() const { return m_bSelected; };

  bool selected() { return m_bSelected; }
  bool highlighted() { return m_bHighlight; }

  void Delete(bool del) { m_deleted = del; }

  bool deleted() { return m_deleted; }

  virtual void SetChanged(bool change) {}

 protected:
  int m_type;

  int m_uniqueId;

  Geometry *m_pGeometry;

  bool m_bSelected = false;

  bool m_bHighlight = false;

  Layer *m_mapLayer;

  bool m_deleted = false;
};

}  // namespace geditor
