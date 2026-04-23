
#pragma once

#include "algorithm/common.h"

namespace geditor {

class VectorStyle {
 public:
  VectorStyle();

  virtual ~VectorStyle();
};

class PolyLineSytle : public VectorStyle {
 public:
  PolyLineSytle();

  virtual ~PolyLineSytle();

  V4f GetBackgroundColor() { return m_bkColor; }

  V4f GetLineColor() { return m_lineColor; }

  V3f GetVertexColor() { return m_vertexColor; }

  V3f GetArrowColor() { return m_arrowColor; }

  void SetBackgroundColor(const V4f &bkColor) { m_bkColor = bkColor; }

  void SetLineColor(const V4f &lineColor) { m_lineColor = lineColor; }

  void SetVertexColor(const V3f &vertexColor) { m_vertexColor = vertexColor; }

  void SetArrowColor(const V3f &arrowColor) { m_arrowColor = arrowColor; }

  V3f GetKeyVertexColor() { return m_keyVertexColor; }

  void SetKeyVertexColor(const V3f &keyVertexColor) {
    m_keyVertexColor = keyVertexColor;
  }

 private:
  V4f m_bkColor;
  V4f m_lineColor;
  V3f m_vertexColor;
  V3f m_arrowColor;
  V3f m_keyVertexColor;
};

class PolygonSytle : public VectorStyle {
 public:
  PolygonSytle();

  virtual ~PolygonSytle();

  V4f GetLineColor() { return m_lineColor; }

  V3f GetVertexColor() { return m_vertexColor; }

  V3f GetArrowColor() { return m_arrowColor; }

  void SetLineColor(const V4f &lineColor) { m_lineColor = lineColor; }

  void SetVertexColor(const V3f &vertexColor) { m_vertexColor = vertexColor; }

  void SetArrowColor(const V3f &arrowColor) { m_arrowColor = arrowColor; }

  V3f GetKeyVertexColor() { return m_keyVertexColor; }

  void SetKeyVertexColor(const V3f &keyVertexColor) {
    m_keyVertexColor = keyVertexColor;
  }

 private:
  V4f m_lineColor;       //������ɫ
  V3f m_vertexColor;     //������ɫ
  V3f m_arrowColor;      //��ͷ��ɫ
  V3f m_keyVertexColor;  //������ɫ
};

class PointSytle : public VectorStyle {
 public:
  PointSytle();

  virtual ~PointSytle();

  V3f GetLineColor() { return m_lineColor; }

  V3f GetVertexColor() { return m_vertexColor; }

  V3f GetArrowColor() { return m_arrowColor; }

  void SetLineColor(const V3f &lineColor) { m_lineColor = lineColor; }

  void SetVertexColor(const V3f &vertexColor) { m_vertexColor = vertexColor; }

  void SetArrowColor(const V3f &arrowColor) { m_arrowColor = arrowColor; }

  V3f GetKeyVertexColor() { return m_keyVertexColor; }

  void SetKeyVertexColor(const V3f &keyVertexColor) {
    m_keyVertexColor = keyVertexColor;
  }

 private:
  V3f m_lineColor;       //������ɫ
  V3f m_vertexColor;     //������ɫ
  V3f m_arrowColor;      //��ͷ��ɫ
  V3f m_keyVertexColor;  //������ɫ
};

}  // namespace geditor
