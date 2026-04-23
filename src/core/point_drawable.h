
#pragma once

#include <vector>
#include "algorithm/bound_box.h"
#include "core/drawable.h"
#include "core/point3d.h"
#include "core/point_color.h"

namespace geditor {

class PointDrawable : public Drawable {
 public:
  PointDrawable();

  virtual ~PointDrawable();

 public:
  void add(float x, float y, float z, const V4f &color);

  BoundBox3f getBound() { return m_boundBox; }

  virtual void DrawImplementation(RenderInfo &renderInfo);

  virtual void SetColorType(int type, int r);

 private:
  bool CreateVBO();

  void DeleteVBO();

 private:
  std::vector<PointColor> m_PointArray;
  BoundBox3f m_boundBox;

  unsigned int m_vboId;
  unsigned int m_vxCount;

  int color_type_ = -1;
  int color_r_ = -1;
};

}  // namespace geditor
