
#pragma once

#include "core/drawable.h"
#include "core/point3d.h"
#include "core/point_color.h"

#include "algorithm/bound_box.h"

#include <vector>

namespace geditor {

class RenderLayout;

class VertexFormat;

class VertexBuffer;

class IndexBuffer;

class PointSetDrawable : public Drawable {
 public:
  PointSetDrawable();

  virtual ~PointSetDrawable();

 public:
  void Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
              IndexBuffer *vIBuffer);

  BoundBox3f getBound() { return bound_box_; }

  virtual void DrawImplementation(RenderInfo &renderInfo);

 private:
  BoundBox3f bound_box_;
  RenderLayout *render_layout_;
  VertexFormat *format_;
  VertexBuffer *buffer_;
  IndexBuffer *ibuffer_;
};
}  // namespace geditor
