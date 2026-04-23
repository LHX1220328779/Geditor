
#pragma once

#include "algorithm/bound_box.h"
#include "core/drawable.h"
#include "core/point3d.h"
#include "core/point_color.h"

#include <vector>

namespace geditor {

class RenderLayout;

class VertexFormat;

class VertexBuffer;

class IndexBuffer;

class EgdeDrawable : public Drawable {
 public:
  EgdeDrawable();

  virtual ~EgdeDrawable();

 public:
  void Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
              IndexBuffer *vIBuffer);

  BoundBox3f getBound() { return m_boundBox; }

  virtual void DrawImplementation(RenderInfo &renderInfo);

 private:
  BoundBox3f m_boundBox;
  RenderLayout *m_pRenderLayout;
  VertexFormat *m_vFormat;
  VertexBuffer *m_vBuffer;
  IndexBuffer *m_vIBuffer;
};

}  // namespace geditor
