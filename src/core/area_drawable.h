
#pragma once

#include "core/drawable.h"
#include "core/point3d.h"
#include "core/point_color.h"

#include "algorithm/bound_box.h"

#include "renderGL/mc_buffer.h"
#include "renderGL/mc_hdw_vertex_buffer.h"
#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"

#include <vector>

namespace geditor {

class AreaDrawable : public Drawable {
 public:
  AreaDrawable();

  virtual ~AreaDrawable();

 public:
  virtual void DrawImplementation(RenderInfo &renderInfo);

 private:
  BoundBox3f m_boundBox;

 public:
  VertexFormat *m_pVertexformat;
  VertexBuffer *m_pVertexBuffer;
  IndexBuffer *m_pIndexBuffer;
};

}  // namespace geditor
