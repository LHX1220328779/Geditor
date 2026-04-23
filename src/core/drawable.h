
#pragma once

#include "algorithm/bound_box.h"
#include "core/object.h"
#include "core/render_info.h"

#include "renderGL/mc_buffer.h"
#include "renderGL/mc_program.h"
#include "renderGL/mc_vertex_format.h"

namespace geditor {

class Drawable : public Object {
 public:
  Drawable();

  virtual ~Drawable();

 public:
  void Draw(RenderInfo &renderInfo);

 public:
  virtual bool IsValid() const;

  virtual void DrawImplementation(RenderInfo &renderInfo) = 0;

  virtual void SetColorType(int type, int r) {}
  virtual void SetParameter(RenderInfo &renderInfo) {}

 protected:
  void DrawVertexAttr(Program *pProgram, int drawMode,
                      VertexFormat *pVertexformat, VertexBuffer *pVertexBuffer,
                      IndexBuffer *pIndexBuffer);

 protected:
  BoundBox3f m_boundbox;
};

}  // namespace geditor
