
#include "core/edge_drawable.h"
#include "map/projection_utm.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_render_layout.h"
#include "renderGL/mc_vertex_format.h"

namespace geditor {

EgdeDrawable::EgdeDrawable() {
  m_vFormat = 0;
  m_vBuffer = 0;
  m_vIBuffer = 0;

  m_pRenderLayout = new RenderLayout();

  m_boundBox.Reset();
}

EgdeDrawable::~EgdeDrawable() {
  if (m_pRenderLayout) {
    delete m_pRenderLayout;
  }
}

void EgdeDrawable::Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
                          IndexBuffer *vIBuffer) {
  m_pRenderLayout->SetTopologyType(RenderLayout::TT_TriangleList);
  m_pRenderLayout->BindVertexStream(vBuffer, vFormat);
  m_pRenderLayout->BindIndexStream(vIBuffer);

  if (m_vFormat) {
    delete m_vFormat;
  }

  if (m_vBuffer) {
    delete m_vBuffer;
  }

  if (m_vIBuffer) {
    delete m_vIBuffer;
  }

  m_vFormat = vFormat;
  m_vBuffer = vBuffer;
  m_vIBuffer = vIBuffer;
}

void EgdeDrawable::DrawImplementation(RenderInfo &renderInfo) {
  Program *pProgram = renderInfo.program_;
  if (pProgram != NULL) {
    DrawVertexAttr(pProgram, GL_LINES, m_vFormat, m_vBuffer, m_vIBuffer);
  }
}

}  // namespace geditor
