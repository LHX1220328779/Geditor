
#include "renderGL/mc_render_technique.h"
#include <cassert>

namespace geditor {
RenderTechnique::RenderTechnique() {}

RenderTechnique::~RenderTechnique() {}

void RenderTechnique::AddPass(RenderPass *pRenderPass) {
  m_rendPasses.push_back(pRenderPass);
}

int RenderTechnique::GetPassCount() const { return m_rendPasses.size(); }

RenderPass *const &RenderTechnique::GetPass(int nIndex) const {
  assert(nIndex >= 0 && nIndex < m_rendPasses.size());
  return m_rendPasses[nIndex];
}
}  // namespace geditor
