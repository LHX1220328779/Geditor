
#pragma once

#include <vector>

namespace geditor {

class RenderPass;

class RenderTechnique {
 public:
  RenderTechnique();

  ~RenderTechnique();

  void AddPass(RenderPass *pRenderPass);

  int GetPassCount() const;

  RenderPass *const &GetPass(int nIndex) const;

 private:
  std::vector<RenderPass *> m_rendPasses;
};

}  // namespace geditor
