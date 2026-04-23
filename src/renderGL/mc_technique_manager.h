
#pragma once

#include <vector>

namespace geditor {

class RenderTechnique;

class TechniqueManager {
 public:
  TechniqueManager();

  ~TechniqueManager();

 public:
  static TechniqueManager *GetInstance();

  void Initialize();

  void AddTechnique(int dataType, RenderTechnique *renderTechnique);

  RenderTechnique *GetTechnique(int type);

 private:
  std::vector<RenderTechnique *> render_techniques_;
};

}  // namespace geditor
