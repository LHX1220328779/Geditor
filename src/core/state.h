
#pragma once

#include "algorithm/matrix44.h"
#include "algorithm/viewport.h"
#include "renderGL/mc_program.h"

namespace geditor {

class State {
 public:
  State();

  void initializeProcs();

  void ApplyProgram(const Program *program);

  void ApplyViewort(const Viewport *matrix);

  void ApplyProjectionMatrix(const Matrix4x4f *matrix);

  void ApplyModelViewMatrix(const Matrix4x4f *matrix);

 private:
  const Matrix4x4f *m_projection;
  const Matrix4x4f *m_modelView;
  const Viewport *m_viewport;
  const Program *m_program;
};
}  // namespace geditor
