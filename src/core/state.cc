
#include <cstdio>

#include "core/state.h"
#include "renderGL/gl_api.h"

namespace geditor {
State::State()
    : m_projection(NULL),
      m_modelView(NULL),
      m_viewport(NULL),
      m_program(NULL) {}

void State::initializeProcs() {}

void State::ApplyViewort(const Viewport *viewport) {
  // if (m_viewport != viewport)
  {
    if (viewport) {
      m_viewport = viewport;
    }
    glViewport(viewport->X(), viewport->Y(), viewport->Width(),
               viewport->Height());
  }
}

void State::ApplyProgram(const Program *program) {
  if (m_program != program) {
    if (program) {
      m_program = program;
    }
    glUseProgram(program->GetHandle());
  }
}

void State::ApplyProjectionMatrix(const Matrix4x4f *matrix) {
  if (m_projection != matrix) {
    if (matrix) {
      m_projection = matrix;
    }
    int prjLoc = 0;
    glUniformMatrix4fv(prjLoc, 1, GL_FALSE, &matrix->mat[0]);
  }
}

void State::ApplyModelViewMatrix(const Matrix4x4f *matrix) {
  if (m_modelView != matrix) {
    if (matrix) {
      m_modelView = matrix;
    }
    int modelLoc = 1;
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, &matrix->mat[0]);
  }
}
}  // namespace geditor
