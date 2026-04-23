
#pragma once

#include <vector>
#include "core/state.h"

namespace geditor {

class RenderInfo {
 public:
  RenderInfo() {}

  void SetState(State *state) { state_ = state; }

  State *GetState() { return state_; }

 private:
  State *state_ = nullptr;

 public:
  Program *program_ = nullptr;
};

}  // namespace geditor
