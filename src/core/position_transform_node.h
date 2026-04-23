
#pragma once

#include "core/group_node.h"

namespace geditor {

class PositionTransformNode : public GroupNode {
 public:
  PositionTransformNode() {}

  virtual ~PositionTransformNode() {}

 public:
  void SetPosition(const V3d &pos) { m_position = pos; }

  const V3d &GetPosition() const { return m_position; }

 private:
  V3d m_position;
};

}  // namespace geditor
