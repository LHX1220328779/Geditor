
#include "core/group_node.h"

namespace geditor {

GroupNode::GroupNode() {}

GroupNode::~GroupNode() {
  for (int i = 0; i < m_children.size(); i++) {
    delete m_children[i];
  }
  m_children.clear();
}

void GroupNode::AddChild(Object *drawable) { m_children.push_back(drawable); }

int GroupNode::GetNumChildren() const {
  return static_cast<int>(m_children.size());
}

Object *GroupNode::GetChild(int index) { return m_children[index]; }

void GroupNode::RemoveAllChild() {
  for (int i = 0; i < m_children.size(); i++) {
    if (m_children[i] != nullptr) {
      delete m_children[i];
      m_children[i] = nullptr;
    }
  }

  m_children.clear();
}

}  // namespace geditor
