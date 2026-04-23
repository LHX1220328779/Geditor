
#pragma once

#include <vector>
#include "core/node.h"

namespace geditor {

class Object;

class Drawable;

class GroupNode : public Node {
 public:
  GroupNode();

  virtual ~GroupNode();

 public:
  void AddChild(Object *drawable);

  int GetNumChildren() const;

  Object *GetChild(int index);

  void RemoveAllChild();

 private:
  std::vector<Object *> m_children;
};

}  // namespace geditor
