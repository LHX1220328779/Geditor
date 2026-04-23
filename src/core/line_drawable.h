
#pragma once

#include "core/drawable.h"
#include "core/point3d.h"
#include "core/point_color.h"

#include <vector>
#include "algorithm/bound_box.h"

namespace geditor {

class LineDrawable : public Drawable {
 public:
  LineDrawable();

  virtual ~LineDrawable();

 public:
  int size() { return m_PointArray.size(); }

  void modify(int index, const V3f &vec);

  void add(const V3f &vec, const V3f &color);

  void add(float x, float y, float z, const V3f &color);

  void addline(int start, int count);

  void reset() { m_PointArray.clear(); }

  virtual void DrawImplementation(RenderInfo &renderInfo);

 private:
  std::vector<std::pair<int, int>> m_lines;

  std::vector<PointColor> m_PointArray;
  BoundBox3f m_boundBox;
};

}  // namespace geditor
