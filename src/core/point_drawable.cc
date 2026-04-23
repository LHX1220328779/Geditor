#include "core/point_drawable.h"
#include "map/projection_utm.h"
#include "renderGL/gl_api.h"

namespace geditor {

PointDrawable::PointDrawable() : m_vboId(0), m_vxCount(0) {
  m_boundBox.Set(V3f(1000.0f, 1000.0f, 1000.0f),
                 V3f(-1000.0f, -1000.0f, -1000.0f));
}

PointDrawable::~PointDrawable() {
  m_PointArray.clear();
  if (m_vboId != 0) {
    DeleteVBO();
  }
}

void PointDrawable::add(float x, float y, float z, const V4f &color) {
  m_boundBox.ExpandBy(x, y, z);
  m_PointArray.push_back(PointColor(x, y, z, color));
}

bool PointDrawable::CreateVBO() {
  int nByteSize = m_PointArray.size() * sizeof(PointColor);
  char *vectexPtr = (char *)&m_PointArray[0];

  glGenBuffers(1, &m_vboId);
  glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
  glBufferData(GL_ARRAY_BUFFER, nByteSize, vectexPtr, GL_STATIC_DRAW);
  return true;
}

void PointDrawable::DeleteVBO() { glDeleteBuffers(1, &m_vboId); }

void PointDrawable::DrawImplementation(RenderInfo &renderInfo) {
  Program *pProgram = renderInfo.program_;
  int posLoc = pProgram->attrib_locs_[0];
  int colorLoc = pProgram->attrib_locs_[1];

  if (m_vboId == 0) {
    if (CreateVBO()) {
      m_vxCount = m_PointArray.size();
      // m_PointArray.clear();
      // std::vector<PointColor>(m_PointArray).swap(m_PointArray);
    } else {
      return;
    }
  }
  glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
  glVertexAttribPointer(posLoc, 3, GL_FLOAT, false, sizeof(PointColor), 0);
  glVertexAttribPointer(colorLoc, 4, GL_FLOAT, false, sizeof(PointColor),
                        (void *)sizeof(V3f));
  glEnableVertexAttribArray(posLoc);
  glEnableVertexAttribArray(colorLoc);
  glDrawArrays(GL_POINTS, 0, m_vxCount);
  glDisableVertexAttribArray(posLoc);
  glDisableVertexAttribArray(colorLoc);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void PointDrawable::SetColorType(int type, int r) {
  if (color_type_ < 0 || color_r_ < 0) {
    color_type_ = type;
    color_r_ = r;
  } else if (color_type_ != type || color_r_ != r) {
    for (auto &pc : m_PointArray) {
      pc.color = PointColor::GetColor(pc.color.w, pc.point.z, type, r);
    }
    m_vboId = 0;
    color_type_ = type;
    color_r_ = r;
  }
}
}  // namespace geditor
