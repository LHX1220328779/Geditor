
#include "core/wline_drawable.h"
#include "map/projection_utm.h"
#include "renderGL/gl_api.h"

#include "renderGL/mc_vertex_buffer_accessor.h"

namespace geditor {

WLineDrawable::WLineDrawable() : m_vboId(0), m_vxCount(0) {
  m_pVertexformat = NULL;
  m_pVertexBuffer = NULL;
  m_pIndexBuffer = NULL;
}

WLineDrawable::~WLineDrawable() {
  if (m_pVertexformat != NULL) {
    delete m_pVertexformat;
    m_pVertexformat = NULL;
  }

  if (m_pVertexBuffer != NULL) {
    delete m_pVertexBuffer;
    m_pVertexBuffer = NULL;
  }

  if (m_pIndexBuffer != NULL) {
    delete m_pIndexBuffer;
    m_pIndexBuffer = NULL;
  }
}

void WLineDrawable::add(float x, float y, float z, const V3f &color) {
  m_boundBox.ExpandBy(x, y, z);
  V4f vClr(color[0], color[1], color[2], 1.0);
  m_PointArray.push_back(PointColor(x, y, z, vClr));
}

void WLineDrawable::add(const V3f &vec, const V3f &color) {
  m_boundBox.ExpandBy(vec[0], vec[1], vec[2]);
  V4f vClr(color[0], color[1], color[2], 1.0);
  m_PointArray.push_back(PointColor(vec, vClr));
}

void WLineDrawable::remove(int index) {
  if (index < m_PointArray.size()) {
    m_PointArray.erase(m_PointArray.begin() + index);
  }
}

void WLineDrawable::modify(int index, const V3f &vec) {
  if (index < m_PointArray.size()) {
    m_boundBox.ExpandBy(vec[0], vec[1], vec[2]);

    m_PointArray[index].point = vec;
  }
}

bool WLineDrawable::CreateVBO() {
  int nByteSize = m_PointArray.size() * sizeof(PointColor);
  char *vectexPtr = (char *)&m_PointArray[0];

  glGenBuffers(1, &m_vboId);

  glBindBuffer(GL_ARRAY_BUFFER, m_vboId);
  glBufferData(GL_ARRAY_BUFFER, nByteSize, vectexPtr, GL_STATIC_DRAW);
  return true;
}

void WLineDrawable::DeleteVBO() { glDeleteBuffers(1, &m_vboId); }

void WLineDrawable::DrawImplementation(RenderInfo &renderInfo) {
  Program *pProgram = renderInfo.program_;
  int posLoc = pProgram->attrib_locs_[0];
  int clrLoc = pProgram->attrib_locs_[1];
  int norLoc = pProgram->attrib_locs_[2];

  UniformParam *param = pProgram->GetUniformParam();
  param->SetParameter(2, m_lineParam);

  if (m_pVertexformat != NULL && m_pVertexBuffer != NULL &&
      m_pIndexBuffer != NULL) {
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    VertexBufferAccessor vba(m_pVertexformat, m_pVertexBuffer);

    char *pData = m_pVertexBuffer->GetData();

    glVertexAttribPointer(posLoc, 3, GL_FLOAT, false,
                          m_pVertexformat->GetStride(),
                          (void *)&(vba.Position<V3f>(0)));
    glVertexAttribPointer(norLoc, 3, GL_FLOAT, false,
                          m_pVertexformat->GetStride(),
                          (void *)&(vba.Normal<V3f>(0)));
    glVertexAttribPointer(clrLoc, 4, GL_FLOAT, false,
                          m_pVertexformat->GetStride(),
                          (void *)&(vba.Color<V4f>(0, 0)));

    glEnableVertexAttribArray(posLoc);
    glEnableVertexAttribArray(norLoc);
    glEnableVertexAttribArray(clrLoc);

    glDrawElements(GL_TRIANGLES, m_pIndexBuffer->GetNumElements(),
                   GL_UNSIGNED_INT, m_pIndexBuffer->GetData());

    glDisableVertexAttribArray(posLoc);
    glDisableVertexAttribArray(norLoc);
    glDisableVertexAttribArray(clrLoc);
  }
}

}  // namespace geditor
