
#pragma once

#include "algorithm/matrix44.h"
#include "algorithm/viewport.h"

namespace geditor {

class Camera {
 public:
  void SetPostion(const V3d &positon);

  V3d GetPostion() const;

  void MoveCamera(const V3d &dir, double distance);

  void SetProjectionMatrix(const Matrix4x4f &projMatrix);

  void SetViewMatrix(const Matrix4x4f &viewMatrix);

  Matrix4x4f GetViewMatrix() const;

  Matrix4x4f GetProjectionMatrix() const;

  Matrix4x4f GetViewProjMatrix() const;

  void SetViewport(const Viewport &viewport);

  Viewport GetViewport() const;

 protected:
  Matrix4x4f m_viewMatrix;  //�۲����
  Matrix4x4f m_invViewMat;

  Matrix4x4f m_projMatrix;  //ͶӰ����
  Matrix4x4f m_invProjMat;

  Matrix4x4f m_vpMatrix;
  Viewport m_viewport;
  V3d m_postion;
};

//------------------------------

class StereoCamera : public Camera {
 public:
  void SetPerspective(float fovy, float aspect, float zn, float zf);
};

class PlanCamera : public Camera {
 public:
  void SetOrtho(float aspect, float zn, float zf);

 private:
};

}  // namespace geditor
