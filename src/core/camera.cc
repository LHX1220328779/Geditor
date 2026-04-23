
#include "core/camera.h"

namespace geditor {

void Camera::SetViewMatrix(const Matrix4x4f &viewMatrix) {
  m_viewMatrix = viewMatrix;
  m_vpMatrix = m_viewMatrix * m_projMatrix;
}

Matrix4x4f Camera::GetViewMatrix() const { return m_viewMatrix; }

void Camera::SetProjectionMatrix(const Matrix4x4f &projMatrix) {
  m_projMatrix = projMatrix;
  m_vpMatrix = m_viewMatrix * m_projMatrix;
}

Matrix4x4f Camera::GetProjectionMatrix() const { return m_projMatrix; }

Matrix4x4f Camera::GetViewProjMatrix() const { return m_vpMatrix; }

Viewport Camera::GetViewport() const { return m_viewport; }

void Camera::SetViewport(const Viewport &viewport) { m_viewport = viewport; }

void Camera::SetPostion(const V3d &positon) {
  m_postion = positon;
  Matrix4x4f rotateMatrix = Matrix4x4f::MakeRotationY(Mathf::ToRadians(180));
  m_viewMatrix = Matrix4x4f::LookAt(
      V3f(0.0f, 0.0f, 0.0f), V3f(0.0f, 0.0f, -1.0f), V3f(0.0f, 1.0f, 0.0f));

  m_vpMatrix = m_viewMatrix * m_projMatrix;
}

void Camera::MoveCamera(const V3d &dir, double distance) {
  V3d vDir = Normalize(dir);
  V3d vTmp = vDir * distance;
  m_postion += vTmp;
}

V3d Camera::GetPostion() const { return m_postion; }

void StereoCamera::SetPerspective(float fovy, float aspect, float zn,
                                  float zf) {
  m_projMatrix =
      Matrix4x4f::Perspective(Mathf::ToRadians(fovy), aspect, zn, zf);
  m_vpMatrix = m_viewMatrix * m_projMatrix;
}

void PlanCamera::SetOrtho(float aspect, float zn, float zf) {
  m_projMatrix = Matrix4x4f::Ortho(-aspect, aspect, -1.0, 1.0f, zn, zf);
  m_vpMatrix = m_viewMatrix * m_projMatrix;
}

}  // namespace geditor
