
#include "core/arc_ball.h"
#include "algorithm/common.h"

namespace geditor {

TrackBall::TrackBall() : trackball_size_(0.8f) {}

TrackBall::~TrackBall() {}

void TrackBall::LButtonDown(float x, float y) {
  start_x_ = GetXnormalized(x);
  start_y_ = GetYnormalized((viewport_.Height() - y));
}

void TrackBall::MouseMove(float x, float y, float &dirX, float &dirY) {}

void TrackBall::MouseRotate(float x, float y) {
  end_x_ = GetXnormalized(x);
  end_y_ = GetYnormalized((viewport_.Height() - y));

  RotateTrackball(end_x_, end_y_, start_x_, start_y_);

  start_x_ = end_x_;
  start_y_ = end_y_;
}

void TrackBall::SetViewport(const Viewport &viewport) { viewport_ = viewport; }

void TrackBall::SetTransformation(const V3f &eye, const V3f &center,
                                  const V3f &up) {
  V3f lv(center - eye);

  V3f f(lv);
  f.normalize();
  V3f s = f.cross(up);
  s.normalize();
  V3f u = s.cross(f);
  u.normalize();

  Matrix4x4d rotation_matrix(s.x, u.x, -f.x, 0.0, s.y, u.y, -f.y, 0.0, s.z, u.z,
                             -f.z, 0.0, 0.0, 0.0, 0.0, 1.0);

  center_ = Vector3d(center.x, center.y, center.z);
  distance_ = lv.Length();
  rotation_ = rotation_matrix.GetRotate().Inverse();
}

// Get the position of the manipulator as 4x4 matrix
Matrix4x4f TrackBall::GetMatrix() const {
  Matrix4x4d mat = Matrix4x4d::MakeTrans(0.0, 0.0, distance_) *
                   Matrix4x4d::MakeRotation(rotation_) *
                   Matrix4x4d::MakeTrans(center_);

  return Matrix4x4f(mat.m11, mat.m12, mat.m13, mat.m14, mat.m21, mat.m22,
                    mat.m23, mat.m24, mat.m31, mat.m32, mat.m33, mat.m34,
                    mat.m41, mat.m42, mat.m43, mat.m44);
}

// Get the position of the manipulator as a inverse matrix of the manipulator,
// typically used as a model view matrix.
Matrix4x4f TrackBall::GetInverseMatrix() const {
  Matrix4x4d mat = Matrix4x4d::MakeTrans(-center_) *
                   Matrix4x4d::MakeRotation(rotation_.inverse()) *
                   Matrix4x4d::MakeTrans(0.0, 0.0, -distance_);

  return Matrix4x4f(mat.m11, mat.m12, mat.m13, mat.m14, mat.m21, mat.m22,
                    mat.m23, mat.m24, mat.m31, mat.m32, mat.m33, mat.m34,
                    mat.m41, mat.m42, mat.m43, mat.m44);
}

void TrackBall::RotateTrackball(const float px0, const float py0,
                                const float px1, const float py1) {
  //计算旋转轴角
  Vector3d axis;
  double angle;
  Trackball(axis, angle, px1, py1, px0, py0);

  //轴角转换成四元数
  Quaterniond new_rotate;
  new_rotate.FromAngleAxis(axis, angle);

  //四元数累积
  rotation_ = rotation_ * new_rotate;
}

void TrackBall::Trackball(V3d &axis, double &angle, float p1x, float p1y,
                          float p2x, float p2y) {
  Matrix4x4d rotation_matrix = Matrix4x4d::MakeRotation(rotation_);

  V3d sv = rotation_matrix.TransformCoord(V3d(1.0f, 0.0f, 0.0f));
  V3d uv = rotation_matrix.TransformCoord(V3d(0.0f, 1.0f, 0.0f));
  V3d lv = rotation_matrix.TransformCoord(V3d(0.0f, 0.0f, 1.0f));

  V3d p1 = sv * (double)p1x + uv * (double)p1y +
           lv * (double)TbProjectToSphere(trackball_size_, p1x, p1y);
  V3d p2 = sv * (double)p2x + uv * (double)p2y +
           lv * (double)TbProjectToSphere(trackball_size_, p2x, p2y);

  axis = p2.cross(p1);
  axis.normalize();

  //���Ҷ�������н�
  double t = (p2 - p1).norm() / (2.0 * trackball_size_);

  if (t > 1.0) t = 1.0;
  if (t < -1.0) t = -1.0;
  angle = asin(t);
}

float TrackBall::TbProjectToSphere(float r, float x, float y) {
  float d, t, z;
  d = sqrt(x * x + y * y);
  /* Inside sphere */
  if (d < r * 0.70710678118654752440)  // x^2 + y^2 < (r^2 )/ 2
  {
    // sqrt[r^2 - (x^2 + y^2)]
    z = sqrt(r * r - d * d);
  }  // On hyperbola
  else {
    // (r^2 / 2) / sqrt(x^2 + y^2)
    t = r / 1.41421356237309504880;
    z = t * t / d;
  }
  return z;
}

float TrackBall::GetXnormalized(float x) {
  return (x - viewport_.X()) / viewport_.Width() * 2.0f - 1.0f;
}

float TrackBall::GetYnormalized(float y) {
  return (y - viewport_.Y()) / viewport_.Height() * 2.0f - 1.0f;
}

}  // namespace geditor
