
#pragma once

#include "algorithm/common.h"
#include "algorithm/matrix44.h"
#include "algorithm/viewport.h"

namespace geditor {

class TrackBall {
 public:
  TrackBall();

  ~TrackBall();

 public:
  void LButtonDown(float x, float y);

  void MouseRotate(float x, float y);

  void MouseMove(float x, float y, float &dirX, float &dirY);

  void SetViewport(const Viewport &viewport);

  void SetTransformation(const V3f &eye, const V3f &center, const V3f &up);

  Matrix4x4f GetInverseMatrix() const;

  Matrix4x4f GetMatrix() const;

 private:
  void RotateTrackball(const float px0, const float py0, const float px1,
                       const float py1);

  void Trackball(V3d &axis, double &angle, float p1x, float p1y, float p2x,
                 float p2y);

  float TbProjectToSphere(float r, float x, float y);

  float GetXnormalized(float x);

  float GetYnormalized(float y);

 private:
  float start_x_;
  float start_y_;
  float end_x_;
  float end_y_;

  float trackball_size_;
  Viewport viewport_;

  double distance_;
  V3d center_;
  Quatd rotation_;
};

}  // namespace geditor
