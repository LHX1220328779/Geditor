
#pragma once

#include "core/drawable.h"
#include "core/geometry.h"
#include "core/job_area.h"

namespace geditor {

class FactoryDrawable {
 public:
  static Drawable *CreateLineDrawable(Geometry *pPolyline, const V3d &vCnt,
                                      const V4f &clr);

  static Drawable *CreateArrowDrawable(Geometry *pPolyline, const V3d &vCnt,
                                       const V4f &clr, float size,
                                       float r = 0.5);

  static Drawable *CreateNodeDrawable(Geometry *pPolyline, const V3d &vCnt,
                                      float size, const V4f &vclr,
                                      int hlid = -1);

  static Drawable *CreateAreaDrawable(Geometry *pPolyline, const V3d &vCnt,
                                      const V4f &clr);

  static Drawable *CreateWidthLineDrawable(Geometry *pPolyline, const V3d &vCnt,
                                           double dWidth, const V4f &clr);

  static Drawable *CreateCarBodyLineDrawable(Geometry *pPolyline,
                                             const V3d &vCnt, double dWidth,
                                             const V4f &clr,
                                             std::vector<JobArea *> &jobArea);
};

}  // namespace geditor
