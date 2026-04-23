
#include "utils/color_interpolation.h"
#include <algorithm>
#include "algorithm/common.h"

namespace geditor {

ColorInterpolation::ColorInterpolation() {}

ColorInterpolation::~ColorInterpolation() {}

void ColorInterpolation::Update() {
  if (steps_.size() >= static_cast<int>(MIN_STEPS)) {
    Sort();

    const unsigned stepCount = static_cast<unsigned>(steps_.size());

    unsigned j = 0;
    for (unsigned i = 0; i < MAX_STEPS; ++i) {
      const double relativePos = static_cast<double>(i) / (MAX_STEPS - 1);

      while (j + 2 < stepCount &&
             steps_[j + 1].getRelativePos() < relativePos) {
        ++j;
      }

      const V3d colBefore(steps_[j].getColor().redF(),
                          steps_[j].getColor().greenF(),
                          steps_[j].getColor().blueF());

      const V3d colNext(steps_[j + 1].getColor().redF(),
                        steps_[j + 1].getColor().greenF(),
                        steps_[j + 1].getColor().blueF());

      const double alpha =
          (relativePos - steps_[j].getRelativePos()) /
          (steps_[j + 1].getRelativePos() - steps_[j].getRelativePos());

      const V3d interpCol = colBefore + (colNext - colBefore) * alpha;

      rgba_scale_[i] =
          Color(static_cast<unsigned char>(interpCol[0] * 255),
                static_cast<unsigned char>(interpCol[1] * 255),
                static_cast<unsigned char>(interpCol[2] * 255), 255);
    }
  }
}

void ColorInterpolation::Insert(const ccColorScaleElement &step) {
  steps_.push_back(step);
  Update();
}

void ColorInterpolation::SetValue(float minValue, float maxValue) {
  min_value_ = minValue;
  max_value_ = maxValue;
}

Color ColorInterpolation::GetColor(float value, int steps) {
  double relativePos = (value - min_value_) / (max_value_ - min_value_);
  unsigned index =
      (static_cast<unsigned>((relativePos * steps) * 65535.0)) >> 16;
  index = (index * (MAX_STEPS - 1)) / steps;
  return rgba_scale_[index];
}

void ColorInterpolation::Sort() {
  std::sort(steps_.begin(), steps_.end(), ccColorScaleElement::IsSmaller);
}

}  // namespace geditor