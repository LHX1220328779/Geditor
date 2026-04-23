
#pragma once

#include <vector>
#include "algorithm/common.h"

namespace geditor {

struct Color {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;

  Color() : r(0), g(0), b(0), a(255) {}

  Color(unsigned char rr, unsigned char gg, unsigned char bb)
      : r(rr), g(gg), b(bb), a(255) {}

  Color(unsigned char rr, unsigned char gg, unsigned char bb, unsigned char aa)
      : r(rr), g(gg), b(bb), a(aa) {}

  float redF() const { return ((float)r / 255.0f); }

  float greenF() const { return ((float)g / 255.0f); }

  float blueF() const { return ((float)b / 255.0f); }
};

//! Color scale element: one value + one color
class ccColorScaleElement {
 public:
  ccColorScaleElement(double relativePos, Color color)
      : m_relativePos(relativePos), m_color(color) {}

  inline void setRelativePos(double pos) { m_relativePos = pos; }

  inline double getRelativePos() const { return m_relativePos; }

  inline void setColor(Color color) { m_color = color; }

  inline const Color &getColor() const { return m_color; }

  inline static bool IsSmaller(const ccColorScaleElement &e1,
                               const ccColorScaleElement &e2) {
    return e1.getRelativePos() < e2.getRelativePos();
  }

 protected:
  double m_relativePos;
  Color m_color;
};

class ColorInterpolation {
 public:
  static const unsigned MIN_STEPS = 2;
  static const unsigned DEFAULT_STEPS = 256;
  static const unsigned MAX_STEPS = 1024;

 public:
  ColorInterpolation();

  ~ColorInterpolation();

  void Insert(const ccColorScaleElement &step);

  void SetValue(float minValue, float maxValue);

  Color GetColor(float value, int steps);

  void Sort();

  void Update();

 private:
  float min_value_;
  float max_value_;
  Color clr_begin_;
  Color clr_end_;
  Color rgba_scale_[MAX_STEPS];
  std::vector<ccColorScaleElement> steps_;
};

inline V4f Hsv2Rgb(float H, float S, float V) {
  float R, G, B;

  int i;
  float f, p, q, t;
  if (S == 0) {
    // achromatic (grey)
    R = G = B = V;
    return V4f(R / 255.0, G / 255.0, B / 255.0, 1.0);
  }

  H /= 60.0;  // sector 0 to 5
  i = floor(H);
  f = H - i;  // factorial part of h
  p = V * (1 - S);
  q = V * (1 - S * f);
  t = V * (1 - S * (1 - f));

  switch (i) {
    case 0:
      R = V;
      G = t;
      B = p;
      break;
    case 1:
      R = q;
      G = V;
      B = p;
      break;
    case 2:
      R = p;
      G = V;
      B = t;
      break;
    case 3:
      R = p;
      G = q;
      B = V;
      break;
    case 4:
      R = t;
      G = p;
      B = V;
      break;
    default:  // case 5:
      R = V;
      G = p;
      B = q;
      break;
  }

  return V4f(R / 255.0, G / 255.0, B / 255.0, 1.0);
}

}  // namespace geditor
