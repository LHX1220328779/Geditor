
#pragma once

#include <cstdint>
#include "algorithm/common.h"

namespace geditor {

/// NOTE
/// 由于Eigen会自动将内部数据对齐到16bit，所以下面这个PointColor数据应该占32字节（8个float而非7个）
/// 因此，在访问color数据时，也不能直接在point数据偏移一个sizeof(V3f)，而得偏移sizeof(V4f)
/// 在windows下没有这个问题
struct PointColor {
 public:
  V3f point;
  V4f color;

 public:
  PointColor() {}

  PointColor(float x, float y, float z, const V4f &clr)
      : point(x, y, z), color(clr) {}

  PointColor(const V3f &vec, const V4f &clr) : point(vec), color(clr) {}

  static V4f FromRGB(std::uint32_t rgb, float intensity) {
    V4f color;
    color.x = static_cast<float>((rgb >> 16) & 0xff) / 255.0f;
    color.y = static_cast<float>((rgb >> 8) & 0xff) / 255.0f;
    color.z = static_cast<float>(rgb & 0xff) / 255.0f;
    // Alpha is reused by the existing shader as the optional intensity filter.
    color.w = intensity;
    return color;
  }

  static V4f GetColor(int intensity, float z, int type, int c) {
    static std::vector<V4f> iColor;

    if (iColor.size() <= 0) {
      for (int i = 0; i < 256; i++) {
        V4f p;
        p.z = 0;
        p.y = i / 255.;
        p.x = 1;
        iColor.push_back(p);
      }

      for (int i = 0; i < 256; i++) {
        V4f p;
        p.z = 0;
        p.y = 1;
        p.x = (255 - i) / 255.;
        iColor.push_back(p);
      }
      for (int i = 0; i < 256; i++) {
        V4f p;
        p.z = i / 255.;
        p.y = 1;
        p.x = 0;
        iColor.push_back(p);
      }
      for (int i = 0; i < 256; i++) {
        V4f p;
        p.z = 1;
        p.y = (255 - i) / 255.;
        p.x = 0;
        iColor.push_back(p);
      }
      for (int i = 0; i < 256; i++) {
        V4f p;
        p.z = 1;
        p.y = 0;
        p.x = i / 255.;
        iColor.push_back(p);
      }
      for (int i = 0; i < 256; i++) {
        V4f p;
        p.z = (255 - i) / 255.;
        p.y = 0;
        p.x = 1;
        iColor.push_back(p);
      }
    }
    int index = intensity;
    if (type == 1) {
      index = z / 20.0 * 255;
    } else if (type == 2) {
      index += z / 20.0 * 255;
      index *= 0.5;
    }
    // filed = Math<int>::Clamp(filed, 0, 255);
    index *= c;
    index = Math<int>::Clamp(index, 0, 1530);
    // index = index % 1530;
    V4f p = iColor[index];
    p.w = intensity;
    return p;
  }
};

}  // namespace geditor
