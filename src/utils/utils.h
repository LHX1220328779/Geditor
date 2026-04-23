
#pragma once

namespace geditor {

class Utils {
 public:
  static unsigned int CalcCrc32(const void *buf, int size);

 private:
  static const unsigned int crc32tab_[];
};
}  // namespace geditor
