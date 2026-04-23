
#pragma once

namespace geditor {

template <typename Ty>
class Constant {
 public:
  static const Ty MAX_REAL;
  static const Ty ZERO_TOLERANCE;
};

//------------------------------------------------------------------------------
typedef Constant<float> Constantf;
typedef Constant<double> Constantd;

//------------------------------------------------------------------------------

}  // namespace geditor
