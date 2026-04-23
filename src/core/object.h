
#pragma once

namespace geditor {
class Object {
 public:
  enum class DataVariance { DYNAMIC, STATIC, UNSPECIFIED };

 public:
  Object();

  virtual ~Object();

  DataVariance GetDataVariance() const { return data_variance_; }

 private:
  DataVariance data_variance_;
};

}  // namespace geditor
