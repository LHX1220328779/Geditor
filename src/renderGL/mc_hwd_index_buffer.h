
#pragma once

#include "renderGL/mc_buffer.h"

namespace geditor {

class HwdIndexBuffer {
 public:
  HwdIndexBuffer();

  ~HwdIndexBuffer();

  void Create(const IndexBuffer *indexbuf);

  void Destroy();

  void Enable();

  void Disable();

  void *Lock(Buffer::LockMode mode);

  void Unlock();

  bool IsValidate() const;

  unsigned int GetHandle() const;

 private:
  unsigned int handle_;
};

}  // namespace geditor
