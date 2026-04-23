
#include "pcd/memory_file.h"
#include <memory.h>
#include <cstdio>

namespace geditor {

MemroyFile::MemroyFile() : memory_(NULL), length_(0), position_(0) {}

MemroyFile::~MemroyFile() {
  if (memory_) {
    delete[] memory_;
    memory_ = NULL;
  }

  length_ = 0;
}

bool MemroyFile::Allocate(int length) {
  char *pMemroy = new char[length];
  if (!pMemroy) {
    return NULL;
  }

  if (memory_) {
    delete[] memory_;
  }

  length_ = length;
  memory_ = pMemroy;

  return true;
}

char *MemroyFile::GetBuffer() const { return memory_; }

int MemroyFile::GetLength() const { return length_; }

bool MemroyFile::eof() { return (position_ >= length_); }

bool MemroyFile::seekg(int offset, Mode origin) {
  int lNewPos = position_;

  if (origin == beg) {
    lNewPos = offset;
  } else if (origin == cur) {
    lNewPos += offset;
  } else if (origin == end) {
    lNewPos = length_ + offset;
  } else {
    return false;
  }

  if (lNewPos < 0) {
    lNewPos = 0;
  }

  position_ = lNewPos;
  return true;
}

bool MemroyFile::Read(char *buffer, int nSize) {
  int leftSize = length_ - position_;
  if (leftSize <= nSize) {
    memcpy(buffer, memory_ + position_, nSize);
    return true;
  }
  return false;
}

void MemroyFile::ReadLine(char *buffer, int nSize) {
  int pos = 0;
  while (memory_[position_]) {
    char ch = memory_[position_++];
    if (ch != '\n' && pos < nSize - 1) {
      buffer[pos++] = ch;
    } else {
      buffer[pos++] = 0;
      break;
    }
  }
}

int MemroyFile::tellg() { return position_; }

void MemroyFile::close() {}

}  // namespace geditor
