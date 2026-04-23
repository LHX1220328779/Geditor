
#pragma once

#include "pcd/file.h"

namespace geditor {

class MemroyFile : public File {
 public:
  enum Mode {
    beg = 0,
    cur = 1,
    end = 2,
  };

 public:
  MemroyFile();

  virtual ~MemroyFile();

 public:
  bool Allocate(int length);

  char *GetBuffer() const;

  int GetLength() const;

  virtual bool eof();

  virtual bool seekg(int offset, Mode way);

  virtual int tellg();

  virtual void close();

  virtual bool Read(char *buffer, int nSize);

  virtual void ReadLine(char *buffer, int nSize);

 private:
  char *memory_;
  int length_;
  int position_;
};

}  // namespace geditor
