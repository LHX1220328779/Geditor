
#pragma once

#include <cstdio>
#include "pcd/file.h"

namespace geditor {

class StdioFile : public File {
 public:
  StdioFile();

  virtual ~StdioFile();

 public:
  bool open(const char *filename);

  int GetLength() const;

  virtual bool eof();

  virtual bool seekg(int offset, Mode way);

  virtual int tellg();

  virtual void close();

  virtual bool Read(char *buffer, int nSize);

  virtual void ReadLine(char *buffer, int nSize);

 private:
  FILE *file_;
};

}  // namespace geditor
