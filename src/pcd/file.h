
#pragma once

namespace geditor {

class File {
 public:
  enum Mode {
    beg = 0,
    cur = 1,
    end = 2,
  };

  File() {}

  virtual ~File() {}

 public:
  virtual bool eof() = 0;

  virtual bool seekg(int offset, Mode way) = 0;

  virtual int tellg() = 0;

  virtual void close() = 0;

  virtual bool Read(char *buffer, int nSize) = 0;

  virtual void ReadLine(char *buffer, int nSize) = 0;
};

}  // namespace geditor
