
#include "pcd/stdio_file.h"
#include "pcd/file.h"

namespace geditor {

StdioFile::StdioFile() {}

StdioFile::~StdioFile() {}

bool StdioFile::open(const char *filename) {
  file_ = fopen(filename, "rb");
  if (file_ != NULL) {
    return true;
  }
  return false;
}

int StdioFile::GetLength() const {
  fseek(file_, 0L, SEEK_END);
  long ret = ftell(file_);
  fseek(file_, 0L, SEEK_SET);
  return ret;
}

bool StdioFile::eof() {
  int ret = feof(file_);
  return (ret != 0);
}

bool StdioFile::seekg(int offset, Mode way) {
  int orgin = SEEK_SET;

  if (way == beg) {
    orgin = SEEK_SET;
  } else if (way == cur) {
    orgin = SEEK_CUR;
  } else if (way == end) {
    orgin = SEEK_END;
  }
  fseek(file_, offset, orgin);
  return true;
}

int StdioFile::tellg() {
  long ret = ftell(file_);
  return (int)ret;
}

bool StdioFile::Read(char *buffer, int nSize) {
  size_t nReadBytes = fread(buffer, 1, nSize, file_);
  return (nReadBytes == nSize);
}

void StdioFile::ReadLine(char *buffer, int nSize) {
  fgets(buffer, nSize, file_);
}

void StdioFile::close() {
  fclose(file_);
  file_ = NULL;
}

}  // namespace geditor
