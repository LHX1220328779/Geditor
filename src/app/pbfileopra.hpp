#pragma once
#include <fcntl.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "fileopra.hpp"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"

namespace fs = std::filesystem;
namespace geditor {
namespace file_opra {
bool SetProtoToASCIIFile(const google::protobuf::Message& message,
                         int file_descriptor) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  if (file_descriptor < 0) {
    LOG(ERROR) << "Invalid file descriptor.";
    return false;
  }
  ZeroCopyOutputStream* output = new FileOutputStream(file_descriptor);
  bool success = TextFormat::Print(message, output);
  delete output;
  close(file_descriptor);
  return success;
}

bool SetProtoToASCIIFile(const google::protobuf::Message& message,
                         const std::string& file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  if (fd < 0) {
    LOG(ERROR) << "Unable to open file " << file_name << " to write.";
    return false;
  }
  return SetProtoToASCIIFile(message, fd);
}
}  // namespace file_opra
}  // namespace geditor