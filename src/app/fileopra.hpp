#pragma once
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;
namespace geditor {
namespace file_opra {
void ReCreateDirectory(const std::string& path) {
  try {
    if (fs::exists(path)) {
      fs::remove_all(path);
    }
    if (fs::create_directories(path)) {
      std::cout << "Directory created successfully: " << path << std::endl;
    } else {
      std::cerr << "Failed to create directory: " << path << std::endl;
    }
  } catch (const fs::filesystem_error& e) {
    std::cerr << "Filesystem error: " << e.what() << std::endl;
  }
}
void CreateDirectory(const std::string& path) {
  try {
    if (fs::exists(path)) {
      return;
    }
    if (fs::create_directories(path)) {
      std::cout << "Directory created successfully: " << path << std::endl;
    } else {
      std::cerr << "Failed to create directory: " << path << std::endl;
    }
  } catch (const fs::filesystem_error& e) {
    std::cerr << "Filesystem error: " << e.what() << std::endl;
  }
}
}  // namespace file_opra
}  // namespace geditor