
#pragma once

#include <string>
#include <vector>

namespace geditor {

class String {
 public:
  static void Trim(char *szTmp);

  static void TrimLeft(std::string &str);

  static void TrimRight(std::string &str);

  static void Split(std::vector<std::string> &st, std::string line,
                    std::string de);

  static bool IEquals(const std::string &str1, const std::string &str2);
};

}  // namespace geditor
