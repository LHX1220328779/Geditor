
#include "utils/string_ext.h"

namespace geditor {

void String::Trim(char *szTmp) {}

void String::TrimLeft(std::string &str) {}

void String::TrimRight(std::string &str) {
  int pos = 0;

  while (str[pos]) {
    if (str[pos] != '\n' && str[pos] != '\r') {
      pos++;
    } else {
      break;
    }
  }
  str = str.substr(0, pos);
}

void String::Split(std::vector<std::string> &result, std::string line,
                   std::string delimiters) {
  result.clear();
  size_t start = 0;
  while (start < line.size()) {
    //���ݶ���ָ���ָ�
    size_t itRes = line.find(delimiters[0], start);
    for (size_t i = 1; i < delimiters.size(); ++i) {
      size_t it = line.find(delimiters[i], start);
      if (it < itRes) itRes = it;
    }
    if (itRes == std::string::npos) {
      result.push_back(line.substr(start, line.size() - start));
      break;
    }
    result.push_back(line.substr(start, itRes - start));
    start = itRes;
    ++start;
  }
}

bool String::IEquals(const std::string &str1, const std::string &str2) {
  if (str1 == str2) {
    return true;
  }
  return false;
}

}  // namespace geditor
