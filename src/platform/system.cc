
#include "platform/system.h"

#include <unistd.h>

#include "core/log.h"

namespace geditor {

int System::GetAppPath(std::string &path) {
  char buff[1024];
  readlink("/proc/self/exe", buff, 1024);
  std::string s(buff);
  path = s.substr(0, s.find_last_of('/'));
  LOG(INFO) << "app path: " << path;
  return 1;
}

}  // namespace geditor
