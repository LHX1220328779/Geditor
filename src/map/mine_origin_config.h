#pragma once

#include <string>
#include <vector>

namespace geditor {

struct MineOrigin {
  std::string name;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  int zone = 0;
  double longitude = 0.0;
  double latitude = 0.0;
};

class MineOriginConfig {
 public:
  static bool Load(const std::string &path, std::vector<MineOrigin> &origins,
                   std::string *error = nullptr,
                   std::string *current_origin_name = nullptr);

  // Persists the selected origin by name without copying or changing any
  // coordinate values. A following Load() returns that origin first.
  static bool SaveCurrentOrigin(const std::string &path,
                                const std::string &origin_name,
                                std::string *error = nullptr);
};

}  // namespace geditor
