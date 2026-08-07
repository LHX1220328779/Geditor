#include "map/mine_origin_config.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include "map/projection_utm.h"

namespace geditor {
namespace {

std::string Trim(const std::string &value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "";
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

}  // namespace

bool MineOriginConfig::Load(const std::string &path,
                            std::vector<MineOrigin> &origins,
                            std::string *error,
                            std::string *current_origin_name) {
  origins.clear();
  if (current_origin_name) current_origin_name->clear();
  std::ifstream input(path);
  if (!input.is_open()) {
    if (error) *error = "cannot open " + path;
    return false;
  }

  std::string line;
  int line_number = 0;
  MineOrigin current;
  bool has_current = false;
  bool has_lat = false;
  bool has_lon = false;
  bool has_alt = false;
  std::string configured_current_origin;

  auto finish_current = [&]() -> bool {
    if (!has_current) return true;
    if (!has_lat || !has_lon || !has_alt) {
      if (error) {
        *error = "mine '" + current.name +
                 "' must define GLOBAL_ORIGIN_LAT, GLOBAL_ORIGIN_LON and "
                 "GLOBAL_ORIGIN_ALT";
      }
      return false;
    }
    ProjectionUTM projection;
    UTMPoint utm;
    projection.LatLonToCartesian(current.latitude, current.longitude, utm);
    current.x = utm.x;
    current.y = utm.y;
    current.zone = utm.zone;
    origins.push_back(current);
    return true;
  };

  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    if (Trim(line).empty()) continue;
    const auto indent = line.find_first_not_of(' ');
    const std::string value = Trim(line);
    if (indent == 0 && value == "mine_origins:") continue;

    const auto separator = value.find(':');
    if (separator == std::string::npos) {
      if (error) *error = "invalid YAML at line " + std::to_string(line_number);
      return false;
    }
    const std::string key = Trim(value.substr(0, separator));
    const std::string scalar = Trim(value.substr(separator + 1));
    try {
      if (indent == 0 && key == "current_origin" && !scalar.empty()) {
        configured_current_origin = scalar;
      } else if (indent == 2 && scalar.empty()) {
        if (!finish_current()) return false;
        current = MineOrigin();
        current.name = key;
        has_current = true;
        has_lat = has_lon = has_alt = false;
      } else if (indent == 4 && has_current) {
        if (key == "GLOBAL_ORIGIN_LAT") {
          current.latitude = std::stod(scalar);
          has_lat = true;
        } else if (key == "GLOBAL_ORIGIN_LON") {
          current.longitude = std::stod(scalar);
          has_lon = true;
        } else if (key == "GLOBAL_ORIGIN_ALT") {
          current.z = std::stod(scalar);
          has_alt = true;
        } else {
          throw std::invalid_argument("unsupported field " + key);
        }
      } else {
        throw std::invalid_argument("unexpected YAML indentation");
      }
    } catch (const std::exception &e) {
      if (error) {
        *error = "invalid value at line " + std::to_string(line_number) +
                 ": " + e.what();
      }
      return false;
    }
  }

  if (!finish_current()) return false;

  if (origins.empty()) {
    if (error) *error = "no mine origins configured in " + path;
    return false;
  }

  if (!configured_current_origin.empty()) {
    const auto current =
        std::find_if(origins.begin(), origins.end(), [&](const MineOrigin &o) {
          return o.name == configured_current_origin;
        });
    if (current == origins.end()) {
      if (error) {
        *error = "current_origin '" + configured_current_origin +
                 "' is not defined in " + path;
      }
      origins.clear();
      return false;
    }
    std::rotate(origins.begin(), current, current + 1);
  }
  if (current_origin_name) *current_origin_name = origins.front().name;
  return true;
}

bool MineOriginConfig::SaveCurrentOrigin(const std::string &path,
                                         const std::string &origin_name,
                                         std::string *error) {
  if (origin_name.empty() || origin_name.find_first_of(":\r\n#") !=
                                 std::string::npos) {
    if (error) *error = "invalid current origin name";
    return false;
  }

  std::ifstream input(path);
  if (!input.is_open()) {
    if (error) *error = "cannot open " + path;
    return false;
  }

  std::vector<std::string> lines;
  std::string line;
  bool replaced = false;
  while (std::getline(input, line)) {
    const std::string value = Trim(line);
    const auto indent = line.find_first_not_of(' ');
    if (indent == 0 && value.rfind("current_origin:", 0) == 0) {
      lines.push_back("current_origin: " + origin_name);
      replaced = true;
    } else if (!replaced && indent == 0 && value == "mine_origins:") {
      lines.push_back("current_origin: " + origin_name);
      lines.push_back(line);
      replaced = true;
    } else {
      lines.push_back(line);
    }
  }
  input.close();

  if (!replaced) {
    if (error) *error = "mine_origins section not found in " + path;
    return false;
  }

  const std::filesystem::path target(path);
  const std::filesystem::path temporary = target.string() + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output.is_open()) {
    if (error) *error = "cannot write " + temporary.string();
    return false;
  }
  for (const std::string &saved_line : lines) output << saved_line << '\n';
  output.flush();
  if (!output.good()) {
    output.close();
    std::error_code remove_error;
    std::filesystem::remove(temporary, remove_error);
    if (error) *error = "failed to write " + temporary.string();
    return false;
  }
  output.close();

  std::error_code rename_error;
  std::filesystem::rename(temporary, target, rename_error);
  if (rename_error) {
    std::error_code remove_error;
    std::filesystem::remove(temporary, remove_error);
    if (error) {
      *error = "cannot replace " + path + ": " + rename_error.message();
    }
    return false;
  }
  return true;
}

}  // namespace geditor
