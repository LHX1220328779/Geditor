#pragma once

#include <map>
#include <string>
#include <vector>

struct DrawType {
  int type = 0;
  int subtype = 0;
  DrawType() {}
  DrawType(int t, int s) : type(t), subtype(s) {}
  static DrawType Boundary(int s = 1) { return {0, s}; }
  static DrawType Lane(int s = 1) { return {1, s}; }
  static DrawType RoadArea(int s = 0) { return {2, s}; }
  static DrawType Trafficsign(int s = 14) { return {3, s}; }
  static DrawType FunArea(int s = 0) { return {4, s}; }
};

struct FeatType {
  std::string name;
  int type;

  static int Find(const std::vector<FeatType>& types, int type) {
    for (int i = 0; i < types.size(); ++i) {
      if (types[i].type == type) return i;
    }
    return types.size() - 1;
  }
};

struct BT {
  static int L2(int l, int r) { return (l & 0xff) << 8 | r; }
};
struct TV {
  static const int none = 0xffffffff;
  static std::vector<FeatType> sign;
  static std::vector<FeatType> boundary;
  static std::vector<FeatType> roadarea;
  static std::vector<FeatType> laneturn;
  static std::vector<FeatType> lanetype;
  static const int jobpoint = 100;
  static std::vector<FeatType> jobarea;
};
#define kNone TV::none
#define kSign0 TV::sign0
#define kSignType TV::sign
#define kBoundaryType TV::boundary
#define kRoadAreaType TV::roadarea
#define kLaneTurnType TV::laneturn
#define kLaneType TV::lanetype
#define kJobAreaType TV::jobarea