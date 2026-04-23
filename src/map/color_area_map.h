
#pragma once

#include <map>
#include "utils/color_interpolation.h"

namespace geditor {

class ColorAreaMap {
 public:
  enum {
    CAM_UNKOWN = 0,
    CAM_SIDEWALK = 1,
    CAM_MOTORWAY = 2,
    CAM_GREENSPACE = 3,
    CAM_CROSSWALK = 4,
    CAM_RIDEWAY = 5,
    CAM_SPEEDBUMP = 6,
    CAM_STOPLINE = 7,
    CAM_YIELDLINE = 8,
    CAM_JUNCTION = 9,
    CAM_SCHOOL = 10,
    CAM_WARIN = 11,
    CAM_NOSTOP = 12,

    //===================================

    WOM_ROAD = 101,
    WOM_CONTROL = 102,
    WOM_GOODS = 103,
    WOM_SWEEP = 104,
    WOM_TRASH = 105,
    WOM_CONNTECT = 106,
    WOM_MAINTAIN = 107,
    WOM_PILE = 108,
    WOM_SPEEDBUMP = 109,
    WOM_SPEEDWON = 113,
    WOM_WATER = 114,

    WOM_CURBA = 115,
    WOM_CURBB = 116,
    WOM_CURBC = 117,
    WOM_CURBD = 118,

    //=================================

  };

 public:
  ColorAreaMap();

  ~ColorAreaMap();

  int GetAreaOrder(int type);

  Color GetAreaColor(int type);

  Color GetWoColor(int type, float fpro, bool bGps);

  Color GetDisplayColor(int type);

 private:
  std::map<int, Color> map_color_;
  std::map<int, Color> map_display_;
  std::map<int, int> map_order_;
};

}  // namespace geditor
