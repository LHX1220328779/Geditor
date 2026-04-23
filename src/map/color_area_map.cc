
#include "map/color_area_map.h"

namespace geditor {

ColorAreaMap::ColorAreaMap() {
  map_color_.insert(std::make_pair<int, Color>(CAM_UNKOWN, Color(0, 0, 0, 0)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_SIDEWALK, Color(100, 200, 255, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_MOTORWAY, Color(100, 100, 255, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_GREENSPACE, Color(0, 255, 0, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_CROSSWALK, Color(100, 50, 255, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_RIDEWAY, Color(100, 150, 255, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_SPEEDBUMP, Color(150, 150, 245, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_STOPLINE, Color(100, 150, 255, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_YIELDLINE, Color(150, 150, 245, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_JUNCTION, Color(100, 250, 255, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_SCHOOL, Color(245, 200, 0, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_WARIN, Color(150, 250, 50, 127)));
  map_color_.insert(
      std::make_pair<int, Color>(CAM_NOSTOP, Color(150, 250, 250, 127)));

  map_color_.insert(
      std::make_pair<int, Color>(WOM_ROAD, Color(0, 255, 0, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_CONTROL, Color(0, 100, 150, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_GOODS, Color(0, 100, 255, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_SWEEP, Color(110, 100, 0, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_TRASH, Color(110, 100, 100, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_CONNTECT, Color(30, 100, 100, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_MAINTAIN, Color(255, 100, 255, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_PILE, Color(255, 100, 0, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_SPEEDBUMP, Color(180, 100, 100, 255)));

  map_color_.insert(
      std::make_pair<int, Color>(WOM_SPEEDWON, Color(0, 40, 120, 255)));
  map_color_.insert(
      std::make_pair<int, Color>(WOM_WATER, Color(154, 205, 50, 255)));
  //===============================================================

  map_display_.insert(
      std::make_pair<int, Color>(CAM_UNKOWN, Color(0, 0, 0, 0)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_SIDEWALK, Color(47, 0, 7, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_MOTORWAY, Color(0, 100, 0, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_GREENSPACE, Color(0, 255, 0, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_CROSSWALK, Color(204, 204, 204, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_RIDEWAY, Color(100, 150, 255, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_SPEEDBUMP, Color(64, 128, 128, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_STOPLINE, Color(200, 10, 255, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_YIELDLINE, Color(150, 150, 10, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_JUNCTION, Color(255, 255, 0, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_SCHOOL, Color(245, 200, 0, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_WARIN, Color(150, 250, 50, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(CAM_NOSTOP, Color(50, 0, 250, 127)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_ROAD, Color(0, 255, 0, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_CONTROL, Color(0, 100, 150, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_GOODS, Color(0, 100, 255, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_SWEEP, Color(110, 100, 0, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_TRASH, Color(110, 100, 100, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_CONNTECT, Color(30, 100, 100, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_MAINTAIN, Color(255, 100, 255, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_PILE, Color(255, 100, 0, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_SPEEDBUMP, Color(180, 100, 100, 255)));

  map_display_.insert(
      std::make_pair<int, Color>(WOM_SPEEDWON, Color(0, 40, 120, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_WATER, Color(154, 205, 50, 255)));

  map_display_.insert(
      std::make_pair<int, Color>(WOM_CURBA, Color(255, 0, 0, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_CURBB, Color(0, 255, 0, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_CURBC, Color(0, 0, 255, 255)));
  map_display_.insert(
      std::make_pair<int, Color>(WOM_CURBD, Color(255, 255, 0, 255)));

  //=========================================================================================
  map_order_.insert(std::make_pair<int, int>(CAM_UNKOWN, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_SIDEWALK, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_MOTORWAY, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_GREENSPACE, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_CROSSWALK, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_RIDEWAY, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_SPEEDBUMP, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_STOPLINE, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_YIELDLINE, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_JUNCTION, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_SCHOOL, 0));
  map_order_.insert(std::make_pair<int, int>(CAM_WARIN, 0));
  map_order_.insert(std::make_pair<int, int>(WOM_ROAD, 1));
  map_order_.insert(std::make_pair<int, int>(WOM_CONTROL, 2));
  map_order_.insert(std::make_pair<int, int>(WOM_GOODS, 3));
  map_order_.insert(std::make_pair<int, int>(WOM_SWEEP, 3));
  map_order_.insert(std::make_pair<int, int>(WOM_TRASH, 3));
  map_order_.insert(std::make_pair<int, int>(WOM_CONNTECT, 3));
  map_order_.insert(std::make_pair<int, int>(WOM_MAINTAIN, 3));
  map_order_.insert(std::make_pair<int, int>(WOM_PILE, 4));
  map_order_.insert(std::make_pair<int, int>(WOM_SPEEDBUMP, 4));
  // m_mapOrder.insert(std::make_pair<int, int>(99,          5));
  // m_mapOrder.insert(std::make_pair<int, int>(99,          5));
  // m_mapOrder.insert(std::make_pair<int, int>(99,          5));
  map_order_.insert(std::make_pair<int, int>(WOM_SPEEDWON, 5));
  map_order_.insert(std::make_pair<int, int>(WOM_WATER, 5));
}

ColorAreaMap::~ColorAreaMap() {}

Color ColorAreaMap::GetDisplayColor(int type) {
  std::map<int, Color>::iterator it_find = map_display_.find(type);

  if (map_display_.end() != it_find) {
    return it_find->second;
  } else {
    return Color();
  }
}

Color ColorAreaMap::GetAreaColor(int type) {
  std::map<int, Color>::iterator it_find = map_color_.find(type);

  if (map_color_.end() != it_find) {
    return it_find->second;
  } else {
    return Color();
  }
}

Color ColorAreaMap::GetWoColor(int type, float paramPro, bool bGps) {
  // 100����������Э��
  if (type < 100) {
    //����Э�飬תΪ��ϵС��Э��
    if (type == 2) {
      type = WOM_ROAD;
    }
  }

  int nX = 0;
  if (type == WOM_ROAD)  //��·��
  {
    nX = 4;
  } else if (type == WOM_SWEEP)  //��ɨ��
  {
    nX = 2;
  }

  //-------------------����G----------------
  unsigned char g = (unsigned char)(nX * 50);

  //-------------------����B----------------
  float fPro = paramPro;  //
  int nU = (bGps ? 2 : 0);

  unsigned char b = (unsigned char)(nU * 50 + int(fPro * 10));

  return Color(0, g, b, 255);
}

int ColorAreaMap::GetAreaOrder(int type) {
  std::map<int, int>::iterator it_find = map_order_.find(type);

  if (map_order_.end() != it_find) {
    return it_find->second;
  } else {
    return 0;
  }
}

}  // namespace geditor