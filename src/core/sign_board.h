
#pragma once

#include "core/geo_polyline.h"
#include "core/map_feature.h"
#include "core/vector_style.h"

namespace geditor {

class LaneSegment;

class RoadArea;

class SignBoard : public MapFeature {
 public:
  SignBoard();

  virtual ~SignBoard();

 public:
  bool IsChanged();

  void SetChanged(bool change);

  virtual void SetSelectedState(bool selected);

  virtual void SetHighlightState(bool high);

  void SetHighlightPoint(int index);

  void SetProperty(SignBoardProperty *pProperty);

  int GetHighlightPoint();

  PolygonSytle *GetStyle();

  SignBoardProperty *GetProperty();

  void SetSignboardType(int type);

  //��ȡ���й�������
  void GetRelationSegment(std::vector<LaneSegment *> &segment);

  //��ӹ�������
  void AddRelationSegment(LaneSegment *segment);

  //�Ƿ��������
  bool IsRelationSegment(LaneSegment *segment);

  //���ù���ֹͣ��
  void SetRelationStopline(RoadArea *roadArea);

  //���ع���ֹͣ��
  RoadArea *GetRelationStopline() const;

 private:
  SignBoardProperty m_property;
  std::vector<LaneSegment *> m_relationSegment;
  RoadArea *m_relationStopline;

 private:
  bool m_Changed;
  PolygonSytle *m_lineStyle = NULL;
  int m_highlightPoint;
};

}  // namespace geditor
