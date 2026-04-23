#pragma once

#include <QMenu>
#include <QPoint>

class DisplayWidget;

class MenuWidget : public QObject {
 public:
  MenuWidget(DisplayWidget* win);
  void Exec(const QPoint& p);

 private:
  void SetLaneBoundary();
  void CreateLane();
  void SetLaneGroup();
  void SetReverseLaneGroup();
  void CreateReverseLaneGroup();
  void CreateReverseLaneGroup2();
  void CreateBoundaryByLane();
  void SetLaneRelation();
  void SetSignRelation();
  void ReverseObj();
  void InsertPointBoundary();

 private:
  DisplayWidget* win_;
  QMenu* menu_;
};