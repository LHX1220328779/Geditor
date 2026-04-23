#pragma once

#include <QComboBox>
#include <QTableWidget>

#include "core/map_feature.h"

namespace geditor {

class TabAtt : public QObject {
  Q_OBJECT
 public:
  TabAtt(QTableWidget *tab);
  ~TabAtt() {}

  void OnSelected(std::vector<MapFeature *> feats);

  // 强制下次 OnSelected 重新渲染（属性被外部修改后调用）
  void InvalidateSelection() { selected_ = nullptr; }

 private:
  void ShowSize(std::vector<MapFeature *> feats);
  void OnSelectedLine(MapFeature *feat);
  void OnSelectedLane(MapFeature *feat);
  void OnSelectedRoadArea(MapFeature *feat);
  void OnSelectedSign(MapFeature *feat);
  void OnSelectedJobArea(MapFeature *feat);

  void OnSelectedNone();
  void SetItemNo(int c);
  void SetItemEdit(int r, int c);

  QTableWidgetItem *NewItem(const QString &text, int flag = 33) {
    QTableWidgetItem *item = new QTableWidgetItem(text);
    item->setFlags((Qt::ItemFlags)flag);
    if (flag <= 33) item->setTextColor(QColor("gray"));
    return item;
  }
  int ShowFeatruePoint(MapFeature *feat, int idx);

  QTableWidgetItem *NewItemByVal(const QString &val, int flag) {
    return NewItem(val, flag);
  }
  QTableWidgetItem *NewItemByVal(const char *val, int flag) {
    return NewItem(QString(val), flag);
  }

  template <typename T>
  QTableWidgetItem *SetItems(int r, QString key, T val, int flag = 33) {
    tab_att_->setItem(r, 0, NewItem(key));
    auto item = NewItemByVal(val, flag);
    tab_att_->setItem(r, 1, item);
    return item;
  }

  template <typename T>
  QTableWidgetItem *NewItemByVal(T val, int flag) {
    return NewItem(QString::number(val), flag);
  }

 private slots:
  void TypeChanged(int i, std::string str = "");
  void CellChanged(int r, int c);

 private:
  QTableWidget *tab_att_;
  MapFeature *selected_ = nullptr;
  bool show_ = false;
  // std::map<int, std::function<void(int idx)>>
};

// template <>
// QTableWidgetItem *TabAtt::NewItemByVal<QString>(QString val, int flag) {
//   return NewItem(val, flag);
// }

}  // namespace geditor