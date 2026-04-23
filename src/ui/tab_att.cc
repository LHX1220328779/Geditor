#include "ui/tab_att.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QSpinBox>
#include <cstring>

#include "core/bound_segment.h"
#include "core/job_area.h"
#include "core/lane_segment.h"
#include "core/road_area.h"
#include "core/sign_board.h"
#include "map/feature_type.h"
#include "ui/geditor_mainwindow.h"

namespace geditor
{

  TabAtt::TabAtt(QTableWidget *tab) : tab_att_(tab)
  {
    tab_att_->setColumnCount(2);
    tab_att_->setColumnWidth(0, 100);

    QTableWidgetItem *item0 = new QTableWidgetItem("属性");
    QTableWidgetItem *item1 = new QTableWidgetItem("值");

    tab_att_->setHorizontalHeaderItem(0, item0);
    tab_att_->setHorizontalHeaderItem(1, item1);
    tab_att_->setEditTriggers(QTableWidget::DoubleClicked);
    connect(tab_att_, SIGNAL(cellChanged(int, int)), this,
            SLOT(CellChanged(int, int)));
    tab_att_->setRowCount(32);
  }

  void TabAtt::OnSelected(std::vector<MapFeature *> feats)
  {
    if (feats.size() > 1)
    {
      static MapFeature *tem = new MapFeature(0);
      show_ = false;
      selected_ = tem;
      ShowSize(feats);
      return;
    }
    MapFeature *feat = nullptr;
    if (!feats.empty())
      feat = feats.back();
    if (selected_ == feat)
      return;
    else
      selected_ = feat;
    tab_att_->clearContents();
    if (!feat)
    {
      return;
    }
    show_ = false;
    switch (feat->GetType())
    {
    case MapFeature::MFT_BOUNDARY:
      OnSelectedLine(feat);
      break;
    case MapFeature::MFT_LANE_SEG:
      OnSelectedLane(feat);
      break;
    case MapFeature::MFT_ROAD_AREA:
      OnSelectedRoadArea(feat);
      break;
    case MapFeature::MFT_SIGNBORAD:
      OnSelectedSign(feat);
      break;
    case MapFeature::MFT_JOB_AREA:
      OnSelectedJobArea(feat);
      break;

    default:
      OnSelectedNone();
      break;
    }
    show_ = true;
  }

  void TabAtt::ShowSize(std::vector<MapFeature *> feats)
  {
    tab_att_->clearContents();
    int line = 0, lane = 0, area = 0, sign = 0;
    for (auto f : feats)
    {
      switch (f->GetType())
      {
      case MapFeature::MFT_BOUNDARY:
        line++;
        break;
      case MapFeature::MFT_LANE_SEG:
        lane++;
        break;
      case MapFeature::MFT_ROAD_AREA:
        area++;
        break;
      case MapFeature::MFT_SIGNBORAD:
        sign++;
        break;
      default:
        break;
      }
    }
    int idx = 0;
    if (line)
      SetItems(idx++, "boundary", line);
    if (lane)
      SetItems(idx++, "lane", lane);
    if (area)
      SetItems(idx++, "road area", area);
    if (sign)
      SetItems(idx++, "sign", sign);
  }

  void TabAtt::CellChanged(int r, int c)
  {
    if (!show_)
      return;
    LOG(INFO) << tab_att_->item(r, c)->text().toStdString();
  }

  void TabAtt::OnSelectedLine(MapFeature *feat)
  {
    BoundSegment *seg = (BoundSegment *)feat;
    SetItems(0, "id", feat->GetUniqueID());
    tab_att_->setItem(1, 0, NewItem("type"));
    QComboBox *combox = new QComboBox();
    for (auto &s : kBoundaryType)
      combox->addItem(s.name.c_str(), s.type);
    int re = FeatType::Find(kBoundaryType, seg->GetProperty()->boundType);
    combox->setCurrentIndex(re);
    tab_att_->setCellWidget(1, 1, (QWidget *)combox);
    connect(combox, SIGNAL(currentIndexChanged(int)), this,
            SLOT(TypeChanged(int)));
    SetItems(2, "legth", seg->GetProperty()->length);
    int idx = ShowFeatruePoint(feat, 3);
    if (seg->GetProperty()->lanePitch > 0)
    {
      SetItems(idx, "seg_idx", seg->GetProperty()->lanePitch); // 存储了导入数据的索引，该索引对应上导入数据的名字
    }
  }
  void TabAtt::OnSelectedLane(MapFeature *feat)
  {
    LaneSegment *seg = (LaneSegment *)feat;
    int idx = 0;
    SetItems(idx++, "id", feat->GetUniqueID());
    // SetItems(1, "type", seg->GetProperty()->laneType, 35);
    {
      tab_att_->setItem(idx, 0, NewItem("type"));
      QComboBox *combox = new QComboBox();
      for (auto &s : kLaneType)
        combox->addItem(s.name.c_str(), s.type);
      int re = FeatType::Find(kLaneType, seg->GetProperty()->laneType);
      combox->setCurrentIndex(re);
      tab_att_->setCellWidget(idx++, 1, (QWidget *)combox);
      connect(
          combox,
          static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          [this](int t)
          { this->TypeChanged(t, "type"); });
    }

    {
      tab_att_->setItem(idx, 0, NewItem("矿山路段索引"));
      QSpinBox *spin = new QSpinBox();
      spin->setMinimum(0);
      spin->setMaximum(1000000000);
      spin->setValue(seg->GetProperty()->mineSegmentIndex);
      tab_att_->setCellWidget(idx++, 1, spin);
      connect(spin,
              static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
              [this](int val)
              {
                ((LaneSegment *)selected_)->GetProperty()->mineSegmentIndex = val;
              });
    }

    {
      tab_att_->setItem(idx, 0, NewItem("矿山路段编号"));
      QLineEdit *le = new QLineEdit();
      le->setText(QString(seg->GetProperty()->mineSegmentCode));
      tab_att_->setCellWidget(idx, 1, le);
      connect(le, &QLineEdit::textChanged, [this](QString val)
              {
      QByteArray src = val.toUtf8();
      auto p = ((LaneSegment*)selected_)->GetProperty();
      std::memset(p->mineSegmentCode, 0, sizeof(p->mineSegmentCode));
      int size = sizeof(p->mineSegmentCode) - 1;
      if (src.size() < size) size = src.size();
      strncpy(p->mineSegmentCode, src.constData(), size); });
      idx++;
    }

    {
      tab_att_->setItem(idx, 0, NewItem("road_right"));
      QCheckBox *cb = new QCheckBox();
      cb->setChecked(seg->GetProperty()->road_right != 0);
      cb->setText(seg->GetProperty()->road_right ? "均匀碾压(1)" : "不均匀碾压(0)");
      tab_att_->setCellWidget(idx, 1, cb);
      connect(cb, &QCheckBox::toggled, [this, cb](bool v)
              {
      ((LaneSegment*)selected_)->GetProperty()->road_right = v ? 1 : 0;
      cb->setText(v ? "均匀碾压(1)" : "不均匀碾压(0)"); });
      idx++;
    }

    {
      tab_att_->setItem(idx, 0, NewItem("行驶方向"));
      QComboBox *combox = new QComboBox();
      combox->addItem("未设定", 0);
      combox->addItem("上山", 1);
      combox->addItem("下山", 2);
      int cur = seg->GetProperty()->direction;
      if (cur < 0 || cur > 2) cur = 0;
      combox->setCurrentIndex(cur);
      tab_att_->setCellWidget(idx, 1, combox);
      connect(combox,
              static_cast<void (QComboBox::*)(int)>(
                  &QComboBox::currentIndexChanged),
              [this](int t) {
                ((LaneSegment *)selected_)->GetProperty()->direction = t;
              });
      idx++;
    }

    {
      tab_att_->setItem(idx, 0, NewItem("限速km/h"));
      QDoubleSpinBox *ds = new QDoubleSpinBox();
      ds->setMinimum(-10000.0);
      ds->setMaximum(10000.0);
      ds->setDecimals(3);
      ds->setValue(seg->GetProperty()->speed);
      tab_att_->setCellWidget(idx++, 1, ds);
      connect(ds,
              static_cast<void (QDoubleSpinBox::*)(double)>(
                  &QDoubleSpinBox::valueChanged),
              [this](double val)
              {
                ((LaneSegment *)selected_)->GetProperty()->speed = val;
              });
    }

    // {
    //   tab_att_->setItem(idx, 0, NewItem("turn_type"));
    //   QComboBox* combox = new QComboBox();
    //   for (auto& s : kLaneTurnType) combox->addItem(s.name.c_str(), s.type);
    //   int re = FeatType::Find(kLaneTurnType, seg->GetProperty()->turnType);
    //   combox->setCurrentIndex(re);
    //   tab_att_->setCellWidget(idx++, 1, (QWidget*)combox);
    //   connect(
    //       combox,
    //       static_cast<void
    //       (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this](int t)
    //       { this->TypeChanged(t, "turn"); });
    // }

    SetItems(idx++, "lane_seq", seg->GetProperty()->laneSeq);
    if (seg->GetProperty()->leftBoundary)
      SetItems(idx++, "左边界", seg->GetProperty()->leftBoundary);
    else
      SetItems(idx++, "左边界", "");
    if (seg->GetProperty()->rightBoundary)
      SetItems(idx++, "右边界", seg->GetProperty()->rightBoundary);
    else
      SetItems(idx++, "右边界", "");
    std::vector<int> segs;
    seg->GetSuccessorSegment(segs);
    SetItems(idx++, "后续车道数", segs.size());
    std::vector<int> segsr;
    seg->GetPredecessorSegment(segsr);
    SetItems(idx++, "前续车道数", segsr.size());
    // if (seg->GetParallelSegment()->leftSegment)
    //   SetItems(9, "左侧车道", seg->GetParallelSegment()->leftSegment);
    // else
    //   SetItems(9, "左侧车道", "");
    // if (seg->GetParallelSegment()->rightSegment)
    //   SetItems(10, "右侧车道", seg->GetParallelSegment()->rightSegment);
    // else
    //   SetItems(10, "右侧车道", "");

    if (seg->GetParallelSegment()->leftReverseSegment)
      SetItems(idx++, "左侧反向车道",
               seg->GetParallelSegment()->leftReverseSegment);
    else
      SetItems(idx++, "左侧反向车道", "");
    if (seg->GetParallelSegment()->rightReverseSegment)
      SetItems(idx++, "右侧反向车道",
               seg->GetParallelSegment()->rightReverseSegment);
    else
      SetItems(idx++, "右侧反向车道", "");
    if (seg->GetPredecessorFeature())
      SetItems(idx++, "前续功能区", seg->GetPredecessorFeature()->GetUniqueID());
    else
      SetItems(idx++, "前续功能区", "");
    if (seg->GetSuccessorFeature())
      SetItems(idx++, "后继功能区", seg->GetSuccessorFeature()->GetUniqueID());
    else
      SetItems(idx++, "后继功能区", "");

    SetItems(idx++, "关联数量", seg->AttachObjectSize());
    std::vector<MapFeature *> objs;
    seg->GetAttachObject(objs);
    for (auto &obj : objs)
      SetItems(idx++, "关联对象", obj->GetUniqueID());
    idx = ShowFeatruePoint(feat, idx);
    SetItems(idx++, "legth", seg->GetProperty()->length);
    {
      tab_att_->setItem(idx, 0, NewItem("name"));
      QLineEdit *le = new QLineEdit();
      le->setText(QString(seg->GetProperty()->name));
      tab_att_->setCellWidget(idx, 1, le);
      connect(le, &QLineEdit::textChanged, [this](QString val)
              {
      QByteArray src = val.toUtf8();
      auto p = ((LaneSegment*)selected_)->GetProperty();
      std::memset(p->name, 0, sizeof(p->name));
      int size = sizeof(p->name) - 1;
      if (src.size() < size) size = src.size();
      strncpy(p->name, src.constData(), size); });
      idx++;
    }
  }
  void TabAtt::OnSelectedRoadArea(MapFeature *feat)
  {
    RoadArea *seg = (RoadArea *)feat;
    SetItems(0, "id", feat->GetUniqueID());
    tab_att_->setItem(1, 0, NewItem("type"));
    QComboBox *combox = new QComboBox();
    for (auto &s : kRoadAreaType)
      combox->addItem(s.name.c_str(), s.type);
    int re = FeatType::Find(kRoadAreaType, seg->GetProperty()->areaType);
    combox->setCurrentIndex(re);
    tab_att_->setCellWidget(1, 1, (QWidget *)combox);
    connect(combox, SIGNAL(currentIndexChanged(int)), this,
            SLOT(TypeChanged(int)));
    ShowFeatruePoint(feat, 2);
  }
  void TabAtt::OnSelectedJobArea(MapFeature *feat)
  {
    JobArea *seg = (JobArea *)feat;
    int idx = 0;
    SetItems(idx++, "id", feat->GetUniqueID());
    tab_att_->setItem(idx++, 0, NewItem("type"));
    QComboBox *combox = new QComboBox();
    bool area = seg->GetProperty()->areaType < TV::jobpoint;
    int n = 0;
    for (auto &s : kJobAreaType)
    {
      combox->addItem(s.name.c_str(), s.type);
      bool sim =
          (area && s.type < TV::jobpoint) || (!area && s.type >= TV::jobpoint);
      if (!sim)
        combox->setItemData(n, QVariant(0), Qt::UserRole - 1);
      // if ((area && s.type < TV::jobpoint) || (!area && s.type >= TV::jobpoint))
      //   combox->addItem(s.name.c_str(), s.type);
      n++;
    }
    int re = FeatType::Find(kJobAreaType, seg->GetProperty()->areaType);
    combox->setCurrentIndex(re);
    tab_att_->setCellWidget(1, 1, (QWidget *)combox);
    connect(combox, SIGNAL(currentIndexChanged(int)), this,
            SLOT(TypeChanged(int)));

    std::vector<MapFeature *> objs;
    seg->GetAttachObject(objs);
    SetItems(idx++, "关联数量", objs.size());
    for (auto &obj : objs)
      SetItems(idx++, "关联对象", obj->GetUniqueID());
    ShowFeatruePoint(feat, idx);
  }
  void TabAtt::OnSelectedSign(MapFeature *feat)
  {
    SignBoard *seg = (SignBoard *)feat;
    SetItems(0, "id", feat->GetUniqueID());
    tab_att_->setItem(1, 0, NewItem("type"));
    QComboBox *combox = new QComboBox();
    // combox->setEditable(true);
    // combox->setAutoCompletion(true);
    for (auto &s : kSignType)
      combox->addItem(s.name.c_str(), s.type);
    int re = FeatType::Find(kSignType, seg->GetProperty()->areaType);
    combox->setCurrentIndex(re);
    tab_att_->setCellWidget(1, 1, (QWidget *)combox);
    connect(combox, SIGNAL(currentIndexChanged(int)), this,
            SLOT(TypeChanged(int)));
    if (seg->GetRelationStopline())
      SetItems(2, "关联停止线", seg->GetRelationStopline()->GetUniqueID());
    else
      SetItems(2, "关联停止线", "");
    int idx = 3;
    std::vector<LaneSegment *> lanes;
    seg->GetRelationSegment(lanes);
    for (auto &lane : lanes)
      SetItems(idx++, "关联车道", lane->GetUniqueID());
    if (lanes.empty())
      SetItems(idx++, "关联车道", "");
    ShowFeatruePoint(feat, idx);
  }

  void TabAtt::OnSelectedNone() { tab_att_->clearContents(); }
  void TabAtt::SetItemNo(int c)
  {
    for (int i = 0; i < tab_att_->rowCount(); ++i)
      if (tab_att_->item(i, c))
        tab_att_->item(i, c)->setFlags((Qt::ItemFlags)33);
  }
  void TabAtt::SetItemEdit(int r, int c)
  {
    if (tab_att_->item(r, c))
      tab_att_->item(r, c)->setFlags((Qt::ItemFlags)63);
  }

  void TabAtt::TypeChanged(int i, std::string str)
  {
    switch (selected_->GetType())
    {
    case MapFeature::MFT_BOUNDARY:
    {
      BoundSegment *seg = (BoundSegment *)selected_;
      seg->GetProperty()->boundType = kBoundaryType[i].type;
      break;
    }
    case MapFeature::MFT_LANE_SEG:
    {
      LaneSegment *seg = (LaneSegment *)selected_;
      if (str == "type")
        seg->GetProperty()->laneType = kLaneType[i].type;
      else if (str == "turn")
        seg->GetProperty()->turnType = kLaneTurnType[i].type;
      break;
    }
    break;
    case MapFeature::MFT_ROAD_AREA:
    {
      RoadArea *seg = (RoadArea *)selected_;
      seg->GetProperty()->areaType = kRoadAreaType[i].type;
      break;
    }
    case MapFeature::MFT_SIGNBORAD:
    {
      SignBoard *seg = (SignBoard *)selected_;
      seg->GetProperty()->areaType = kSignType[i].type;
      break;
    }
    case MapFeature::MFT_JOB_AREA:
    {
      JobArea *seg = (JobArea *)selected_;
      seg->GetProperty()->areaType = kJobAreaType[i].type;
      break;
    }
    default:
      break;
    }
  }
  int TabAtt::ShowFeatruePoint(MapFeature *feat, int idx)
  {
    int size = feat->GetGeometry()->GetVertexCount();
    SetItems(idx++, "点数量", size);
    SetItems(idx++, "起点id", feat->GetGeometry()->GetStartVertex().Id());
    SetItems(idx++, "终点id", feat->GetGeometry()->GetEndVertex().Id());
    return idx;
  }

} // namespace geditor
