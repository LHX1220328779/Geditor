#include "menu_widget.h"

#include <QMessageBox>

#include "display_widget.h"

MenuWidget::MenuWidget(DisplayWidget* win) : win_(win) {
  menu_ = new QMenu((QWidget*)win_);
  menu_->addAction("边界创建车道", this, &MenuWidget::CreateLane);
  // menu_->addAction("设置并行车道组", this, &MenuWidget::SetLaneGroup);
  menu_->addAction("设置反向车道组", this, &MenuWidget::SetReverseLaneGroup);
  menu_->addAction("中心线创建左右边界", this,
                   &MenuWidget::CreateBoundaryByLane);
  menu_->addAction("中心边界创建正反双车道", this,
                   &MenuWidget::CreateReverseLaneGroup2);
  menu_->addAction("两侧边界创建正反双车道", this,
                   &MenuWidget::CreateReverseLaneGroup);
  menu_->addAction("车道设置边界", this, &MenuWidget::SetLaneBoundary);
  menu_->addAction("反向", this, &MenuWidget::ReverseObj);
  // menu_->addAction("标志 路面区关联到车道(同时车道关联到标志)", this,
  //                  &MenuWidget::SetLaneRelation);
  // menu_->addAction("停止线关联到交通标志", this,
  // &MenuWidget::SetSignRelation);
  // menu_->addAction("功能区(点)关联车道", this, &MenuWidget::SetLaneRelation);
  menu_->addAction("边界插入点", this, &MenuWidget::InsertPointBoundary);
}
void MenuWidget::Exec(const QPoint& p) { menu_->exec(p); }

void MenuWidget::SetLaneBoundary() {
  int re = win_->framework()->SetLaneBoundary();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "选择对象需为一条车道中线和两条边界");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "两条边界需在车道两侧");
}
void MenuWidget::CreateLane() {
  int re = win_->framework()->CreateLaneByBoundary();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "选择对象需为两条边界");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "生成中心线失败");
}
void MenuWidget::SetLaneGroup() {
  int re = win_->framework()->SetParallelSegment();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "选择对象需全部为车道中线");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "选择车道数量小于2条");
}
void MenuWidget::SetReverseLaneGroup() {
  int re = win_->framework()->SetReverseSegment();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "选择对象需全部为车道中线");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "选择车道数量不等于2条");
}
void MenuWidget::CreateReverseLaneGroup2() {
  int re = win_->framework()->CreateReverseLaneGroup2();
  if (re < 0) QMessageBox::warning(win_, "警告", "选择对象需为一条边界");
}

void MenuWidget::CreateBoundaryByLane() {
  int re = win_->framework()->CreateBoundaryByLane();
  if (re < 0) QMessageBox::warning(win_, "警告", "选择对象需为一条车道中心线");
}
void MenuWidget::CreateReverseLaneGroup() {
  int re = win_->framework()->CreateReverseLaneGroup();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "选择对象需为两条边界");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "边界方向不支持构建正反双车道");
}
void MenuWidget::SetLaneRelation() {
  int re = win_->framework()->SetSegmentRelation();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "需要至少选择一条车道中线");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "需要至少一个功能区或功能点");
  // QMessageBox::warning(win_, "警告", "需要至少一个交通标志或路面区域");
}
void MenuWidget::SetSignRelation() {
  int re = win_->framework()->SetTrafficStopline();
  if (re < 0)
    QMessageBox::warning(win_, "警告", "需要至少选择一个交通标志");
  else if (re == 0)
    QMessageBox::warning(win_, "警告", "需要至少一个关联对象");
}

void MenuWidget::ReverseObj() {
  int re = win_->framework()->ReverseObj();
  if (re < 0) QMessageBox::warning(win_, "警告", "该对象不可反向");
}

void MenuWidget::InsertPointBoundary() {
  win_->framework()->FillPointsOnSelectedBoundary(1.0);
  // if (re < 0)
  //   QMessageBox::warning(win_, "警告", "需要至少选择一条车道中线");
  // else if (re == 0)
  //   QMessageBox::warning(win_, "警告", "需要至少一个功能区或功能点");
  // // QMessageBox::warning(win_, "警告", "需要至少一个交通标志或路面区域");
}