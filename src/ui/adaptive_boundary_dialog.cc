#include "adaptive_boundary_dialog.h"

#include <QButtonGroup>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QRadioButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include "core/framework.h"
#include "core/lane_segment.h"
#include "core/map_feature.h"
#include "ui/display_widget.h"

// 进程级参数缓存（UI 重启时才重置到默认）
double AdaptiveBoundaryDialog::s_min_lane_dual = 5.8;
double AdaptiveBoundaryDialog::s_max_lane_dual = 7.0;
double AdaptiveBoundaryDialog::s_max_lane_single = 5.8;
double AdaptiveBoundaryDialog::s_gap = 0.2;
double AdaptiveBoundaryDialog::s_taper = 10.0;
bool AdaptiveBoundaryDialog::s_local_mode = false;

AdaptiveBoundaryDialog::AdaptiveBoundaryDialog(DisplayWidget *display,
                                               QWidget *parent)
    : QDialog(parent), display_(display) {
  setWindowTitle("创建双向边界线 - 所有数值均为车道半宽（中心线→边界线距离）");
  resize(760, 560);

  // ---------- 参数输入 ----------
  spin_min_lane_dual_ = new QDoubleSpinBox(this);
  spin_min_lane_dual_->setRange(0.5, 100.0);
  spin_min_lane_dual_->setSingleStep(0.1);
  spin_min_lane_dual_->setDecimals(2);
  spin_min_lane_dual_->setSuffix(" 米");
  spin_min_lane_dual_->setValue(s_min_lane_dual);
  spin_min_lane_dual_->setToolTip(
      "双开均匀碾压·最低车道宽（半宽）：两中心线横向距离的一半大于该值时启用双开模式。");

  spin_max_lane_dual_ = new QDoubleSpinBox(this);
  spin_max_lane_dual_->setRange(0.5, 100.0);
  spin_max_lane_dual_->setSingleStep(0.1);
  spin_max_lane_dual_->setDecimals(2);
  spin_max_lane_dual_->setSuffix(" 米");
  spin_max_lane_dual_->setValue(s_max_lane_dual);
  spin_max_lane_dual_->setToolTip(
      "双开均匀碾压·最高车道宽（半宽）：半宽封顶，顶到时放宽内-内 gap 约束。");

  spin_max_lane_single_ = new QDoubleSpinBox(this);
  spin_max_lane_single_->setRange(0.5, 100.0);
  spin_max_lane_single_->setSingleStep(0.1);
  spin_max_lane_single_->setDecimals(2);
  spin_max_lane_single_->setSuffix(" 米");
  spin_max_lane_single_->setValue(s_max_lane_single);
  spin_max_lane_single_->setToolTip(
      "单开均匀碾压·最高车道宽（半宽）：下山优先占用该宽度，上山取 d - 该值 - 边界间距。");

  spin_gap_ = new QDoubleSpinBox(this);
  spin_gap_->setRange(0.0, 5.0);
  spin_gap_->setSingleStep(0.05);
  spin_gap_->setDecimals(2);
  spin_gap_->setSuffix(" 米");
  spin_gap_->setValue(s_gap);
  spin_gap_->setToolTip("对向内边界之间的最小间距。");

  spin_taper_ = new QDoubleSpinBox(this);
  spin_taper_->setRange(0.0, 50.0);
  spin_taper_->setSingleStep(0.5);
  spin_taper_->setDecimals(2);
  spin_taper_->setSuffix(" 米");
  spin_taper_->setValue(s_taper);
  spin_taper_->setToolTip(
      "平滑区间：段间接缝两侧各该长度内，把宽度平滑过渡到相邻段在该端点的最小宽度。");

  // ---------- 布局：三组 ----------
  auto *gDual = new QGroupBox("双开均匀碾压", this);
  auto *fDual = new QFormLayout(gDual);
  fDual->addRow("最低车道宽", spin_min_lane_dual_);
  fDual->addRow("最高车道宽", spin_max_lane_dual_);

  auto *gSingle = new QGroupBox("单开均匀碾压", this);
  auto *fSingle = new QFormLayout(gSingle);
  fSingle->addRow("最高车道宽", spin_max_lane_single_);

  auto *gCommon = new QGroupBox("平滑与边界", this);
  auto *fCommon = new QFormLayout(gCommon);
  fCommon->addRow("边界间距", spin_gap_);
  fCommon->addRow("平滑区间", spin_taper_);

  // ---------- 全局 / 局部 单选 ----------
  radio_global_ = new QRadioButton("全局（所有中心线）", this);
  radio_local_ = new QRadioButton("局部（仅手动选中的中心线）", this);
  auto *bgrp = new QButtonGroup(this);
  bgrp->addButton(radio_global_);
  bgrp->addButton(radio_local_);
  if (s_local_mode) radio_local_->setChecked(true);
  else radio_global_->setChecked(true);

  auto *gScope = new QGroupBox("作用范围", this);
  auto *hScope = new QHBoxLayout(gScope);
  hScope->addWidget(radio_global_);
  hScope->addWidget(radio_local_);
  hScope->addStretch();

  // ---------- 输出区 ----------
  lbl_summary_ = new QLabel("尚未运行", this);
  lbl_summary_->setWordWrap(true);

  txt_detail_ = new QTextEdit(this);
  txt_detail_->setReadOnly(true);
  txt_detail_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  txt_detail_->setPlaceholderText(
      "点击 [生成边界] 按当前参数为全部或选中的中心线重建左右边界。\n"
      "所有 \"车道宽\" 参数均指车道半宽（中心线到边界的距离）。\n"
      "局部模式下需先在主视图手动选中若干中心线再执行。");

  btn_run_ = new QPushButton("生成边界", this);
  btn_run_->setStyleSheet("font-weight:bold; padding:6px 18px;");
  btn_run_->setDefault(true);
  btn_close_ = new QPushButton("关闭", this);

  auto *btns = new QHBoxLayout;
  btns->addStretch();
  btns->addWidget(btn_run_);
  btns->addWidget(btn_close_);

  auto *root = new QVBoxLayout(this);
  // 上半：参数三组水平并排，节省纵向空间
  auto *paramRow = new QHBoxLayout;
  paramRow->addWidget(gDual);
  paramRow->addWidget(gSingle);
  paramRow->addWidget(gCommon);
  root->addLayout(paramRow);
  root->addWidget(gScope);
  root->addWidget(lbl_summary_);
  root->addWidget(txt_detail_, 1);
  root->addLayout(btns);

  connect(btn_run_, &QPushButton::clicked, this, &AdaptiveBoundaryDialog::OnRun);
  connect(btn_close_, &QPushButton::clicked, this, &QDialog::accept);
}

void AdaptiveBoundaryDialog::OnRun() {
  if (!display_ || !display_->framework()) return;

  // 1) 记忆参数到静态缓存
  s_min_lane_dual = spin_min_lane_dual_->value();
  s_max_lane_dual = spin_max_lane_dual_->value();
  s_max_lane_single = spin_max_lane_single_->value();
  s_gap = spin_gap_->value();
  s_taper = spin_taper_->value();
  s_local_mode = radio_local_->isChecked();

  geditor::Framework::AdaptiveBoundaryParams params;
  params.min_lane_width_dual = s_min_lane_dual;
  params.max_lane_width_dual = s_max_lane_dual;
  params.max_lane_width_single = s_max_lane_single;
  params.boundary_gap = s_gap;
  params.taper_len = s_taper;

  geditor::Framework::AdaptiveBoundaryDiagnostic diag;
  int n = 0;

  if (s_local_mode) {
    // 从选中的 MapFeature 过滤出 LaneSegment
    std::vector<geditor::LaneSegment *> selLanes;
    auto sel = display_->GetSelected();
    for (auto *f : sel) {
      if (f && f->GetType() == geditor::MapFeature::MFT_LANE_SEG) {
        selLanes.push_back(static_cast<geditor::LaneSegment *>(f));
      }
    }
    if (selLanes.empty()) {
      QMessageBox::warning(
          this, "创建双向边界线",
          "局部模式需要先在主视图选中至少一条中心线，再执行。");
      return;
    }
    n = display_->framework()->GenerateAdaptiveBoundariesForSelected(
        params, selLanes, &diag);
  } else {
    n = display_->framework()->GenerateAdaptiveBoundaries(params, &diag);
  }

  QString scope = s_local_mode ? "局部" : "全局";
  QString summary = QString(
                        "[%1] 总中心线=%2  处理=%3  双开=%4  单开=%5  "
                        "跳过=%6  本次写入=%7")
                        .arg(scope)
                        .arg(diag.totalLanes)
                        .arg(diag.processed)
                        .arg(diag.dualWide)
                        .arg(diag.singleWide)
                        .arg(diag.skipped)
                        .arg(n);
  lbl_summary_->setText(summary);
  txt_detail_->setPlainText(QString::fromStdString(diag.detail));
  display_->update();
}
