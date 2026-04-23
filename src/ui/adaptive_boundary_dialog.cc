#include "adaptive_boundary_dialog.h"

#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include "core/framework.h"
#include "ui/display_widget.h"

AdaptiveBoundaryDialog::AdaptiveBoundaryDialog(DisplayWidget *display,
                                               QWidget *parent)
    : QDialog(parent), display_(display) {
  setWindowTitle("自适应边界生成 (R3 + R4 平滑)");
  resize(760, 560);

  spin_min_lane_ = new QDoubleSpinBox(this);
  spin_min_lane_->setRange(0.5, 100.0);
  spin_min_lane_->setSingleStep(0.1);
  spin_min_lane_->setDecimals(2);
  spin_min_lane_->setValue(5.8);
  spin_min_lane_->setSuffix(" 米");
  spin_min_lane_->setToolTip(
      "双开均匀碾压·最低车道宽：当整段轨迹对向距离/2 > 该值时，左右边界按 "
      "(d/2 - 边界间距) 生成，每顶点变宽，均匀碾压");

  spin_max_lane_ = new QDoubleSpinBox(this);
  spin_max_lane_->setRange(0.5, 100.0);
  spin_max_lane_->setSingleStep(0.1);
  spin_max_lane_->setDecimals(2);
  spin_max_lane_->setValue(9.0);
  spin_max_lane_->setSuffix(" 米");
  spin_max_lane_->setToolTip(
      "双开均匀碾压·最高车道宽：单侧车道(inner+outer)不超过该值。"
      "顶到上限时放宽内-内 gap 约束，避免边界线过宽。");

  spin_rec_lane_ = new QDoubleSpinBox(this);
  spin_rec_lane_->setRange(0.5, 100.0);
  spin_rec_lane_->setSingleStep(0.1);
  spin_rec_lane_->setDecimals(2);
  spin_rec_lane_->setValue(4.0);
  spin_rec_lane_->setSuffix(" 米");
  spin_rec_lane_->setToolTip(
      "单开均匀碾压·建议车道宽：对向距离/2 不达阈值时，下山用该宽度，\n"
      "上山用 (对向距离 - 建议车道宽 - 边界间距)");

  spin_gap_ = new QDoubleSpinBox(this);
  spin_gap_->setRange(0.0, 5.0);
  spin_gap_->setSingleStep(0.05);
  spin_gap_->setDecimals(2);
  spin_gap_->setValue(0.1);
  spin_gap_->setSuffix(" 米");
  spin_gap_->setToolTip("边界间距：左右边界相对理论宽度的收缩间距");

  spin_taper_ = new QDoubleSpinBox(this);
  spin_taper_->setRange(0.0, 50.0);
  spin_taper_->setSingleStep(0.5);
  spin_taper_->setDecimals(2);
  spin_taper_->setValue(5.0);
  spin_taper_->setSuffix(" 米");
  spin_taper_->setToolTip(
      "taper 长度：段间接缝两侧各 taper_len 米，把宽度线性过渡到\n"
      "相邻段在该端点的最小宽度，保证边界无断层又保留中段均匀碾压");

  auto *gDual = new QGroupBox("双开均匀碾压", this);
  auto *fDual = new QFormLayout(gDual);
  fDual->addRow("最低车道宽", spin_min_lane_);
  fDual->addRow("最高车道宽", spin_max_lane_);

  auto *gSingle = new QGroupBox("单开均匀碾压", this);
  auto *fSingle = new QFormLayout(gSingle);
  fSingle->addRow("建议车道宽", spin_rec_lane_);

  auto *gGlobal = new QGroupBox("平滑与边界", this);
  auto *fGlobal = new QFormLayout(gGlobal);
  fGlobal->addRow("边界间距", spin_gap_);
  fGlobal->addRow("taper 长度", spin_taper_);

  lbl_summary_ = new QLabel("尚未运行", this);
  lbl_summary_->setWordWrap(true);

  txt_detail_ = new QTextEdit(this);
  txt_detail_->setReadOnly(true);
  txt_detail_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  txt_detail_->setPlaceholderText(
      "点击 [生成边界] 将按当前参数为所有中心线重建左右边界。\n"
      "中段保持均匀碾压，接缝两侧 taper 区线性收敛到相邻段的最小宽度。");

  btn_run_ = new QPushButton("生成边界", this);
  btn_run_->setStyleSheet("font-weight:bold;");
  btn_close_ = new QPushButton("关闭", this);

  auto *btns = new QHBoxLayout;
  btns->addStretch();
  btns->addWidget(btn_run_);
  btns->addWidget(btn_close_);

  auto *root = new QVBoxLayout(this);
  root->addWidget(gDual);
  root->addWidget(gSingle);
  root->addWidget(gGlobal);
  root->addWidget(lbl_summary_);
  root->addWidget(txt_detail_, 1);
  root->addLayout(btns);

  connect(btn_run_, &QPushButton::clicked, this, &AdaptiveBoundaryDialog::OnRun);
  connect(btn_close_, &QPushButton::clicked, this, &QDialog::accept);
}

void AdaptiveBoundaryDialog::OnRun() {
  if (!display_ || !display_->framework()) return;
  geditor::Framework::AdaptiveBoundaryParams params;
  params.min_lane_width = spin_min_lane_->value();
  params.max_lane_width = spin_max_lane_->value();
  params.recommended_lane_width = spin_rec_lane_->value();
  params.boundary_gap = spin_gap_->value();
  params.taper_len = spin_taper_->value();
  geditor::Framework::AdaptiveBoundaryDiagnostic diag;
  int n = display_->framework()->GenerateAdaptiveBoundaries(params, &diag);
  QString summary =
      QString("总中心线=%1  处理=%2  双开=%3  单开=%4  跳过=%5  本次写入=%6")
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
