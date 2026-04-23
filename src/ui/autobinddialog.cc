#include "autobinddialog.h"

#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include "ui/display_widget.h"

AutoBindDialog::AutoBindDialog(DisplayWidget *display, QWidget *parent)
    : QDialog(parent), display_(display) {
  setWindowTitle("自动绑定轨迹 - 阈值调试");
  resize(720, 520);

  spin_dist_tol_ = new QDoubleSpinBox(this);
  spin_dist_tol_->setRange(0.1, 200.0);
  spin_dist_tol_->setSingleStep(0.5);
  spin_dist_tol_->setDecimals(2);
  spin_dist_tol_->setValue(10.0);
  spin_dist_tol_->setSuffix(" 米");
  spin_dist_tol_->setToolTip(
      "走廊宽度：中心线/轨迹上的采样点距离另一条折线小于此值则视为\"重合\"。\n"
      "推荐范围 5-20m。越大越宽松。");

  spin_coverage_ = new QDoubleSpinBox(this);
  spin_coverage_->setRange(0.0, 1.0);
  spin_coverage_->setSingleStep(0.05);
  spin_coverage_->setDecimals(2);
  spin_coverage_->setValue(0.30);
  spin_coverage_->setToolTip(
      "覆盖率阈值：0~1。\n"
      "两条折线互相落入对方走廊内的点数比例的调和均值。\n"
      "低于此值则视为不匹配。越低越宽松。");

  auto *form = new QFormLayout;
  form->addRow("走廊宽度 (distTol)", spin_dist_tol_);
  form->addRow("覆盖率阈值", spin_coverage_);

  lbl_summary_ = new QLabel("尚未运行", this);
  lbl_summary_->setWordWrap(true);

  txt_detail_ = new QTextEdit(this);
  txt_detail_->setReadOnly(true);
  txt_detail_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  txt_detail_->setPlaceholderText(
      "点击\"预览\"按当前阈值计算并显示每条中心线的匹配情况，但不修改数据。\n"
      "点击\"执行绑定\"正式写入 mineSegmentIndex / mineSegmentCode。");

  btn_preview_ = new QPushButton("预览 (不修改)", this);
  btn_apply_ = new QPushButton("执行绑定", this);
  btn_close_ = new QPushButton("关闭", this);
  btn_apply_->setStyleSheet("font-weight:bold; color:#a00;");

  auto *btns = new QHBoxLayout;
  btns->addStretch();
  btns->addWidget(btn_preview_);
  btns->addWidget(btn_apply_);
  btns->addWidget(btn_close_);

  auto *root = new QVBoxLayout(this);
  root->addLayout(form);
  root->addWidget(lbl_summary_);
  root->addWidget(txt_detail_, 1);
  root->addLayout(btns);

  connect(btn_preview_, &QPushButton::clicked, this, &AutoBindDialog::OnPreview);
  connect(btn_apply_, &QPushButton::clicked, this, &AutoBindDialog::OnApply);
  connect(btn_close_, &QPushButton::clicked, this, &QDialog::accept);
}

void AutoBindDialog::OnPreview() { Run(true); }

void AutoBindDialog::OnApply() {
  auto ret = QMessageBox::question(
      this, "确认",
      "将按当前阈值写入 mineSegmentIndex/mineSegmentCode 到所有匹配成功的"
      "中心线。\n该操作可通过撤销恢复(如果支持)，是否继续？",
      QMessageBox::Yes | QMessageBox::No);
  if (ret != QMessageBox::Yes) return;
  Run(false);
}

void AutoBindDialog::Run(bool dryRun) {
  if (!display_) return;
  geditor::Framework::AutoBindDiagnostic diag;
  double distTol = spin_dist_tol_->value();
  double coverage = spin_coverage_->value();
  int n = display_->OnAutoBindTrajectory(distTol, coverage, dryRun, &diag);

  QString summary = QString(
                        "中心线: %1 (未绑定 %2)   参考轨迹: %3 "
                        "(TrajectoryLayer=%4, BoundaryLayer=%5, "
                        "SegmentLayer=%6)   %7: %8")
                        .arg(diag.totalLanes)
                        .arg(diag.unboundLanes)
                        .arg(diag.refTotal)
                        .arg(diag.refFromTrajectoryLayer)
                        .arg(diag.refFromBoundaryLayer)
                        .arg(diag.refFromSegmentLayer)
                        .arg(dryRun ? "可绑定" : "已绑定")
                        .arg(n);
  lbl_summary_->setText(summary);
  txt_detail_->setPlainText(QString::fromStdString(diag.detail));
}
