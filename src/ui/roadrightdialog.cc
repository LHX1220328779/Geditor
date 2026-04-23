#include "roadrightdialog.h"

#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>

#include "ui/display_widget.h"

RoadRightDialog::RoadRightDialog(DisplayWidget *display, QWidget *parent)
    : QDialog(parent), display_(display)
{
    setWindowTitle("road_right 自动判定 - 阈值实时调整");
    resize(720, 520);

    spin_thr_ = new QDoubleSpinBox(this);
    spin_thr_->setRange(0.1, 100.0);
    spin_thr_->setSingleStep(0.1);
    spin_thr_->setDecimals(2);
    spin_thr_->setValue(5.8);
    spin_thr_->setSuffix(" 米");
    spin_thr_->setToolTip(
        "阈值 A：整条路段上每个中心线点到左、右边界线的最小距离\n"
        "若处处都 > 阈值 A ，则 road_right=1 (均匀碾压)。否则 road_right=0。");

    auto *form = new QFormLayout;
    form->addRow("阈值 A", spin_thr_);

    lbl_summary_ = new QLabel("尚未运行", this);
    lbl_summary_->setWordWrap(true);

    txt_detail_ = new QTextEdit(this);
    txt_detail_->setReadOnly(true);
    txt_detail_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    txt_detail_->setPlaceholderText(
        "点击 [立即校验并写入] 将按当前阈值判定所有中心线的 road_right，\n"
        "判定结果会直接写入属性面板，可在属性窗口内再手动修改。");

    btn_run_ = new QPushButton("立即校验并写入", this);
    btn_run_->setStyleSheet("font-weight:bold;");
    btn_close_ = new QPushButton("关闭", this);

    auto *btns = new QHBoxLayout;
    btns->addStretch();
    btns->addWidget(btn_run_);
    btns->addWidget(btn_close_);

    auto *root = new QVBoxLayout(this);
    root->addLayout(form);
    root->addWidget(lbl_summary_);
    root->addWidget(txt_detail_, 1);
    root->addLayout(btns);

    connect(btn_run_, &QPushButton::clicked, this, &RoadRightDialog::OnRun);
    connect(btn_close_, &QPushButton::clicked, this, &QDialog::accept);
    connect(spin_thr_,
            static_cast<void (QDoubleSpinBox::*)(double)>(
                &QDoubleSpinBox::valueChanged),
            this, &RoadRightDialog::OnThresholdChanged);
}

void RoadRightDialog::OnThresholdChanged(double v)
{
    lbl_summary_->setText(
        QString("当前阈值 = %1 m （点击 [立即校验并写入] 执行）").arg(v));
}

void RoadRightDialog::OnRun()
{
    if (!display_)
        return;
    geditor::Framework::RoadRightDiagnostic diag;
    double thr = spin_thr_->value();
    int n = display_->OnAutoCheckRoadRight(thr, &diag);

    QString summary = QString(
                          "总中心线=%1  均匀碾压(1)=%2  均匀碾压(0)=%3  "
                          "缺少边界=%4  本次更新=%5  (阈值 A=%6 m)")
                          .arg(diag.totalLanes)
                          .arg(diag.compliant)
                          .arg(diag.nonCompliant)
                          .arg(diag.missingBoundary)
                          .arg(n)
                          .arg(thr);
    lbl_summary_->setText(summary);
    txt_detail_->setPlainText(QString::fromStdString(diag.detail));
}
