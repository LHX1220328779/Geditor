#pragma once

#include <QDialog>

class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QTextEdit;
class DisplayWidget;

// 自适应边界生成对话框（R3+R4）：
// - 最低车道宽：双开均匀碾压阈值
// - 建议车道宽：单开均匀碾压宽度
// - 边界间距：左右边界与理论宽度的收缩间距
// - taper 长度：段间平滑过渡长度
class AdaptiveBoundaryDialog : public QDialog {
  Q_OBJECT

 public:
  explicit AdaptiveBoundaryDialog(DisplayWidget *display,
                                  QWidget *parent = nullptr);
  ~AdaptiveBoundaryDialog() override = default;

 private slots:
  void OnRun();

 private:
  DisplayWidget *display_;
  QDoubleSpinBox *spin_min_lane_;
  QDoubleSpinBox *spin_max_lane_;
  QDoubleSpinBox *spin_rec_lane_;
  QDoubleSpinBox *spin_gap_;
  QDoubleSpinBox *spin_taper_;
  QPushButton *btn_run_;
  QPushButton *btn_close_;
  QLabel *lbl_summary_;
  QTextEdit *txt_detail_;
};
