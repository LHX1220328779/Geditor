#pragma once

#include <QDialog>

class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QTextEdit;
class QRadioButton;
class DisplayWidget;

// 自适应边界生成对话框。
// 所有 "车道宽" = 中心线到边界线的距离（车道半宽）。
// 参数在进程生命周期内通过静态缓存保留，UI 重启时才重置为初始值。
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

  QDoubleSpinBox *spin_min_lane_dual_;
  QDoubleSpinBox *spin_max_lane_dual_;
  QDoubleSpinBox *spin_max_lane_single_;
  QDoubleSpinBox *spin_gap_;
  QDoubleSpinBox *spin_taper_;

  QRadioButton *radio_global_;
  QRadioButton *radio_local_;

  QPushButton *btn_run_;
  QPushButton *btn_close_;
  QLabel *lbl_summary_;
  QTextEdit *txt_detail_;

  // 进程级缓存：跨对话框实例保留用户调过的参数
  static double s_min_lane_dual;
  static double s_max_lane_dual;
  static double s_max_lane_single;
  static double s_gap;
  static double s_taper;
  static bool s_local_mode;
};
