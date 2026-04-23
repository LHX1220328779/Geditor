#pragma once

#include <QDialog>

class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QTextEdit;
class DisplayWidget;

// 阈值 A 实时可调的 road_right 自动判定对话框。
class RoadRightDialog : public QDialog {
  Q_OBJECT

 public:
  explicit RoadRightDialog(DisplayWidget *display, QWidget *parent = nullptr);
  ~RoadRightDialog() override = default;

 private slots:
  void OnRun();
  void OnThresholdChanged(double v);

 private:
  DisplayWidget *display_;
  QDoubleSpinBox *spin_thr_;
  QPushButton *btn_run_;
  QPushButton *btn_close_;
  QLabel *lbl_summary_;
  QTextEdit *txt_detail_;
};
