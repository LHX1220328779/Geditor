#pragma once

#include <QDialog>

class QDoubleSpinBox;
class QPushButton;
class QTextEdit;
class QLabel;
class DisplayWidget;

class AutoBindDialog : public QDialog {
  Q_OBJECT

 public:
  explicit AutoBindDialog(DisplayWidget *display, QWidget *parent = nullptr);
  ~AutoBindDialog() override = default;

 private slots:
  void OnPreview();
  void OnApply();

 private:
  void Run(bool dryRun);

  DisplayWidget *display_;
  QDoubleSpinBox *spin_dist_tol_;
  QDoubleSpinBox *spin_coverage_;
  QPushButton *btn_preview_;
  QPushButton *btn_apply_;
  QPushButton *btn_close_;
  QLabel *lbl_summary_;
  QTextEdit *txt_detail_;
};
