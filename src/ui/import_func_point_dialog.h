#ifndef IMPORT_FUNC_POINT_DIALOG_H
#define IMPORT_FUNC_POINT_DIALOG_H

#include <QDialog>
#include <QStandardItemModel>

#include "core/bound_segment.h"
#include "core/job_area.h"

namespace Ui {
class ImportFuncPointDialog;
}

class ImportFuncPointDialog : public QDialog {
  Q_OBJECT

 public:
  explicit ImportFuncPointDialog(QWidget *parent = 0);

  ~ImportFuncPointDialog();

  std::vector<geditor::BoundSegment *> GetBoundary() { return m_boundArray; }

  std::vector<geditor::JobArea *> GetJobArea() { return m_jobArray; }

 private slots:

  void on_import_btn_clicked();

  void on_cancel_btn_clicked();

  void on_toolButton_clicked();

 private:
  bool ReadDBBoundary(const char *filename,
                      std::vector<geditor::BoundSegment *> &segArray);

  bool ReadDBFunctionPoint(const char *filename,
                           std::vector<geditor::JobArea *> &jobArray);

  bool ParseFunctionPoint(const char *filename,
                          std::vector<geditor::JobArea *> &jobArray);

  void ShowData();

 private:
  Ui::ImportFuncPointDialog *ui;
  std::vector<geditor::JobArea *> m_jobArray;
  std::vector<geditor::BoundSegment *> m_boundArray;
  QStandardItemModel *model_ = nullptr;
};

#endif  // IMPORT_FUNC_POINT_DIALOG_H
