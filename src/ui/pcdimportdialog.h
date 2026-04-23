#ifndef PCDImportDialog_H
#define PCDImportDialog_H

#include <pcd/db_read_write.h>

#include <QDialog>

#include "algorithm/common.h"
#include "pcd/read_pcl.h"

namespace Ui {
class PCDImportDialog;
}

class PCDImportDialog : public QDialog {
  Q_OBJECT

 public:
  explicit PCDImportDialog(QWidget *parent = 0);

  ~PCDImportDialog();

 private slots:

  void on_set_pcd_path_btn_clicked();

  void on_set_pdb_path_btn_clicked();

  void on_import_btn_clicked();

 private:
  /// 切分线程
  void split_task();

  bool merge_pcd2pcd(const std::string &db_file_name,
                     geditor::PointCloud<geditor::PCLPoint> &out);

  Ui::PCDImportDialog *ui;

  geditor::V3d origin_;
  int zone_ = 0;
  std::string pcd_path_;
  std::string pdb_path_;
  std::map<std::string, geditor::V3d> tra_;
};

#endif  // PCDImportDialog_H
