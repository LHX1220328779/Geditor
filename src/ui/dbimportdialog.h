#ifndef DBIMPORTDIALOG_H
#define DBIMPORTDIALOG_H

#include <pcd/db_read_write.h>
#include <QDialog>

#include "algorithm/common.h"
#include "pcd/read_pcl.h"

namespace Ui {
class DBImportDialog;
}

class DBImportDialog : public QDialog {
  Q_OBJECT

 public:
  explicit DBImportDialog(QWidget *parent = 0);

  ~DBImportDialog();

 private slots:

  void on_set_db_path_btn_clicked();

  void on_set_pdb_path_btn_clicked();

  void on_import_btn_clicked();

 private:
  /// 切分线程
  void split_task();

  bool merge_db2pcd2(const std::string &db_file_name,
                     geditor::PointCloud<geditor::PCLPoint> &out);
  bool merge_db2pcd(const std::string &db_file_name,
                    geditor::PointCloud<geditor::PCLPoint> &out);

  void merge_point_cloud(geditor::DBReadWrite &dbrw,
                         const std::vector<UniqueType> &array_id,
                         geditor::PointCloud<geditor::PCLPoint> &merged_cloud);

  bool parse_pointcloud_data(const char *pBuffer, int length, int version,
                             geditor::PointCloud<geditor::PCLPoint> &cloud);

  Ui::DBImportDialog *ui;

  geditor::V3d origin_;
  int zone_ = 0;
  std::string db_path_;
  std::string pdb_path_;
};

#endif  // DBIMPORTDIALOG_H
