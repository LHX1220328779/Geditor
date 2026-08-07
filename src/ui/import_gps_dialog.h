#ifndef IMPORT_GPS_DIALOG_H
#define IMPORT_GPS_DIALOG_H

#include <QDialog>
#include <QStandardItemModel>

#include "core/point3d.h"
#include "map/mine_origin_config.h"

namespace Ui {
class ImportGPSDialog;
}

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTabWidget;

class ImportGPSDialog : public QDialog {
  Q_OBJECT

 public:
  explicit ImportGPSDialog(QWidget *parent = 0);
  // 0: force WGS84 GPS, 1: force local XY. Kept for existing callers.
  void SetFrame(int t);
  void SetMineOrigin(const geditor::MineOrigin &origin);

  ~ImportGPSDialog();

  std::vector<geditor::CurbsTrack *> GetSegment() const {
    return m_segmentArray;
  }

  int GetImportType() const { return import_type_; }

 private slots:

  void on_add_btn_clicked();

  void on_delete_btn_clicked();

  void on_clear_btn_clicked();

  void on_import_btn_clicked();

  void on_radioButton_clicked();

  void on_radioButton_2_clicked();

  void on_radioButton_3_clicked();

 private:
  bool ParsePointSet(const QString &filename,
                     std::vector<geditor::TrackPoint> &pointSet);

  void AddTrajectoryFiles(const QStringList &files);
  void AddTrajectoryDirectory();
  bool GenerateMapping(bool show_success);
  bool ConvertSegmentMap(bool show_success);

 private:
  Ui::ImportGPSDialog *ui;

  std::vector<geditor::CurbsTrack *> m_segmentArray;
  int import_type_ = 2;
  QStandardItemModel *model_ = nullptr;
  int frame_type_ = -1;

  QTabWidget *workflow_tabs_ = nullptr;
  QComboBox *coordinate_mode_combo_ = nullptr;
  QLineEdit *production_route_edit_ = nullptr;
  QLineEdit *mapping_output_edit_ = nullptr;
  QLineEdit *old_mapping_edit_ = nullptr;
  QLabel *preprocess_status_ = nullptr;
  QLineEdit *segment_map_edit_ = nullptr;
  QLineEdit *gps_output_edit_ = nullptr;
  QLabel *origin_display_ = nullptr;
  QLabel *conversion_status_ = nullptr;
  QCheckBox *add_converted_check_ = nullptr;
  QPushButton *convert_segment_button_ = nullptr;
  geditor::MineOrigin mine_origin_;
  bool has_mine_origin_ = false;
};

#endif  // IMPORT_GPS_DIALOG_H
