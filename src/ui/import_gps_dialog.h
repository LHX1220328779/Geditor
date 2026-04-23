#ifndef IMPORT_GPS_DIALOG_H
#define IMPORT_GPS_DIALOG_H

#include <QDialog>
#include <QStandardItemModel>

#include "core/point3d.h"

namespace Ui {
class ImportGPSDialog;
}

class ImportGPSDialog : public QDialog {
  Q_OBJECT

 public:
  explicit ImportGPSDialog(QWidget *parent = 0);
  //0 gps->utm 1 xyz 
  void SetFrame(int t) { frame_type_ = t; }

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

 private:
  Ui::ImportGPSDialog *ui;

  std::vector<geditor::CurbsTrack *> m_segmentArray;
  int import_type_ = 2;
  QStandardItemModel *model_ = nullptr;
  int frame_type_ = 1;
};

#endif  // IMPORT_GPS_DIALOG_H
