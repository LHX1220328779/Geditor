#pragma once

#include <QDialog>
#include <QString>

#include <vector>

#include "map/mine_origin_config.h"

class QComboBox;
class QLabel;

class MineOriginDialog : public QDialog {
 public:
  explicit MineOriginDialog(QWidget *parent = nullptr);

  bool IsConfigValid() const { return config_valid_; }
  geditor::MineOrigin SelectedOrigin() const;

 private:
  void UpdateDetails(int index);

  QComboBox *combo_ = nullptr;
  QLabel *details_ = nullptr;
  std::vector<geditor::MineOrigin> origins_;
  QString config_path_;
  bool config_valid_ = false;
};
