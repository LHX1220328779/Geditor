#include "mineorigindialog.h"

#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QLabel>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

namespace {

QString ConfigPath() {
  const QDir executable_dir(QCoreApplication::applicationDirPath());
  return QDir::cleanPath(
      executable_dir.absoluteFilePath("../mine_origins.yaml"));
}

}  // namespace

MineOriginDialog::MineOriginDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle("矿山原点配置");
  setMinimumWidth(620);

  auto *layout = new QVBoxLayout(this);
  layout->addWidget(
      new QLabel("请选择本次计算和导出统一使用的矿山原点：", this));
  combo_ = new QComboBox(this);
  combo_->setObjectName("mineOriginCombo");
  layout->addWidget(combo_);
  details_ = new QLabel(this);
  details_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  details_->setWordWrap(true);
  layout->addWidget(details_);

  auto *button_layout = new QHBoxLayout();
  auto *cancel_button = new QPushButton("取消", this);
  cancel_button->setObjectName("cancelMineOriginButton");
  auto *confirm_button = new QPushButton("确认并应用", this);
  confirm_button->setObjectName("confirmMineOriginButton");
  confirm_button->setDefault(true);
  confirm_button->setAutoDefault(true);
  button_layout->addStretch();
  button_layout->addWidget(cancel_button);
  button_layout->addWidget(confirm_button);
  button_layout->setDirection(QBoxLayout::LeftToRight);
  layout->addLayout(button_layout);
  connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
  connect(confirm_button, &QPushButton::clicked, this, [this]() {
    const auto origin = SelectedOrigin();
    std::string save_error;
    if (!geditor::MineOriginConfig::SaveCurrentOrigin(
            config_path_.toStdString(), origin.name, &save_error)) {
      QMessageBox::critical(
          this, "无法保存当前原点",
          QString("原点尚未应用。请确认配置文件可写：\n%1\n\n%2")
              .arg(config_path_, QString::fromStdString(save_error)));
      return;
    }
    accept();
  });
  connect(combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int index) { UpdateDetails(index); });

  config_path_ = ConfigPath();
  std::string error;
  config_valid_ = geditor::MineOriginConfig::Load(
      config_path_.toStdString(), origins_, &error);
  if (!config_valid_) {
    confirm_button->setEnabled(false);
    details_->setText(QString::fromStdString(error));
    return;
  }

  for (const auto &origin : origins_) {
    combo_->addItem(
        QString("%1  |  GLOBAL_ORIGIN_LAT=%2, GLOBAL_ORIGIN_LON=%3, zone=%4")
            .arg(QString::fromStdString(origin.name))
            .arg(origin.latitude, 0, 'f', 9)
            .arg(origin.longitude, 0, 'f', 9)
            .arg(origin.zone));
  }
  UpdateDetails(0);
}

geditor::MineOrigin MineOriginDialog::SelectedOrigin() const {
  const int index = combo_->currentIndex();
  if (index < 0 || index >= static_cast<int>(origins_.size())) return {};
  return origins_[index];
}

void MineOriginDialog::UpdateDetails(int index) {
  if (index < 0 || index >= static_cast<int>(origins_.size())) return;
  const auto &origin = origins_[index];
  details_->setText(
      QString("矿山：%1\nGLOBAL_ORIGIN_LAT=%2\nGLOBAL_ORIGIN_LON=%3\n"
              "GLOBAL_ORIGIN_ALT=%4\nUTM zone=%5\n配置文件：%6")
          .arg(QString::fromStdString(origin.name))
          .arg(origin.latitude, 0, 'f', 9)
          .arg(origin.longitude, 0, 'f', 9)
          .arg(origin.z, 0, 'f', 3)
          .arg(origin.zone)
          .arg(config_path_));
}
