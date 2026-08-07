#include <QApplication>
#include <QCoreApplication>
#include <QDir>

#include <algorithm>
#include <iostream>
#include <vector>

#include "core/log.h"
#include "geditor_mainwindow.h"
#include "map/mine_origin_config.h"
#include "trajectory_preprocessor.h"

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 0;
  QApplication a(argc, argv);
  QApplication::setStyle("Fusion");

  if ((argc == 4 || argc == 5) &&
      std::string(argv[1]) == "--convert-segment-map") {
    const QDir executable_dir(QCoreApplication::applicationDirPath());
    const QString config_path =
        executable_dir.absoluteFilePath("../mine_origins.yaml");
    std::vector<geditor::MineOrigin> origins;
    std::string config_error;
    if (!geditor::MineOriginConfig::Load(config_path.toStdString(), origins,
                                         &config_error)) {
      std::cerr << "Cannot load mine origin: " << config_error << std::endl;
      return 2;
    }
    if (argc == 5) {
      const std::string requested_origin = argv[4];
      const auto selected =
          std::find_if(origins.begin(), origins.end(),
                       [&](const geditor::MineOrigin &origin) {
                         return origin.name == requested_origin;
                       });
      if (selected == origins.end()) {
        std::cerr << "Unknown mine origin: " << requested_origin << std::endl;
        return 2;
      }
      std::rotate(origins.begin(), selected, selected + 1);
    }
    SegmentMapConversionResult result;
    QString conversion_error;
    if (!TrajectoryPreprocessor::ConvertSegmentMapToGps(
            QString::fromLocal8Bit(argv[2]), QString::fromLocal8Bit(argv[3]),
            origins.front(), result, conversion_error)) {
      std::cerr << conversion_error.toStdString() << std::endl;
      return 3;
    }
    std::cout << "Converted " << result.gps_files.size() << " files and "
              << result.point_count << " points to "
              << result.output_directory.toStdString() << " using origin "
              << origins.front().name << std::endl;
    return 0;
  }

  GeditorMainWindow w;
  w.show();

  return a.exec();
}
