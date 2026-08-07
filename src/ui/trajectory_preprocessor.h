#pragma once

#include <QString>
#include <QStringList>

#include "map/mine_origin_config.h"

struct TrajectoryPreprocessResult {
  QString mapping_file;
  QStringList trajectory_files;
  int reused_count = 0;
  int created_count = 0;
  QString warning;
};

struct SegmentMapConversionResult {
  QString output_directory;
  QStringList gps_files;
  qint64 point_count = 0;
};

class TrajectoryPreprocessor {
 public:
  // Converts every CSV in input_directory into an importable trajectory text
  // file and writes output_directory/mapping.txt. old_mapping_file is optional.
  static bool Process(const QString &input_directory,
                      const QString &output_directory,
                      const QString &old_mapping_file,
                      TrajectoryPreprocessResult &result, QString &error);

  // Converts local ENU segment_map CSV files (x,y,heading,v,gear,kk) to
  // WGS84 GPS CSV files (x,y,heading,v), where x=longitude and y=latitude.
  static bool ConvertSegmentMapToGps(
      const QString &input_directory, const QString &output_directory,
      const geditor::MineOrigin &origin, SegmentMapConversionResult &result,
      QString &error);
};
