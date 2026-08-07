#include "trajectory_preprocessor.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QSaveFile>
#include <QSet>
#include <QTextStream>

#include <algorithm>
#include <cmath>

#include "map/projection_utm.h"

namespace {

QStringList ParseCsvLine(const QString &line, bool *ok = nullptr) {
  QStringList fields;
  QString field;
  bool quoted = false;
  for (int i = 0; i < line.size(); ++i) {
    const QChar ch = line.at(i);
    if (ch == '"') {
      if (quoted && i + 1 < line.size() && line.at(i + 1) == '"') {
        field += '"';
        ++i;
      } else {
        quoted = !quoted;
      }
    } else if (ch == ',' && !quoted) {
      fields.push_back(field.trimmed());
      field.clear();
    } else {
      field += ch;
    }
  }
  fields.push_back(field.trimmed());
  if (ok) *ok = !quoted;
  return fields;
}

QString CsvEscape(const QString &value) {
  if (!value.contains(',') && !value.contains('"') &&
      !value.contains('\n') && !value.contains('\r')) {
    return value;
  }
  QString escaped = value;
  escaped.replace('"', "\"\"");
  return '"' + escaped + '"';
}

bool ReadOldMapping(const QString &path, QMap<QString, int> &mapping,
                    int &max_index, QString &warning) {
  mapping.clear();
  max_index = 0;
  if (path.trimmed().isEmpty()) return true;

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    warning = QString("无法读取旧 mapping.txt：%1；将创建全新的映射。")
                  .arg(path);
    return true;
  }

  QMap<QString, int> parsed;
  int parsed_max = 0;
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  bool first_line = true;
  int line_number = 0;
  while (!stream.atEnd()) {
    const QString line = stream.readLine();
    ++line_number;
    if (first_line) {
      first_line = false;
      continue;
    }
    if (line.trimmed().isEmpty()) continue;
    bool csv_ok = false;
    const QStringList fields = ParseCsvLine(line, &csv_ok);
    bool index_ok = false;
    const int index = fields.size() >= 2 ? fields.at(1).toInt(&index_ok) : 0;
    if (!csv_ok || !index_ok || index <= 0 || fields.at(0).isEmpty()) {
      warning = QString("旧 mapping.txt 第 %1 行无效；将忽略旧文件并创建全新的映射。")
                    .arg(line_number);
      mapping.clear();
      max_index = 0;
      return true;
    }
    parsed[fields.at(0)] = index;
    parsed_max = std::max(parsed_max, index);
  }
  mapping = parsed;
  max_index = parsed_max;
  return true;
}

bool ReadCsv(const QString &path, QStringList &header,
             QList<QStringList> &rows, QString &error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    error = QString("无法读取生产路线文件：%1").arg(path);
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  if (stream.atEnd()) {
    error = QString("生产路线文件为空：%1").arg(path);
    return false;
  }

  bool csv_ok = false;
  header = ParseCsvLine(stream.readLine(), &csv_ok);
  if (!csv_ok || header.size() < 2) {
    error = QString("生产路线文件表头无效：%1").arg(path);
    return false;
  }

  int line_number = 1;
  while (!stream.atEnd()) {
    const QString line = stream.readLine();
    ++line_number;
    if (line.trimmed().isEmpty()) continue;
    QStringList fields = ParseCsvLine(line, &csv_ok);
    if (!csv_ok || fields.size() < 2) {
      error = QString("%1 第 %2 行格式无效").arg(path).arg(line_number);
      return false;
    }
    bool lon_ok = false;
    bool lat_ok = false;
    const double longitude = fields.at(0).toDouble(&lon_ok);
    const double latitude = fields.at(1).toDouble(&lat_ok);
    if (!lon_ok || !lat_ok || !std::isfinite(longitude) ||
        !std::isfinite(latitude) || longitude < -180.0 ||
        longitude > 180.0 || latitude < -80.0 || latitude > 84.0) {
      error = QString("%1 第 %2 行经纬度无效").arg(path).arg(line_number);
      return false;
    }
    rows.push_back(fields);
  }
  return true;
}

bool WriteTrajectory(const QString &input_file, const QString &output_file,
                     int index, QString &error) {
  QStringList header;
  QList<QStringList> rows;
  if (!ReadCsv(input_file, header, rows, error)) return false;

  QSaveFile file(output_file);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = QString("无法写入轨迹文件：%1").arg(output_file);
    return false;
  }
  QTextStream stream(&file);
  stream.setCodec("UTF-8");
  stream << "index " << header.join(' ') << '\n';

  QList<int> selected_rows;
  if (rows.size() <= 2) {
    for (int i = 0; i < rows.size(); ++i) selected_rows.push_back(i);
  } else {
    selected_rows.push_back(0);
    for (int i = 1; i < rows.size() - 1; i += 5) selected_rows.push_back(i);
    selected_rows.push_back(rows.size() - 1);
  }

  geditor::ProjectionUTM projection;
  for (int row_index : selected_rows) {
    const QStringList &row = rows.at(row_index);
    const double longitude = row.at(0).toDouble();
    const double latitude = row.at(1).toDouble();
    geditor::UTMPoint utm;
    projection.LatLonToCartesian(latitude, longitude, utm);
    stream << index << ' ' << QString::number(utm.x, 'g', 15) << ' '
           << QString::number(utm.y, 'g', 15);
    for (int column = 2; column < row.size(); ++column) {
      stream << ' ' << row.at(column);
    }
    stream << '\n';
  }
  if (!file.commit()) {
    error = QString("保存轨迹文件失败：%1").arg(output_file);
    return false;
  }
  return true;
}

QString NormalizedHeader(QString value) {
  if (!value.isEmpty() && value.at(0) == QChar(0xfeff)) value.remove(0, 1);
  return value.trimmed().toLower();
}

QString CoordinateText(double value) {
  QString text = QString::number(value, 'f', 6);
  while (text.contains('.') && text.endsWith('0')) text.chop(1);
  if (text.endsWith('.')) text.chop(1);
  return text;
}

bool ConvertSegmentMapFile(const QString &input_file,
                           const QString &output_file,
                           const geditor::MineOrigin &origin,
                           qint64 &point_count, QString &error) {
  QFile input(input_file);
  if (!input.open(QIODevice::ReadOnly | QIODevice::Text)) {
    error = QString("无法读取 segment_map 文件：%1").arg(input_file);
    return false;
  }
  QTextStream input_stream(&input);
  input_stream.setCodec("UTF-8");
  if (input_stream.atEnd()) {
    error = QString("segment_map 文件为空：%1").arg(input_file);
    return false;
  }

  bool csv_ok = false;
  const QStringList raw_header = ParseCsvLine(input_stream.readLine(), &csv_ok);
  QStringList header;
  for (const QString &field : raw_header) header.push_back(NormalizedHeader(field));
  const QStringList required = {"x", "y", "heading", "v"};
  if (!csv_ok || header.size() < required.size()) {
    error = QString("CSV 表头无效：%1").arg(input_file);
    return false;
  }
  for (int i = 0; i < required.size(); ++i) {
    if (header.at(i) != required.at(i)) {
      error = QString("%1 表头必须以 x,y,heading,v 开头，当前第 %2 列为 %3")
                  .arg(input_file)
                  .arg(i + 1)
                  .arg(raw_header.value(i));
      return false;
    }
  }

  QSaveFile output(output_file);
  if (!output.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = QString("无法写入 GPS CSV：%1").arg(output_file);
    return false;
  }
  QTextStream output_stream(&output);
  output_stream.setCodec("UTF-8");
  output_stream << "x,y,heading,v\n";

  geditor::ProjectionUTM projection;
  qint64 file_point_count = 0;
  int line_number = 1;
  while (!input_stream.atEnd()) {
    const QString line = input_stream.readLine();
    ++line_number;
    if (line.trimmed().isEmpty()) continue;
    const QStringList fields = ParseCsvLine(line, &csv_ok);
    if (!csv_ok || fields.size() < 4) {
      error = QString("%1 第 %2 行不是有效的 CSV 数据")
                  .arg(input_file)
                  .arg(line_number);
      return false;
    }
    bool east_ok = false;
    bool north_ok = false;
    bool heading_ok = false;
    bool velocity_ok = false;
    const double east = fields.at(0).toDouble(&east_ok);
    const double north = fields.at(1).toDouble(&north_ok);
    const double heading = fields.at(2).toDouble(&heading_ok);
    const double velocity = fields.at(3).toDouble(&velocity_ok);
    if (!east_ok || !north_ok || !heading_ok || !velocity_ok ||
        !std::isfinite(east) || !std::isfinite(north) ||
        !std::isfinite(heading) || !std::isfinite(velocity)) {
      error = QString("%1 第 %2 行包含无效数值")
                  .arg(input_file)
                  .arg(line_number);
      return false;
    }

    geditor::GPSPoint gps;
    projection.LocalENUToGPS(east, north, 0.0, origin.latitude,
                             origin.longitude, origin.z, gps);
    output_stream << CoordinateText(gps.latlon.lon) << ','
                  << CoordinateText(gps.latlon.lat) << ','
                  << fields.at(2).trimmed() << ','
                  << fields.at(3).trimmed() << '\n';
    ++file_point_count;
  }
  if (file_point_count == 0) {
    error = QString("segment_map 文件没有数据行：%1").arg(input_file);
    return false;
  }
  if (!output.commit()) {
    error = QString("保存 GPS CSV 失败：%1").arg(output_file);
    return false;
  }
  point_count += file_point_count;
  return true;
}

}  // namespace

bool TrajectoryPreprocessor::Process(
    const QString &input_directory, const QString &output_directory,
    const QString &old_mapping_file, TrajectoryPreprocessResult &result,
    QString &error) {
  result = TrajectoryPreprocessResult();
  error.clear();

  QDir input_dir(input_directory);
  if (input_directory.trimmed().isEmpty() || !input_dir.exists()) {
    error = "请选择有效的生产路线包目录";
    return false;
  }
  if (output_directory.trimmed().isEmpty()) {
    error = "请选择 mapping.txt 输出路径";
    return false;
  }
  if (!QDir().mkpath(output_directory)) {
    error = QString("无法创建输出目录：%1").arg(output_directory);
    return false;
  }
  QDir output_dir(output_directory);

  QMap<QString, int> old_mapping;
  int max_index = 0;
  ReadOldMapping(old_mapping_file, old_mapping, max_index, result.warning);

  const QFileInfoList csv_files = input_dir.entryInfoList(
      QStringList() << "*.csv", QDir::Files | QDir::Readable, QDir::Name);
  QSet<QString> output_names;
  struct MappingRecord {
    QString name;
    int index;
  };
  QList<MappingRecord> records;

  for (const QFileInfo &csv : csv_files) {
    QString route_name = csv.completeBaseName();
    if (route_name.size() > 4) route_name.chop(4);
    if (route_name.isEmpty()) {
      error = QString("无法从文件名生成路线名称：%1").arg(csv.fileName());
      return false;
    }
    if (output_names.contains(route_name)) {
      error = QString("多个CSV生成了相同的路线名称：%1").arg(route_name);
      return false;
    }
    output_names.insert(route_name);

    int index = old_mapping.value(route_name, 0);
    if (index > 0) {
      ++result.reused_count;
    } else {
      index = ++max_index;
      ++result.created_count;
    }
    const QString output_file = output_dir.filePath(route_name + ".txt");
    if (!WriteTrajectory(csv.absoluteFilePath(), output_file, index, error)) {
      return false;
    }
    result.trajectory_files.push_back(QDir::cleanPath(output_file));
    records.push_back({route_name, index});
  }

  result.mapping_file = output_dir.filePath("mapping.txt");
  QSaveFile mapping_file(result.mapping_file);
  if (!mapping_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = QString("无法写入 mapping.txt：%1").arg(result.mapping_file);
    return false;
  }
  QTextStream mapping_stream(&mapping_file);
  mapping_stream.setCodec("UTF-8");
  mapping_stream << "filename,index\n";
  for (const MappingRecord &record : records) {
    mapping_stream << CsvEscape(record.name) << ',' << record.index << '\n';
  }
  if (!mapping_file.commit()) {
    error = QString("保存 mapping.txt 失败：%1").arg(result.mapping_file);
    return false;
  }

  if (csv_files.isEmpty()) {
    result.warning = result.warning.isEmpty()
                         ? "生产路线包中没有找到CSV文件，已生成空的 mapping.txt。"
                         : result.warning + "\n生产路线包中没有找到CSV文件，已生成空的 mapping.txt。";
  }
  return true;
}

bool TrajectoryPreprocessor::ConvertSegmentMapToGps(
    const QString &input_directory, const QString &output_directory,
    const geditor::MineOrigin &origin, SegmentMapConversionResult &result,
    QString &error) {
  result = SegmentMapConversionResult();
  error.clear();

  QDir input_dir(input_directory);
  if (input_directory.trimmed().isEmpty() || !input_dir.exists()) {
    error = "请选择有效的 segment_map 目录";
    return false;
  }
  const QFileInfoList csv_files = input_dir.entryInfoList(
      QStringList() << "*.csv", QDir::Files | QDir::Readable, QDir::Name);
  if (csv_files.isEmpty()) {
    error = QString("目录中没有 CSV 文件：%1").arg(input_directory);
    return false;
  }
  if (output_directory.trimmed().isEmpty()) {
    error = "请选择 gps_full 输出目录";
    return false;
  }
  if (QDir::cleanPath(input_directory) == QDir::cleanPath(output_directory)) {
    error = "输出目录不能与 segment_map 输入目录相同";
    return false;
  }
  if (!QDir().mkpath(output_directory)) {
    error = QString("无法创建输出目录：%1").arg(output_directory);
    return false;
  }

  QDir output_dir(output_directory);
  result.output_directory = QDir::cleanPath(output_dir.absolutePath());
  for (const QFileInfo &csv : csv_files) {
    const QString output_file =
        output_dir.filePath(csv.completeBaseName() + "_gps.csv");
    if (!ConvertSegmentMapFile(csv.absoluteFilePath(), output_file, origin,
                               result.point_count, error)) {
      return false;
    }
    result.gps_files.push_back(QDir::cleanPath(output_file));
  }
  return true;
}
