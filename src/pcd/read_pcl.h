
#pragma once

#include <fstream>

#include "algorithm/common.h"
#include "pcd/async_task.h"
#include "pcd/memory_file.h"
#include "pcd/pointcloud.h"
#include "pcd/stdio_file.h"
#include "utils/string_ext.h"

namespace geditor {

enum PCLPointType {
  PCL_INT8 = 1,
  PCL_UINT8 = 2,
  PCL_INT16 = 3,
  PCL_UINT16 = 4,
  PCL_INT32 = 5,
  PCL_UINT32 = 6,
  PCL_FLOAT32 = 7,
  PCL_FLOAT64 = 8,
};

struct PCLField {
  std::string name;
  PCLPointType datatype;
  int offset;
  int count;
};

struct PCLPoint {
  float x;
  float y;
  float z;
  float w;
};

struct PCLPointLable {
  float x;
  float y;
  float z;
  float w;
  float h;
};

template <typename PointT>
class PCDReader {
 public:
  PCDReader();

  void SetListener(AsyncTask *task, int minVal, int maxVal);

  bool LoadMemery(const char *fileName);

  bool loadFile(const char *fileName);

  bool readHeader(File *fs, int &offset);

  int ReadAscii(File *fs, const int offset);

  int ReadBinary(File *fs, const int offset);

  void Destroy();

 public:
  PointCloud<PointT> cloud_;

 private:
  int getFieldType(const int size, char type);

  void OnProgress(int value);

 private:
  AsyncTask *task_progress_;
  int progress_value_;
  int min_progress_value_;
  int progress_range_;

  std::vector<PCLField> fields_;

  float version_;
  int points_;
  int width_;
  int height_;
  int data_type_;

  int point_step_;
  int row_step_;
};

//=============================================

template <typename Type, typename PCLPointT>
void copyStringValue(const std::string &st, PCLPointT &data, int field_offset) {
  Type value;
  if (String::IEquals(st, "nan")) {
    value = std::numeric_limits<Type>::quiet_NaN();
  } else {
    std::istringstream is(st);
    is.imbue(std::locale::classic());
    if (!(is >> value)) {
      value = static_cast<Type>(atof(st.c_str()));
    }
  }

  memcpy((char *)(&data) + field_offset, &value, sizeof(Type));
}

template <typename PointT>
void PCDReader<PointT>::SetListener(AsyncTask *task, int minVal, int maxVal) {
  task_progress_ = task;
  min_progress_value_ = minVal;
  progress_range_ = maxVal - minVal;
}

template <typename PointT>
void PCDReader<PointT>::OnProgress(int value) {
  if (task_progress_) {
    float fPrecent = (100.0f * (float)value / (float)points_);
    int nProgress =
        (int)(min_progress_value_ + fPrecent * progress_range_ / 100.0f);

    if (progress_value_ < nProgress) {
      progress_value_ = nProgress;
    }
  }
}

template <typename PointT>
int PCDReader<PointT>::getFieldType(const int size, char type) {
  type = std::toupper(type, std::locale::classic());
  switch (size) {
    case 1:
      if (type == 'I') return (PCLPointType::PCL_INT8);
      if (type == 'U') return (PCLPointType::PCL_UINT8);
      break;

    case 2:
      if (type == 'I') return (PCLPointType::PCL_INT16);
      if (type == 'U') return (PCLPointType::PCL_UINT16);
      break;

    case 4:
      if (type == 'I') return (PCLPointType::PCL_INT32);
      if (type == 'U') return (PCLPointType::PCL_UINT32);
      if (type == 'F') return (PCLPointType::PCL_FLOAT32);
      break;

    case 8:
      if (type == 'F') return (PCLPointType::PCL_FLOAT64);
      break;
  }
  return (-1);
}

template <typename PointT>
PCDReader<PointT>::PCDReader() : task_progress_(NULL) {}

template <typename PointT>
bool PCDReader<PointT>::LoadMemery(const char *fileName) {
  std::ifstream fs;

  fs.open(fileName, std::ios::binary);
  if (!fs.is_open() || fs.fail()) {
    fs.close();
    return false;
  }

  fs.seekg(0, fs.end);
  int length = (int)fs.tellg();
  fs.seekg(0, fs.beg);

  MemroyFile file;

  if (!file.Allocate(length)) {
    fs.close();
    return false;
  }

  if (!fs.read(file.GetBuffer(), file.GetLength())) {
    fs.close();
    return false;
  } else {
    fs.close();
  }

  //����
  int offset = 0;
  if (!readHeader(&file, offset)) {
    fs.close();
    return false;
  }

  if (data_type_ == 0) {
    int bRet = ReadAscii(&file, offset);
    return true;
  } else if (data_type_ == 1) {
    int bRet = ReadBinary(&file, offset);

    return true;
  }
  return false;
}

template <typename PointT>
bool PCDReader<PointT>::loadFile(const char *fileName) {
  StdioFile fs;
  if (fs.open(fileName)) {
    int offset = 0;
    if (!readHeader(&fs, offset)) {
      fs.close();
      return false;
    }

    if (data_type_ == 0) {
      int bRet = ReadAscii(&fs, offset);
    } else if (data_type_ == 1) {
      int bRet = ReadBinary(&fs, offset);
    }

    fs.close();
    return true;
  }
  return false;
}

template <typename PointT>
int PCDReader<PointT>::ReadBinary(File *fs, const int offset) {
  // m_data.resize(m_points);

  fs->seekg(offset, File::beg);

  for (int i = 0; i < points_; i++) {
    PointT data;
    if (fs->Read((char *)&data, sizeof(PointT))) {
      cloud_.PushBack(data);
    } else {
      return false;
    }
  }
  return true;
}

template <typename PointT>
int PCDReader<PointT>::ReadAscii(File *fs, const int offset) {
  char szLine[1024];

  std::vector<std::string> st;

  // m_data.resize(m_points);

  fs->seekg(offset, File::beg);

  int idx = 0;
  while (idx < points_ && !fs->eof()) {
    fs->ReadLine(szLine, 1024);

    String::Trim(szLine);
    if (szLine[0] == 0) {
      continue;
    }

    std::string line(szLine);
    String::Split(st, line, "\t\r ");

    size_t total = 0;
    PCLPoint pcpt;

    for (int d = 0; d < static_cast<int>(fields_.size()); ++d) {
      for (int c = 0; c < fields_[d].count; ++c) {
        switch (fields_[d].datatype) {
          case PCLPointType::PCL_INT32: {
            copyStringValue<float>(st.at(total + c), pcpt, fields_[d].offset);
            break;
          }
          case PCLPointType::PCL_UINT32: {
            copyStringValue<float>(st.at(total + c), pcpt, fields_[d].offset);
            break;
          }
          case PCLPointType::PCL_FLOAT32: {
            copyStringValue<float>(st.at(total + c), pcpt, fields_[d].offset);
            break;
          }
          default:
            LOG(ERROR) << "[pcl::PCDReader::read] Incorrect field data type "
                          "specified!";
            break;
        }
      }
      total += fields_[d].count;
    }

    cloud_.PushBack(pcpt);

    idx++;

    OnProgress(idx);
  }

  return true;
}

template <typename PointT>
bool PCDReader<PointT>::readHeader(File *fs, int &data_idx) {
  char szLine[1024];

  std::vector<std::string> st;

  std::vector<int> field_sizes;
  std::vector<int> field_counts;
  std::vector<char> field_types;

  data_idx = 0;

  while (!fs->eof()) {
    fs->ReadLine(szLine, 1024);

    String::Trim(szLine);
    if (szLine[0] == 0) {
      continue;
    }

    std::string line(szLine);
    std::stringstream sstream(line);
    sstream.imbue(std::locale::classic());

    std::string line_type;
    sstream >> line_type;

    String::Split(st, line, "\t\r ");

    if (line_type.substr(0, 1) == "#") {
      continue;
    }

    if (line_type.substr(0, 7) == "VERSION") {
      sstream >> version_;
      continue;
    }

    if (line_type.substr(0, 6) == "FIELDS") {
      int specified_channel_count = static_cast<int>(st.size() - 1);

      fields_.resize(specified_channel_count);
      for (int i = 0; i < specified_channel_count; ++i) {
        std::string col_type = st.at(i + 1);
        fields_[i].name = col_type;
      }

      int offset = 0;
      for (int i = 0; i < specified_channel_count; ++i, offset += 4) {
        fields_[i].offset = offset;
        fields_[i].datatype = PCLPointType::PCL_FLOAT32;
        fields_[i].count = 1;
      }
      point_step_ = offset;
      continue;
    }

    // field_counts;
    // field_types represents the type of data in a field (e.g., F = float, U =
    // unsigned)
    // std::vector<char> field_types;

    if (line_type.substr(0, 4) == "SIZE") {
      int specified_channel_count = static_cast<int>(st.size() - 1);

      if (specified_channel_count != static_cast<int>(fields_.size())) {
        return false;
      }

      field_sizes.resize(specified_channel_count);

      int offset = 0;
      for (int i = 0; i < specified_channel_count; ++i) {
        int col_type;
        sstream >> col_type;
        fields_[i].offset = offset;
        offset += col_type;
        field_sizes[i] = col_type;
      }
      point_step_ = offset;
      continue;
    }

    // Get the field types
    if (line_type.substr(0, 4) == "TYPE") {
      if (field_sizes.empty()) {
        return false;
      }

      int specified_channel_count = static_cast<int>(st.size() - 1);

      //// Allocate enough memory to accommodate all fields
      if (specified_channel_count != static_cast<int>(fields_.size())) {
        return false;
      }

      field_types.resize(specified_channel_count);

      for (int i = 0; i < specified_channel_count; ++i) {
        field_types[i] = st.at(i + 1).c_str()[0];
        fields_[i].datatype = static_cast<PCLPointType>(
            getFieldType(field_sizes[i], field_types[i]));
      }
      continue;
    }

    // Get the field counts
    if (line_type.substr(0, 5) == "COUNT") {
      if (field_sizes.empty() || field_types.empty()) {
        return false;
      }

      int specified_channel_count = static_cast<int>(st.size() - 1);

      if (specified_channel_count != static_cast<int>(fields_.size())) {
        return false;
      }

      field_counts.resize(specified_channel_count);

      int offset = 0;
      for (int i = 0; i < specified_channel_count; ++i) {
        fields_[i].offset = offset;
        int col_count;
        sstream >> col_count;
        fields_[i].count = col_count;
        offset += col_count * field_sizes[i];
      }

      point_step_ = offset;
      continue;
    }

    if (line_type.substr(0, 5) == "WIDTH") {
      sstream >> width_;
      if (point_step_ != 0) {
        row_step_ = point_step_ * width_;
      }

      continue;
    }

    if (line_type.substr(0, 6) == "HEIGHT") {
      sstream >> height_;
      continue;
    }

    if (line_type.substr(0, 9) == "VIEWPOINT") {
      if (st.size() < 8) {
        return false;
      }

      float vecX, vecY, vecZ;
      sstream >> vecX >> vecY >> vecZ;

      float quatX, quatY, quatZ, quatW;
      sstream >> quatW >> quatX >> quatY >> quatZ;

      continue;
    }

    if (line_type.substr(0, 6) == "POINTS") {
      sstream >> points_;
      continue;
    }

    if (line_type.substr(0, 4) == "DATA") {
      data_idx = static_cast<int>(fs->tellg());

      if (st.at(1).substr(0, 5) == "ascii") {
        data_type_ = 0;
      } else if (st.at(1).substr(0, 6) == "binary") {
        data_type_ = 1;
      }
      continue;
    }
    break;
  }

  // Exit early: if no points have been given, there's no sense to read or check
  // anything anymore
  if (points_ == 0) {
    LOG(ERROR) << "[pcl::PCDReader::readHeader] No points to read\n";
    return false;
  }

  // Compatibility with older PCD file versions
  if (width_ == 0 && height_ == 0) {
    width_ = points_;
    height_ = 1;
    row_step_ = point_step_ * width_;
  }

  if (height_ == 0) {
    height_ = 1;
    LOG(ERROR) << "[PCDReader::readHeader] no HEIGHT given, setting to 1 "
                  "(unorganized).\n";
    if (width_ == 0) {
      width_ = points_;
    }

  } else {
    if (width_ == 0 && points_ != 0) {
      LOG(ERROR) << "[PCDReader::readHeader] HEIGHT given " << height_
                 << " but no WIDTH!\n";
      return false;
    }
  }

  if (int(width_ * height_) != points_) {
    LOG(ERROR) << "[PCDReader::readHeader] HEIGHT " << height_ << " x WIDTH "
               << width_ << " != number of points " << points_;
    return false;
  }

  return true;
}

template <typename PointT>
void PCDReader<PointT>::Destroy() {
  cloud_.Clear();
}

}  // namespace geditor
