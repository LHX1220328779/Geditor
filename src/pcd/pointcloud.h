
#pragma once

#include "algorithm/bound_box.h"
#include "algorithm/matrix44.h"

#include <vector>

namespace geditor {

template <typename PointT>
class PointCloud {
 public:
  PointCloud(int caption = 1024) { points_.reserve(caption); }

  ~PointCloud() { Clear(); }

  void Clear() {
    points_.clear();
    std::vector<PointT>(points_).swap(points_);
  }

  PointCloud(const PointT *points, int nCount) {
    points_.resize(nCount);

    for (int i = 0; i < nCount; i++) {
      points_[i] = *points++;
    }
  }

  void TransformPointCloud(PointCloud &cloud, const Matrix4x4d &mat) {
    for (size_t i = 0; i < points_.size(); ++i) {
      PointT point_cloud = points_[i];
      V3d new_position =
          mat.TransformCoord(V3d(point_cloud.x, point_cloud.y, point_cloud.z));
      point_cloud.x = new_position[0];
      point_cloud.y = new_position[1];
      point_cloud.z = new_position[2];
      cloud.points_.push_back(point_cloud);
    }
  }

  void TransformPointCloud(const Matrix4x4d &mat) {
    for (size_t i = 0; i < points_.size(); ++i) {
      PointT point_cloud = points_[i];
      V3d new_position =
          mat.TransformCoord(V3d(point_cloud.x, point_cloud.y, point_cloud.z));
      point_cloud.x = (float)new_position[0];
      point_cloud.y = (float)new_position[1];
      point_cloud.z = (float)new_position[2];
      points_[i] = point_cloud;
    }
  }

  void MergePointCloud(const PointCloud &cloud) {
    size_t nOldSize = points_.size();
    size_t nSrcSize = cloud.points_.size();

    if (nOldSize + nSrcSize >= points_.capacity()) {
      points_.reserve(nOldSize + nSrcSize + 102400);
    }

    points_.insert(points_.end(), cloud.points_.begin(), cloud.points_.end());
  }

  void Get3DBoundary(BoundBox3d &tileBox) const {
    tileBox.Reset();

    for (unsigned int index = 0; index < points_.size(); index++) {
      PointT point = points_[index];
      tileBox.ExpandBy(Vector3d(point.x, point.y, point.z));
    }
  }

  unsigned int GetPointCount() const { return (unsigned int)points_.size(); };

  PointT GetPoint(int index) const { return points_[index]; };

  void PushBack(const PointT &point) {
    size_t nOldSize = points_.size();
    size_t nCapacity = points_.capacity();
    if (nCapacity - nOldSize <= 1) {
      points_.reserve(nOldSize + 102400);
    }

    points_.push_back(point);
  }

  void ScissorPCD(const BoundBox3d &box, PointCloud<PointT> &outCloud) const {
    size_t iCount = points_.size();

    for (size_t i = 0; i < iCount; i++) {
      const PointT &point = points_[i];

      if (point.x < box.v_max_[0] && point.x >= box.v_min_[0] &&
          point.y < box.v_max_[1] && point.y >= box.v_min_[1]) {
        outCloud.PushBack(point);
      }
    }
  }

  bool WritePCD(const char *fileName, bool bBinary) {
    FILE *pfile = fopen(fileName, "wb");
    if (pfile != NULL) {
      size_t nCount = points_.size();

      fprintf(pfile, "# .PCD v0.7 - Point Cloud Data file format\r\n");
      fprintf(pfile, "VERSION 0.7\r\n");
      fprintf(pfile, "FIELDS x y z intensity\r\n");
      fprintf(pfile, "SIZE 4 4 4 4\r\n");
      fprintf(pfile, "TYPE F F F F\r\n");
      fprintf(pfile, "COUNT 1 1 1 1\r\n");
      fprintf(pfile, "WIDTH %d\r\n", nCount);
      fprintf(pfile, "HEIGHT 1\r\n");
      fprintf(pfile, "VIEWPOINT 0 0 0 1 0 0 0\r\n");
      fprintf(pfile, "POINTS %d\r\n", nCount);

      if (bBinary) {
        fprintf(pfile, "DATA binary\r\n");
        size_t iSize = sizeof(PointT) * nCount;
        const void *pData = &points_[0];
        size_t nWriteByte = fwrite(pData, 1, iSize, pfile);
        if (nWriteByte != iSize) {
          fclose(pfile);
          return false;
        }
      } else {
        fprintf(pfile, "DATA ascii\r\n");
        for (size_t i = 0; i < nCount; i++) {
          const PointT &point = points_[i];
          fprintf(pfile, "%f %f %f %f\r\n", point.x, point.y, point.z, point.w);
        }
      }
      fclose(pfile);
      return true;
    }
    return false;
  }

 private:
  std::vector<PointT> points_;
};

}  // namespace geditor
