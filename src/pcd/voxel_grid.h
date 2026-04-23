
#pragma once

#include <algorithm>
#include "algorithm/bound_box.h"
#include "pcd/pointcloud.h"

namespace geditor {

template <typename PointT>
class VoxelGrid {
 public:
  VoxelGrid() : min_points_per_voxel_(0) {}

 public:
  inline void setInputCloud(PointCloud<PointT> *cloud) { input_ = cloud; }

  inline void setLeafSize(const V3d &leaf_size) {
    leaf_size_ = leaf_size;

    inverse_leaf_size_[0] = 1.0 / leaf_size_[0];
    inverse_leaf_size_[1] = 1.0 / leaf_size_[1];
    inverse_leaf_size_[2] = 1.0 / leaf_size_[2];
  }

  inline void setLeafSize(float lx, float ly, float lz) {
    leaf_size_[0] = lx;
    leaf_size_[1] = ly;
    leaf_size_[2] = lz;

    inverse_leaf_size_[0] = 1.0 / leaf_size_[0];
    inverse_leaf_size_[1] = 1.0 / leaf_size_[1];
    inverse_leaf_size_[2] = 1.0 / leaf_size_[2];
  }

  inline V3d getLeafSize() { return leaf_size_; }

  inline void filter(PointCloud<PointT> &output) {
    if (&output != input_) {
      applyFilter(output);
    } else {
      return;
    }
  }

 private:
  void applyFilter(PointCloud<PointT> &output);

  void getMinMax3D(const PointCloud<PointT> &cloud, V3d &min_pt, V3d &max_pt);

 private:
  V3d leaf_size_;
  V3d inverse_leaf_size_;
  unsigned int min_points_per_voxel_;
  PointCloud<PointT> *input_;
};

template <typename PointT>
void VoxelGrid<PointT>::getMinMax3D(const PointCloud<PointT> &cloud,
                                    V3d &min_pt, V3d &max_pt) {
  BoundBox3d boundBox;
  unsigned int nr_points = cloud.GetPointCount();
  for (unsigned int cp = 0; cp < nr_points; ++cp) {
    const PointT &point = cloud.GetPoint(cp);
    boundBox.ExpandBy(V3d(point.x, point.y, point.z));
  }

  min_pt = boundBox.v_min_;
  max_pt = boundBox.v_max_;
}

struct voxel_cell {
  unsigned int idx;
  unsigned int idy;
  unsigned int idz;

  voxel_cell() : idx(0), idy(0), idz(0) {}

  voxel_cell(unsigned int x, unsigned int y, unsigned int z)
      : idx(x), idy(y), idz(z) {}

  bool operator==(const voxel_cell &p) const {
    if (idx == p.idx && idy == p.idy && idz == p.idz) {
      return true;
    } else {
      return false;
    }
  }
};

struct cloud_point_index_idx {
  voxel_cell point_cell;
  unsigned int cloud_point_index;

  cloud_point_index_idx(const voxel_cell &cell, unsigned int cloud_point_index_)
      : point_cell(cell), cloud_point_index(cloud_point_index_) {}

  bool operator<(const cloud_point_index_idx &p) const {
    if (point_cell.idz < p.point_cell.idz) {
      return true;
    } else if (point_cell.idz == p.point_cell.idz &&
               point_cell.idy < p.point_cell.idy) {
      return true;
    } else if (point_cell.idz == p.point_cell.idz &&
               point_cell.idy == p.point_cell.idy &&
               point_cell.idx < p.point_cell.idx) {
      return true;
    }

    return false;
  }
};

template <typename PointT>
void VoxelGrid<PointT>::applyFilter(PointCloud<PointT> &output) {
  V3d min_p, max_p;
  getMinMax3D(*input_, min_p, max_p);

  long long dx =
      static_cast<long long>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
  long long dy =
      static_cast<long long>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
  long long dz =
      static_cast<long long>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

  const unsigned int max_value = 0xffffffff;
  if (dx > static_cast<long long>(max_value) ||
      dy > static_cast<long long>(max_value) ||
      dz > static_cast<long long>(max_value)) {
    return;
  }

  typedef Vector4<int> Vector4i;
  Vector4i min_b_;
  Vector4i max_b_;

  min_b_[0] = static_cast<int>(Mathd::Floor(min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int>(Mathd::Floor(max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int>(Mathd::Floor(min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int>(Mathd::Floor(max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int>(Mathd::Floor(min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int>(Mathd::Floor(max_p[2] * inverse_leaf_size_[2]));

  std::vector<cloud_point_index_idx> index_vector;

  int nr_point = input_->GetPointCount();
  index_vector.reserve(nr_point);
  for (int it = 0; it < nr_point; ++it) {
    voxel_cell cell;
    PointT pt = input_->GetPoint(it);
    cell.idx =
        static_cast<unsigned int>(Mathd::Floor(pt.x * inverse_leaf_size_[0]) -
                                  static_cast<float>(min_b_[0]));
    cell.idy =
        static_cast<unsigned int>(Mathd::Floor(pt.y * inverse_leaf_size_[1]) -
                                  static_cast<float>(min_b_[1]));
    cell.idz =
        static_cast<unsigned int>(Mathd::Floor(pt.z * inverse_leaf_size_[2]) -
                                  static_cast<float>(min_b_[2]));
    index_vector.push_back(cloud_point_index_idx(cell, it));
  }

  std::sort(index_vector.begin(), index_vector.end(),
            std::less<cloud_point_index_idx>());

  unsigned int total = 0;
  unsigned int index = 0;
  std::vector<std::pair<unsigned int, unsigned int> >
      first_and_last_indices_vector;

  first_and_last_indices_vector.reserve(index_vector.size());
  while (index < index_vector.size()) {
    unsigned int i = index + 1;
    while (i < index_vector.size() &&
           index_vector[i].point_cell == index_vector[index].point_cell)
      ++i;
    if (i - index >= min_points_per_voxel_) {
      ++total;
      first_and_last_indices_vector.push_back(
          std::pair<unsigned int, unsigned int>(index, i));
    }
    index = i;
  }

  index = 0;
  for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp) {
    unsigned int first_index = first_and_last_indices_vector[cp].first;
    unsigned int last_index = first_and_last_indices_vector[cp].second;

    V4d centroid;
    for (unsigned int li = first_index; li < last_index; ++li) {
      PointT pt = input_->GetPoint(index_vector[li].cloud_point_index);

      centroid[0] += pt.x;
      centroid[1] += pt.y;
      centroid[2] += pt.z;
      centroid[3] += pt.w;
    }
    double scalar = static_cast<double>(last_index - first_index);

    PointT outPoint;
    outPoint.x = (float)(centroid[0] / scalar);
    outPoint.y = (float)(centroid[1] / scalar);
    outPoint.z = (float)(centroid[2] / scalar);
    outPoint.w = (float)(centroid[3] / scalar);

    output.PushBack(outPoint);

    ++index;
  }
}

}  // namespace geditor
