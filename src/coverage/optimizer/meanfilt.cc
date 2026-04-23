#include "optimizer/meanfilt.h"
#include <fstream>
#include <iostream>

namespace optimizer {

void MeanFilt::Interpolation(SiteVec &anchors, double density) {
  SiteVec buffer;
  if (anchors.size() < 1) return;
  for (auto it = anchors.begin(); it != anchors.end() - 1; ++it) {
    buffer.push_back(*it);

    Site current_point(*it);
    Site next_point(*(it + 1));
    SiteVec insert_list;
    insert_list.clear();
    double tmp_dis = (next_point - current_point).mold();
    int insert_num = tmp_dis / density;
    Site delta;
    if (insert_num < 1) {
      continue;
    } else {
      delta = (next_point - current_point) / insert_num;
    }

    for (int i = 1; i < insert_num; ++i) {
      insert_list.push_back((current_point + delta * i));
    }

    buffer.insert(buffer.end(), insert_list.begin(), insert_list.end());
  }
  buffer.push_back(anchors.back());
  anchors.swap(buffer);
}

void MeanFilt::Filt(SiteVec &to_smooth_path, double density) {
  iner_path_.clear();
  update_path_.clear();

  Interpolation(to_smooth_path, density);
  if (to_smooth_path.size() < 25) {
    return;
  }
  for (auto &i : to_smooth_path) {
    iner_path_.push_back(i);
  }

  int site_num = iner_path_.size();
  update_path_.push_back(iner_path_.front());

  if (site_num > 40) {
    for (int i = 1; i < site_num - 1; ++i) {
      Site new_site(0, 0);
      if (i < 20) {
        for (int j = 0; j < i + 1; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / (2 * i + 2);
        }
        update_path_.push_back(new_site);
      } else if (i > site_num - 21) {
        int filt_num = 2 * (site_num - i);
        for (int j = 0; j < (site_num - i); ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / filt_num;
        }
        update_path_.push_back(new_site);
      } else {
        for (int j = 0; j < 21; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / 42;
        }
        update_path_.push_back(new_site);
      }
    }
    update_path_.push_back(iner_path_.back());
  } else if (site_num > 20) {
    for (int i = 1; i < site_num - 1; ++i) {
      Site new_site(0, 0);
      if (i < 10) {
        for (int j = 0; j < i + 1; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / (2 * i + 2);
        }
        update_path_.push_back(new_site);
      } else if (i > site_num - 11) {
        int filt_num = 2 * (site_num - i);
        for (int j = 0; j < (site_num - i); ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / filt_num;
        }
        update_path_.push_back(new_site);
      } else {
        for (int j = 0; j < 11; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / 22;
        }
        update_path_.push_back(new_site);
      }
    }
    update_path_.push_back(iner_path_.back());
  } else {
    update_path_ = iner_path_;
  }

  to_smooth_path.swap(update_path_);
}

}  // namespace optimizer
