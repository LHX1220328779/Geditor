#include "optimizer/curvedetection.h"

namespace optimizer {

CurveDetection::CurveDetection() {
  fit_curve_smoothlen_ = 5;
  current_path_direction_ = 0;
  fit_curve_len = 1.0;
  fit_curve_reverse_len = 0.5;
  curve_end_ = 0;
  fit_curve_minlen_ = 5.0;
  total_length_ = 0.0;
  curve_limit_ = 0.5;          // radius is 2m
  curve_limit_max_ = 1 / 1.4;  // radius is 1.4m
}

double CurveDetection::RunDetection(geometry::SiteVec& c_path,
                                    int pathdirection) {
  std::cout << c_path.size() << std::endl;
  int path_size = c_path.size();
  geometry::SiteVec path = c_path;
  GetTotalLength(c_path);
  int smooth_path_halflen =
      std::min(fit_curve_smoothlen_, static_cast<int>(path_size / 2));
  double fitlength;
  current_path_direction_ = pathdirection;
  fitlength = GetFitLength(current_path_direction_);
  if (path_size == 0) return 0;
  // window smoothing process
  for (int i = 0; i < path_size; ++i) {
    int j = 0;
    double xsum = 0;
    double ysum = 0;
    int smooth_pt_cnt = 0;
    if (i < smooth_path_halflen) {
      for (j = 0; j <= 2 * i; ++j) {
        xsum += path[j].x;
        ysum += path[j].y;
      }
      smooth_pt_cnt = j;
    } else if (i >= path_size - smooth_path_halflen) {
      for (j = i - (path_size - 1 - i); j < path_size; ++j) {
        xsum += path[j].x;
        ysum += path[j].y;
      }
      smooth_pt_cnt = path_size - (i - (path_size - 1 - i));
    } else {
      for (j = i - smooth_path_halflen; j <= i + smooth_path_halflen; ++j) {
        xsum += path[j].x;
        ysum += path[j].y;
      }
      smooth_pt_cnt = 2 * smooth_path_halflen + 1;
    }
    path[i].x = xsum / smooth_pt_cnt;
    path[i].y = ysum / smooth_pt_cnt;
  }
  // curvature culculating
  double calclength = 0.0;
  geometry::Site samplepoint[3];
  int i;  // j;
  int fitstart = 0;
  int fitend = 0;
  samplepoint[0] = path[0];
  for (i = 0; i < path_size; ++i) {
    calclength =
        std::hypot(samplepoint[0].x - path[i].x, samplepoint[0].y - path[i].y);
    if (calclength >= fitlength) {
      samplepoint[1] = path[i];
      fitstart = i;
      break;
    }
  }
  if (fitstart == 0) {
    for (i = 0; i < path_size; ++i) {
      if (c_path[i].curvature < 0) {
        c_path[i].curvature = curve_end_;
      }
    }
    return 0;
  }
  for (i = fitstart; i < path_size; ++i) {
    int m;                     // n;
    samplepoint[1] = path[i];  // the middle point of vector
    // calculate the left point of vector
    for (m = i; m > 0; --m) {
      calclength = std::hypot(samplepoint[1].x - path[m].x,
                              samplepoint[1].y - path[m].y);
      if (calclength >= fitlength) {
        samplepoint[0] = path[m];
        break;
      }
    }
    if (m == 0) {
      samplepoint[0] = path[0];
    }
    for (m = i; m < path_size; ++m) {
      calclength = std::hypot(samplepoint[1].x - path[m].x,
                              samplepoint[1].y - path[m].y);
      if (calclength >= fitlength) {
        samplepoint[2] = path[m];
        break;
      }
    }
    if (m == path_size) {
      fitend = i;
      if (c_path[fitend - 1].curvature >= 0) {
        curve_end_ = c_path[fitend - 1].curvature;
      }
      break;
    }
    // calculate point and curvature
    geometry::Site vecA, vecB;
    double angleAB = 0.0;
    // calculate the vec A and B
    vecA.x = samplepoint[0].x - samplepoint[1].x;
    vecA.y = samplepoint[0].y - samplepoint[1].y;
    vecB.x = samplepoint[2].x - samplepoint[1].x;
    vecB.y = samplepoint[2].y - samplepoint[1].y;
    // calculate the angle between A and B
    double acosData = (vecA.x * vecB.x + vecA.y * vecB.y) /
                      (std::hypot(vecA.x, vecA.y) * std::hypot(vecB.x, vecB.y));
    if (acosData > 1) {
      acosData = 1;
    } else if (acosData < -1) {
      acosData = -1;
    }
    angleAB = acos(acosData);
    // calculate the curvature based on the angle and distance
    c_path[i].curvature = 2 * cos(angleAB / 2) / fitlength;
  }
  if (fitstart == fitend) {
    if (c_path[fitstart].curvature < 0) {
      c_path[fitstart].curvature = curve_end_;
    }
  }
  for (i = 0; i < fitstart; ++i) {
    if (c_path[i].curvature < 0) {
      for (int j = i; j < fitstart; ++j) {
        if (c_path[j].curvature >= 0) {
          c_path[j].curvature = c_path[j].curvature;
          break;
        }
      }
    }
  }
  if (total_length_ < fit_curve_minlen_) {
    for (i = fitend; i < path_size; ++i) {
      if (c_path[i].curvature < 0) {
        c_path[i].curvature = curve_end_;
      }
    }
  }
  return PointCheck(c_path);
}

double CurveDetection::GetFitLength(int direction) {
  return direction == 0 ? fit_curve_len : fit_curve_reverse_len;
}

bool CurveDetection::GetTotalLength(geometry::SiteVec& path) {
  total_length_ = 0.0;
  double single_len = 0.0;
  if (path.size() <= 1) return false;
  for (int i = 1; i < path.size(); ++i) {
    single_len =
        std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    total_length_ += single_len;
  }
  return true;
}
double CurveDetection::PointCheck(geometry::SiteVec& path) {
  double max_curvature = 0.0;
  std::cout << "path size" << std::endl;
  if (path.size() <= 10) return 2;
  for (int i = 0; i < path.size(); i++) {
    if (i < 10 || i >= path.size() - 10) continue;
    max_curvature = std::max(max_curvature, path[i].curvature);
    if (path[i].curvature > curve_limit_max_) {
      std::cout << "[index]" << i << "," << path[i].x << "," << path[i].y << ","
                << 1.0 / path[i].curvature << std::endl;
    }
  }
  double curve_radius = 1.0 / max_curvature;
  std::cout << "the curve radius is " << curve_radius << std::endl;
  return curve_radius;
  // if (max_curvature > curve_limit_max_) {
  //   return 2;
  // } else if (max_curvature > curve_limit_) {
  //   return 1;
  // }
  // return 0;
}

}  // namespace optimizer