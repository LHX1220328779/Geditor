#include "pathplanner/blanket/coverageinterface.h"
#include "pathplanner/blanket/astarhybrid.h"
#include "pathplanner/blanket/contourstracing.h"
#include "pathplanner/blanket/coverageplanning.h"
#include "pathplanner/blanket/ddplanner.h"
// #include "pathplanner/blanket/localpathplanner.h"
#include <algorithm>
#include <fstream>
#include "geometry/spline.h"

CoverageInterface::CoverageInterface() {
  sample_point_.clear();
  final_path_.clear();
}

// CoverageInterface::CoverageInterface(ros::NodeHandle nh) {
//   nh.param("basemap", basemap, basemap);
// }

CoverageInterface::~CoverageInterface() {}

//**************************public
// interface*************************************//

bool CoverageInterface::PlanningInterface(const double &sx, const double &sy,
                                          const double &syaw, const double &dx,
                                          const double &dy, const double &dyaw,
                                          const std::vector<double> &x,
                                          const std::vector<double> &y) {
  int convert_result = Polygon2Map(x, y);
  if (!convert_result) return false;
  start_.x = sx * 20;
  start_.y = sy * 20;
  start_.x = start_.x - x_offset_;
  start_.y = start_.y - y_offset_;
  start_.yaw = NormalRAD(syaw * M_PI / 180.0);
  end_.x = dx * 20;
  end_.y = dy * 20;
  end_.x = end_.x - x_offset_;
  end_.y = end_.y - y_offset_;
  end_.yaw = NormalRAD(dyaw * M_PI / 180.0);
  ContoursTracing contour_trace(origin_map_);
  // contour_trace.ShowImage();
  contour_trace.ContoursDetection();
  contour_trace.SliceDecomposition(1);
  std::cout << "region_size:" << contour_trace.free_contours_.size()
            << std::endl;
  CoveragePlanning complete_planner(contour_trace, start_, end_, 0);
  std::vector<ccpp::Pointf> ccpp_path;
  geometry::SiteVec site_path;
  complete_planner.CompletePlanningPoly(ccpp_path);
  Pathvec2Sitevec(ccpp_path, site_path);
  AverageSimpling(site_path, sample_point_);
  // GenerateSamplePoint(site_path, sample_point_);
  GenerateCurve(sample_point_, final_path_);
  Recovery(sample_point_);
  Recovery(final_path_);
#if PATH_FILE_DEBUG
  Path2File(sample_point_, "/home/geditor/Desktop/sample-seg");
  Path2File(final_path_, "/home/geditor/Desktop/final-seg");
#endif
  return true;
}

void CoverageInterface::FromSample2Final(const geometry::SiteVec &sample_seg,
                                         geometry::SiteVec &final_seg) {
  std::string sample_path = "/home/geditor/Desktop/sample-seg";
  std::string final_path = "/home/geditor/Desktop/final-seg";
  GenerateCurve(sample_seg, final_seg);
#if PATH_FILE_DEBUG
  Path2File(sample_seg, "/home/geditor/Desktop/sample-seg");
  Path2File(final_seg, "/home/geditor/Desktop/final-seg");
#endif
}

//**************************public
// interface*************************************//

bool CoverageInterface::PlanningInterface(const double &sx, const double &sy,
                                          const double &syaw, const double &dx,
                                          const double &dy, const double &dyaw,
                                          const int &index,
                                          std::string &mappath) {
  start_.x = sx * 20;
  start_.y = sy * 20;
  start_.yaw = NormalRAD(syaw * M_PI / 180.0);
  end_.x = dx * 20;
  end_.y = dy * 20;
  end_.yaw = NormalRAD(dyaw * M_PI / 180.0);
  std::string newmap = mappath + std::to_string(index) + ".png";
  // std::string newmap =
  //     basemap.substr(0, basemap.size() - 11) + std::to_string(index) +
  //     ".png";
  ContoursTracing contour_trace(newmap, 0);
  contour_trace.ContoursDetection();
  contour_trace.SliceDecomposition(1);
  std::cout << "region_size:" << contour_trace.free_contours_.size()
            << std::endl;
  CoveragePlanning complete_planner(contour_trace, start_, end_, index);
  complete_planner.CompletePlanning(mappath);

  return true;
}

float CoverageInterface::NormalRAD(float angle) {
  float a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  if (fabs(a) < 1e-3 || fabs(a - 2.0 * M_PI) < 1e-3) return -M_PI;
  if (fabs(a - M_PI) < 1e-3) return 0.0;
  return a - M_PI;
}

float CoverageInterface::NormalDEG(float angle) {
  float a = std::fmod(angle + 180.0, 2.0 * 180.0);
  if (a < 0.0) {
    a += 360.0;
  }
  if (fabs(a) < 1e-3 || fabs(a - 360.0) < 1e-3) return -180.0;
  if (fabs(a - 180.0) < 1e-3) return 0.0;
  return a - 180.0;
}

bool CoverageInterface::Polygon2Map(const std::vector<double> &x,
                                    const std::vector<double> &y) {
  if (x.size() != y.size()) return false;
  if (x.size() == 0 || y.size() == 0) return false;
  std::vector<cv::Point> contours;
  auto x_min_it = std::min_element(std::begin(x), std::end(x));
  auto x_max_it = std::max_element(std::begin(x), std::end(x));
  auto y_min_it = std::min_element(std::begin(y), std::end(y));
  auto y_max_it = std::max_element(std::begin(y), std::end(y));
  int x_min = static_cast<int>((*x_min_it) * 20.0) - 100;
  int x_max = static_cast<int>((*x_max_it) * 20.0) + 100;
  int y_min = static_cast<int>((*y_min_it) * 20.0) - 100;
  int y_max = static_cast<int>((*y_max_it) * 20.0) + 100;
  std::cout << x_min << std::endl;
  int col = x_max - x_min + 1;
  int row = y_max - y_min + 1;
  for (std::size_t i = 0; i < x.size(); ++i) {
    cv::Point tmp;
    tmp.x = static_cast<int>(x[i] * 20.0) - x_min - 1;
    tmp.y = static_cast<int>(y[i] * 20.0) - y_min - 1;
    contours.push_back(tmp);
  }
  x_offset_ = x_min - 1;
  y_offset_ = y_min - 1;
  origin_map_ = cv::Mat(row, col, CV_8UC1, cv::Scalar(0));

  for (int x = 0; x < col; ++x) {
    for (int y = 0; y < row; ++y) {
      if (cv::pointPolygonTest(contours, cv::Point(x, y), false) > 0) {
        origin_map_.at<uchar>(y, x) = 255;
      } else {
        origin_map_.at<uchar>(y, x) = 0;
      }
    }
  }
  return true;
}

bool CoverageInterface::Pathvec2Sitevec(const std::vector<ccpp::Pointf> &path,
                                        geometry::SiteVec &pathsite) {
  for (const auto &p : path) {
    pathsite.emplace_back(geometry::Site());
    pathsite.back().x = p.x / 20.0;
    pathsite.back().y = p.y / 20.0;
    pathsite.back().angle = NormalDEG(p.yaw * 180.00 / M_PI);
  }
  optimizer::CurveDetection cdt;
  cdt.RunDetection(pathsite);
  return true;
}

bool CoverageInterface::AverageSimpling(const geometry::SiteVec &site_path,
                                        geometry::SiteVec &sample_point) {
  int point_size = site_path.size();
  int i = 0;
  for (i = 0; i < point_size; i += 10) {
    sample_point.push_back(site_path[i]);
  }
  if (i < point_size + 10) {
    sample_point.push_back(site_path.back());
  }
  return true;
}

bool CoverageInterface::GenerateSamplePoint(const geometry::SiteVec &site_path,
                                            geometry::SiteVec &sample_point) {
  std::vector<int> init_sample;
  if (site_path.size() < 40) return false;
  init_sample.push_back(0);
  for (std::size_t i = 14; i < site_path.size() - 20; i += 20)
    init_sample.push_back(i);
  sample_point.push_back(site_path[0]);
  for (std::size_t i = 1; i < init_sample.size() - 1;) {
    sample_point.push_back(site_path[init_sample[i]]);
    sample_point.back().velocity = init_sample[i] - init_sample[i - 1];
    if (site_path[init_sample[i]].curvature > 0.4) {
      i++;
    } else if (site_path[init_sample[i]].curvature > 0.3) {
      int j_highcur;
      for (j_highcur = i + 1;
           j_highcur < i + 2 && j_highcur < init_sample.size() - 1;
           j_highcur++) {
        if (site_path[init_sample[j_highcur]].curvature > 0.4) {
          break;
        } else {
          continue;
        }
      }
      i = j_highcur;
    } else if (site_path[init_sample[i]].curvature > 0.2) {
      int j_midcur;
      for (j_midcur = i + 1;
           j_midcur < i + 3 && j_midcur < init_sample.size() - 1; j_midcur++) {
        if (site_path[init_sample[j_midcur]].curvature > 0.3) {
          break;
        } else {
          continue;
        }
      }
      i = j_midcur;
    } else {
      int j_lowcur;
      for (j_lowcur = i + 1; j_lowcur < init_sample.size() - 1; j_lowcur++) {
        if (site_path[init_sample[j_lowcur]].curvature > 0.2) {
          break;
        } else {
          continue;
        }
      }
      i = j_lowcur;
    }
  }
  sample_point.push_back(site_path.back());
  return true;
}

bool CoverageInterface::GenerateCurve(const geometry::SiteVec &source_data,
                                      geometry::SiteVec &return_data) {
  return_data.clear();

  int i, j;
  float t;
  PointXY tmp_point;                  //插值点坐标
  float g[4][4], g1[4][4], g2[4][4];  // region 创建并计算G(t)及其一阶及二阶导数

  if (source_data.size() >= 4) {
    for (j = 0; j < 4; j++) {
      t = j / 4.0;
      g[0][j] = (-t * t * t + 3 * t * t - 3 * t + 1) / 6;
      g[1][j] = (3 * t * t * t - 6 * t * t + 4) / 6;
      g[2][j] = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
      g[3][j] = t * t * t / 6;

      g1[0][j] = (-t * t + 2 * t - 1) / 2;
      g1[1][j] = (3 * t * t - 4 * t) / 2;
      g1[2][j] = (-3 * t * t + 2 * t + 1) / 2;
      g1[3][j] = t * t / 2;

      g2[0][j] = -t + 1;
      g2[1][j] = 3 * t - 2;
      g2[2][j] = -3 * t + 1;
      g2[3][j] = t;
    }

    // 为保证经过起点，在点集开始加上两点

    for (j = 0; j < 4; j++) {
      tmp_point.x =
          g[0][j] * source_data.at(0).x + g[1][j] * source_data.at(0).x +
          g[2][j] * source_data.at(0).x + g[3][j] * source_data.at(1).x;
      tmp_point.y =
          g[0][j] * source_data.at(0).y + g[1][j] * source_data.at(0).y +
          g[2][j] * source_data.at(0).y + g[3][j] * source_data.at(1).y;
      //        tmp_point.x = g[0][j] * opts[0][0] + g[1][j] * opts[0][0] +
      //        g[2][j] * opts[0][0] + g[3][j] * opts[1][0]; tmp_point.y =
      //        g[0][j] * opts[0][1] + g[1][j] * opts[0][1] + g[2][j] *
      //        opts[0][1] + g[3][j] * opts[1][1]; x1 = g1[0][j] * opts[0][0] +
      //        g1[1][j] * opts[0][0] + g1[2][j] * opts[0][0] + g1[3][j] *
      //        opts[1][0]; y1 = g1[0][j] * opts[0][1] + g1[1][j] * opts[0][1] +
      //        g1[2][j] * opts[0][1] + g1[3][j] * opts[1][1]; x2 = g2[0][j] *
      //        opts[0][0] + g2[1][j] * opts[0][0] + g2[2][j] * opts[0][0] +
      //        g2[3][j] * opts[1][0]; y2 = g2[0][j] * opts[0][1] + g2[1][j] *
      //        opts[0][1] + g2[2][j] * opts[0][1] + g2[3][j] * opts[1][1];
      //        ls.Add(new double[] { x, y, a });
      return_data.push_back(tmp_point);
    }
    for (j = 0; j < 4; j++) {
      tmp_point.x =
          g[0][j] * source_data.at(0).x + g[1][j] * source_data.at(0).x +
          g[2][j] * source_data.at(1).x + g[3][j] * source_data.at(2).x;
      tmp_point.y =
          g[0][j] * source_data.at(0).y + g[1][j] * source_data.at(0).y +
          g[2][j] * source_data.at(1).y + g[3][j] * source_data.at(2).y;
      //        x = g[0][j] * opts[0][0] + g[1][j] * opts[0][0] + g[2][j] *
      //        opts[1][0] + g[3][j] * opts[2][0]; y = g[0][j] * opts[0][1] +
      //        g[1][j] * opts[0][1] + g[2][j] * opts[1][1] + g[3][j] *
      //        opts[2][1]; x1 = g1[0][j] * opts[0][0] + g1[1][j] * opts[0][0] +
      //        g1[2][j] * opts[1][0] + g1[3][j] * opts[2][0]; y1 = g1[0][j] *
      //        opts[0][1] + g1[1][j] * opts[0][1] + g1[2][j] * opts[1][1] +
      //        g1[3][j] * opts[2][1]; x2 = g2[0][j] * opts[0][0] + g2[1][j] *
      //        opts[0][0] + g2[2][j] * opts[1][0] + g2[3][j] * opts[2][0]; y2 =
      //        g2[0][j] * opts[0][1] + g2[1][j] * opts[0][1] + g2[2][j] *
      //        opts[1][1] + g2[3][j] * opts[2][1]; ls.Add(new double[] { x, y,
      //        a });
      return_data.push_back(tmp_point);
    }

    //    iPointXY1 =  m_gNavRoute.begin();
    //    source_data.push_front(*iPointXY1);

    for (i = 0; i < source_data.size() - 3; i++) {
      for (j = 0; j < 4; j++) {
        tmp_point.x = g[0][j] * source_data.at(i).x +
                      g[1][j] * source_data.at(i + 1).x +
                      g[2][j] * source_data.at(i + 2).x +
                      g[3][j] * source_data.at(i + 3).x;
        tmp_point.y = g[0][j] * source_data.at(i).y +
                      g[1][j] * source_data.at(i + 1).y +
                      g[2][j] * source_data.at(i + 2).y +
                      g[3][j] * source_data.at(i + 3).y;
        //            x = g[0][j] * opts[i][0] + g[1][j] * opts[i + 1][0]
        //                    + g[2][j] * opts[i + 2][0] + g[3][j] * opts[i +
        //                    3][0];
        //            y = g[0][j] * opts[i][1] + g[1][j] * opts[i + 1][1]
        //                    + g[2][j] * opts[i + 2][1] + g[3][j] * opts[i +
        //                    3][1];
        //            x1 = g1[0][j] * opts[i][0] + g1[1][j] * opts[i + 1][0]
        //                    + g1[2][j] * opts[i + 2][0] + g1[3][j] * opts[i +
        //                    3][0];
        //            y1 = g1[0][j] * opts[i][1] + g1[1][j] * opts[i + 1][1]
        //                    + g1[2][j] * opts[i + 2][1] + g1[3][j] * opts[i +
        //                    3][1];
        //            x2 = g2[0][j] * opts[i][0] + g2[1][j] * opts[i + 1][0]
        //                    + g2[2][j] * opts[i + 2][0] + g2[3][j] * opts[i +
        //                    3][0];
        //            y2 = g2[0][j] * opts[i][1] + g2[1][j] * opts[i + 1][1]
        //                    + g2[2][j] * opts[i + 2][1] + g2[3][j] * opts[i +
        //                    3][1];
        //            ls.Add(new double[] { x, y, a });
        return_data.push_back(tmp_point);
      }
    }

    // 为保证经过终点，在点集结尾加上两点
    int endNum = source_data.size();
    for (j = 0; j < 4; j++) {
      tmp_point.x = g[0][j] * source_data.at(endNum - 3).x +
                    g[1][j] * source_data.at(endNum - 2).x +
                    g[2][j] * source_data.at(endNum - 1).x +
                    g[3][j] * source_data.at(endNum - 1).x;
      tmp_point.y = g[0][j] * source_data.at(endNum - 3).y +
                    g[1][j] * source_data.at(endNum - 2).y +
                    g[2][j] * source_data.at(endNum - 1).y +
                    g[3][j] * source_data.at(endNum - 1).y;
      //        x = g[0][j] * opts[opts.Count - 3][0] + g[1][j] *
      //        opts[opts.Count - 2][0]
      //                + g[2][j] * opts[opts.Count - 1][0] + g[3][j] *
      //                opts[opts.Count - 1][0];
      //        y = g[0][j] * opts[opts.Count - 3][1] + g[1][j] *
      //        opts[opts.Count - 2][1]
      //                + g[2][j] * opts[opts.Count - 1][1] + g[3][j] *
      //                opts[opts.Count - 1][1];
      //        x1 = g1[0][j] * opts[opts.Count - 3][0] + g1[1][j] *
      //        opts[opts.Count - 2][0]
      //                + g1[2][j] * opts[opts.Count - 1][0] + g1[3][j] *
      //                opts[opts.Count - 1][0];
      //        y1 = g1[0][j] * opts[opts.Count - 3][1] + g1[1][j] *
      //        opts[opts.Count - 2][1]
      //                + g1[2][j] * opts[opts.Count - 1][1] + g1[3][j] *
      //                opts[opts.Count - 1][1];
      //        x2 = g2[0][j] * opts[opts.Count - 3][0] + g2[1][j] *
      //        opts[opts.Count - 2][0]
      //                + g2[2][j] * opts[opts.Count - 1][0] + g2[3][j] *
      //                opts[opts.Count - 1][0];
      //        y2 = g2[0][j] * opts[opts.Count - 3][1] + g2[1][j] *
      //        opts[opts.Count - 2][1]
      //                + g2[2][j] * opts[opts.Count - 1][1] + g2[3][j] *
      //                opts[opts.Count - 1][1];
      //        ls.Add(new double[] { x, y, a });
      return_data.push_back(tmp_point);
    }
    for (j = 0; j < 4; j++) {
      tmp_point.x = g[0][j] * source_data.at(endNum - 2).x +
                    g[1][j] * source_data.at(endNum - 1).x +
                    g[2][j] * source_data.at(endNum - 1).x +
                    g[3][j] * source_data.at(endNum - 1).x;
      tmp_point.y = g[0][j] * source_data.at(endNum - 2).y +
                    g[1][j] * source_data.at(endNum - 1).y +
                    g[2][j] * source_data.at(endNum - 1).y +
                    g[3][j] * source_data.at(endNum - 1).y;
      //        x = g[0][j] * opts[opts.Count - 2][0] + g[1][j] *
      //        opts[opts.Count - 1][0]
      //                + g[2][j] * opts[opts.Count - 1][0] + g[3][j] *
      //                opts[opts.Count - 1][0];
      //        y = g[0][j] * opts[opts.Count - 2][1] + g[1][j] *
      //        opts[opts.Count - 1][1]
      //                + g[2][j] * opts[opts.Count - 1][1] + g[3][j] *
      //                opts[opts.Count - 1][1];
      //        x1 = g1[0][j] * opts[opts.Count - 2][0] + g1[1][j] *
      //        opts[opts.Count - 1][0]
      //                + g1[2][j] * opts[opts.Count - 1][0] + g1[3][j] *
      //                opts[opts.Count - 1][0];
      //        y1 = g1[0][j] * opts[opts.Count - 2][1] + g1[1][j] *
      //        opts[opts.Count - 1][1]
      //                + g1[2][j] * opts[opts.Count - 1][1] + g1[3][j] *
      //                opts[opts.Count - 1][1];
      //        x2 = g2[0][j] * opts[opts.Count - 2][0] + g2[1][j] *
      //        opts[opts.Count - 1][0]
      //                + g2[2][j] * opts[opts.Count - 1][0] + g2[3][j] *
      //                opts[opts.Count - 1][0];
      //        y2 = g2[0][j] * opts[opts.Count - 2][1] + g2[1][j] *
      //        opts[opts.Count - 1][1]
      //                + g2[2][j] * opts[opts.Count - 1][1] + g2[3][j] *
      //                opts[opts.Count - 1][1];
      //        ls.Add(new double[] { x, y, a });
      return_data.push_back(tmp_point);
    }
    std::cout << "[true]" << return_data.size() << std::endl;
    return true;
  } else {
    std::cout << "[false]" << source_data.size() << std::endl;
    return false;
  }
  // return true;
}

void CoverageInterface::Path2File(const geometry::SiteVec path,
                                  const std::string &blanket_path) {
  std::fstream output_file;
  // std::string blanket_path = "/home/geditor/Desktop/temp-seg";
  output_file.open(blanket_path, std::ofstream::out);
  for (const auto &p : path) {
    output_file << p.x << "," << p.y << "," << p.angle << "," << p.curvature
                << std::endl;
  }
  output_file.close();
}

void CoverageInterface::Recovery(geometry::SiteVec &path) {
  for (std::size_t i = 0; i < path.size(); i++) {
    path[i].x = path[i].x + x_offset_ / 20.0;
    path[i].y = path[i].y + y_offset_ / 20.0;
  }
}