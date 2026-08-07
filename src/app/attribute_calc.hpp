#pragma once
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "core/geometry.h"
#include "core/log.h"
// 定义常量
const double WGS84_A = 6378137.0;       // WGS84椭球体长半轴
const double WGS84_E = 0.0818191908425; // WGS84椭球体第一偏心率

namespace geditor
{
  namespace attribute_calc
  {

    // Initialized from mine_origins.yaml by the UI/command-line entry point.
    double GLOBAL_ORIGIN_LAT = 0.0;
    double GLOBAL_ORIGIN_LON = 0.0;
    double GLOBAL_ORIGIN_ALT = 0.0;
    void InitGlobalOrigin(double lat, double lon, double alt)
    {
      GLOBAL_ORIGIN_LAT = lat;
      GLOBAL_ORIGIN_LON = lon;
      GLOBAL_ORIGIN_ALT = alt;
    }
    double distance(const Point3d &p1, const Point3d &p2)
    {
      return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                       (p2.y - p1.y) * (p2.y - p1.y) +
                       (p2.z - p1.z) * (p2.z - p1.z));
    }

    double distance2d(const Point3d &p1, const Point3d &p2)
    {
      return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                       (p2.y - p1.y) * (p2.y - p1.y));
    }

    // 三维线性插值函数
    Point3d Interpolate(const Point3d &p1, const Point3d &p2, double ratio)
    {
      return Point3d(p1.x + ratio * (p2.x - p1.x), p1.y + ratio * (p2.y - p1.y),
                     p1.z + ratio * (p2.z - p1.z));
    }
    double kmph2mps(double kmph) { return kmph * 1000.0 / 3600.0; }
    // 对三维车道线进行插值，使相邻点之间的距离不超过0.5米 type:
    // 0：2D差值，1：3D差值
    std::vector<Point3d> InterpolateLane(const std::vector<Point3d> &lanePoints,
                                         double maxDist = 0.5)
    {
      std::vector<Point3d> interpolatedPoints;

      for (size_t i = 0; i < lanePoints.size() - 1; ++i)
      {
        const Point3d &p1 = lanePoints[i];
        const Point3d &p2 = lanePoints[i + 1];

        double dist = distance(p1, p2);
        if (dist > maxDist)
        {
          // 插值
          int numSegments = static_cast<int>(std::ceil(dist / maxDist));
          for (int j = 0; j < numSegments; ++j)
          {
            double ratio = static_cast<double>(j) / numSegments;
            interpolatedPoints.push_back(Interpolate(p1, p2, ratio));
          }
        }
        else
        {
          // 如果距离小于等于0.5米，直接添加点
          interpolatedPoints.push_back(p1);
        }
      }
      // 添加最后一个点
      interpolatedPoints.push_back(lanePoints.back());
      return interpolatedPoints;
    }

    Point3d TransGps2Pt(const double d_longitude, const double d_latitude,
                        const float d_altitude)
    {
      double f_l0 = GLOBAL_ORIGIN_LAT;
      double f_lamda0 = GLOBAL_ORIGIN_LON;
      float f_hb = GLOBAL_ORIGIN_ALT;
      double f_re0 =
          WGS84_A / (sqrt(1.0 - WGS84_E * WGS84_E * sin(f_l0 * M_PI / 180) *
                                    sin(f_l0 * M_PI / 180.0)));
      double f_x0 =
          (f_re0 + f_hb) * cos(f_l0 * M_PI / 180.0) * cos(f_lamda0 * M_PI / 180.0);
      double f_y0 =
          (f_re0 + f_hb) * cos(f_l0 * M_PI / 180.0) * sin(f_lamda0 * M_PI / 180.0);
      double f_z0 =
          ((1.0 - WGS84_E * WGS84_E) * f_re0 + f_hb) * sin(f_l0 * M_PI / 180.0);

      double f_l = d_latitude;
      double f_lamda = d_longitude;
      float f_h = d_altitude;
      double f_re =
          WGS84_A / (sqrt(1.0 - WGS84_E * WGS84_E * sin(f_l * M_PI / 180.0) *
                                    sin(f_l * M_PI / 180.0)));
      double f_x =
          (f_re + f_h) * cos(f_l * M_PI / 180.0) * cos(f_lamda * M_PI / 180.0);
      double f_y =
          (f_re + f_h) * cos(f_l * M_PI / 180.0) * sin(f_lamda * M_PI / 180.0);
      double f_z =
          ((1.0 - WGS84_E * WGS84_E) * f_re + f_h) * sin(f_l * M_PI / 180.0);

      double f_dx = f_x - f_x0;
      double f_dy = f_y - f_y0;
      double f_dz = f_z - f_z0;
      double f_dn = -sin(f_l * M_PI / 180.0) * cos(f_lamda * M_PI / 180.0) * f_dx -
                    sin(f_l * M_PI / 180.0) * sin(f_lamda * M_PI / 180.0) * f_dy +
                    cos(f_l * M_PI / 180.0) * f_dz;
      double f_de =
          -sin(f_lamda * M_PI / 180.0) * f_dx + cos(f_lamda * M_PI / 180.0) * f_dy;

      Point3d st_point3d;
      st_point3d.x = f_de;
      st_point3d.y = f_dn;
      st_point3d.z = 0.0;

      return st_point3d;
    }

    // 从局部平面坐标转换回经纬高
    Point3d TransPt2Gps(const Point3d &st_point3d)
    {
      // 全局原点参数
      double f_l0 = GLOBAL_ORIGIN_LAT * M_PI / 180.0;     // 全局原点纬度（弧度）
      double f_lamda0 = GLOBAL_ORIGIN_LON * M_PI / 180.0; // 全局原点经度（弧度）
      double f_hb = GLOBAL_ORIGIN_ALT;                    // 全局原点高度

      // 计算全局原点的 ECEF 坐标
      double f_re0 =
          WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * sin(f_l0) * sin(f_l0));
      double f_x0 = (f_re0 + f_hb) * cos(f_l0) * cos(f_lamda0);
      double f_y0 = (f_re0 + f_hb) * cos(f_l0) * sin(f_lamda0);
      double f_z0 = ((1.0 - WGS84_E * WGS84_E) * f_re0 + f_hb) * sin(f_l0);

      // 局部平面坐标（ENU）到 ECEF 坐标的转换
      double f_de = st_point3d.x; // 东向
      double f_dn = st_point3d.y; // 北向
      double f_du = st_point3d.z; // 天向

      // 计算目标点在 ECEF 坐标系中的坐标
      double f_dx = -sin(f_lamda0) * f_de - sin(f_l0) * cos(f_lamda0) * f_dn +
                    cos(f_l0) * cos(f_lamda0) * f_du;
      double f_dy = cos(f_lamda0) * f_de - sin(f_l0) * sin(f_lamda0) * f_dn +
                    cos(f_l0) * sin(f_lamda0) * f_du;
      double f_dz = cos(f_l0) * f_dn + sin(f_l0) * f_du;

      double f_x = f_x0 + f_dx;
      double f_y = f_y0 + f_dy;
      double f_z = f_z0 + f_dz;

      // 从 ECEF 坐标转换为经纬高
      double f_p = sqrt(f_x * f_x + f_y * f_y);
      double f_theta =
          atan2(f_z * WGS84_A, f_p * WGS84_A * (1.0 - WGS84_E * WGS84_E));

      double f_latitude =
          atan2(f_z + WGS84_E * WGS84_E * WGS84_A * pow(sin(f_theta), 3),
                f_p - WGS84_E * WGS84_E * WGS84_A * pow(cos(f_theta), 3));
      double f_longitude = atan2(f_y, f_x);
      double f_re = WGS84_A / sqrt(1.0 - WGS84_E * WGS84_E * sin(f_latitude) *
                                             sin(f_latitude));
      double f_altitude = f_p / cos(f_latitude) - f_re;

      // 转换为度
      Point3d st_gps;
      st_gps.y = f_latitude * 180.0 / M_PI;
      st_gps.x = f_longitude * 180.0 / M_PI;
      st_gps.z = f_altitude;
      return st_gps;
    }
    double calculateHeading(double A_x, double A_y, double B_x, double B_y)
    {
      double deltaY = B_y - A_y;
      double deltaX = B_x - A_x;
      double heading = std::atan2(deltaY, deltaX);
      // 从x轴逆时针旋转的角度变为y轴顺时针的角度
      double degree = 450.0 - (heading * 180.0 / M_PI);
      // 将角度调整到 0 到 360 的范围
      degree = fmod(degree, 360.0); // 取模运算，确保角度在 -360 到 360 之间
      if (degree < 0)
      {
        degree += 360.0; // 如果角度为负，加上 360
      }
      return degree;
    }
    double calculateHeading(const Point3d &p1, const Point3d &p2)
    {
      return calculateHeading(p1.x, p1.y, p2.x, p2.y);
    }

    void TestCalcHeading()
    {
      double A_x = -52.47129047;
      double A_y = -268.9751058;
      double B_x = -52.26148161;
      double B_y = -268.7200996;
      double heading = calculateHeading(A_x, A_y, B_x, B_y);
      std::cout << "Heading: " << heading << std::endl;
    }

    //===================================计算左宽右宽===========================================//
    bool raySegmentIntersection(const Point3d &origin, const Point3d &direction,
                                const Point3d &p1, const Point3d &p2,
                                Point3d &intersection)
    {
      const double epsilon = 1e-6; // 浮点精度阈值

      // 线段向量
      Point3d seg_vec = {p2.x - p1.x, p2.y - p1.y, 0.0};
      // 射线向量到线段起点
      Point3d origin_to_p1 = {p1.x - origin.x, p1.y - origin.y, 0.0};

      // 计算叉积
      double cross_dir_seg = direction.x * seg_vec.y - direction.y * seg_vec.x;
      double cross_origin_seg =
          origin_to_p1.x * seg_vec.y - origin_to_p1.y * seg_vec.x;

      // 处理射线与线段平行的情况
      if (std::abs(cross_dir_seg) < epsilon)
      {
        // 共线检查
        if (std::abs(cross_origin_seg) < epsilon)
        {
          // 共线时，检查线段端点是否在射线上
          double t1 = (p1.x - origin.x) / (direction.x + epsilon);
          double t2 = (p2.x - origin.x) / (direction.x + epsilon);
          // 如果方向向量x接近0，改用y分量计算
          if (std::abs(direction.x) < epsilon)
          {
            t1 = (p1.y - origin.y) / (direction.y + epsilon);
            t2 = (p2.y - origin.y) / (direction.y + epsilon);
          }

          // 检查端点是否在射线正方向
          bool p1_on_ray = (t1 >= -epsilon);
          bool p2_on_ray = (t2 >= -epsilon);

          if (p1_on_ray || p2_on_ray)
          {
            // 返回最近的交点
            double dist_p1 = distance(origin, p1);
            double dist_p2 = distance(origin, p2);
            intersection = (dist_p1 < dist_p2) ? p1 : p2;
            return true;
          }
        }
        return false; // 平行但不共线，或共线但无交点
      }

      // 计算交点参数
      double t = cross_origin_seg / cross_dir_seg;
      double u = (origin_to_p1.x * direction.y - origin_to_p1.y * direction.x) /
                 cross_dir_seg;

      // 参数有效性检查
      if (t >= -epsilon && u >= -epsilon && u <= 1.0 + epsilon)
      {
        // 计算精确交点坐标
        intersection.x = origin.x + t * direction.x;
        intersection.y = origin.y + t * direction.y;

        // 二次验证交点是否在线段上（应对浮点误差）
        double seg_length = distance(p1, p2);
        double d1 = distance(intersection, p1);
        double d2 = distance(intersection, p2);
        if (std::abs(d1 + d2 - seg_length) > 1e-3)
        {
          return false; // 交点不在线段上
        }
        return true;
      }
      return false;
    }

    // 主算法：计算中心线到边界的横向偏移
    std::vector<double> calculateLateralOffsets(
        int lane_id, const std::vector<Point3d> &central_line,
        const std::vector<Point3d> &boundary_line, bool is_left_boundary)
    {
      std::vector<double> offsets;

      for (size_t i = 0; i < central_line.size(); ++i)
      {
        // 1. 计算当前中心线点的切线方向
        Point3d tangent;
        if (i == 0)
        { // 起始点
          tangent.x = central_line[i + 1].x - central_line[i].x;
          tangent.y = central_line[i + 1].y - central_line[i].y;
        }
        else if (i == central_line.size() - 1)
        { // 结束点
          tangent.x = central_line[i].x - central_line[i - 1].x;
          tangent.y = central_line[i].y - central_line[i - 1].y;
        }
        else
        { // 中间点
          tangent.x = central_line[i + 1].x - central_line[i - 1].x;
          tangent.y = central_line[i + 1].y - central_line[i - 1].y;
        }

        // 2. 计算法线方向（旋转90度）
        Point3d normal(-tangent.y, tangent.x, 0.0); // 左边界方向
        if (!is_left_boundary)
          normal = Point3d(tangent.y, -tangent.x, 0.0); // 右边界方向

        // 归一化法向量
        double len = std::hypot(normal.x, normal.y);
        if (len > 1e-6)
        {
          normal.x /= len;
          normal.y /= len;
        }

        // 3. 沿法线方向搜索边界交点
        double max_search_dist = 50.0; // 最大搜索距离（根据实际情况调整）
        Point3d ray_end = {central_line[i].x + normal.x * max_search_dist,
                           central_line[i].y + normal.y * max_search_dist, 0.0};

        double min_dist = std::numeric_limits<double>::max();
        Point3d closest_point;
        bool found_intersection = false;

        // 遍历边界线段
        for (size_t j = 0; j < boundary_line.size() - 1; ++j)
        {
          Point3d intersect;
          if (raySegmentIntersection(central_line[i], normal, boundary_line[j],
                                     boundary_line[j + 1], intersect))
          {
            double dist = distance(central_line[i], intersect);
            if (dist < min_dist)
            {
              min_dist = dist;
              closest_point = intersect;
              found_intersection = true;
            }
          }
        }
        if (!found_intersection)
        {
          int idx = -1;
          if (i == 0 || i == central_line.size() - 1)
          { // 如果是起始点或结束点
            idx = i == 0 ? 0 : boundary_line.size() - 1;
            double dist = distance(central_line[i], boundary_line[idx]);
            min_dist = dist;
            closest_point = boundary_line[idx];
          }
          else
          { // 如果是中间点
            // 计算与边界的首尾点距离，取最小值
            double dist_first = distance(central_line[i], boundary_line[0]);
            double dist_last =
                distance(central_line[i], boundary_line[boundary_line.size() - 1]);
            min_dist = std::min(dist_first, dist_last);
            idx = (dist_first < dist_last) ? 0 : boundary_line.size() - 1;
            closest_point = boundary_line[idx];
          }
          // std::cout << "intersection found for central line point lane_id= "
          //           << lane_id << " is left=" << is_left_boundary << " index=" << i
          //           << " ,boundary idx=" << idx << std::endl;
        }
        offsets.push_back(min_dist);
        // 记录偏移距离
        // if (found_intersection) {
        //   offsets.push_back(min_dist);
        // } else {
        //   offsets.push_back(-1.0);  // 无效值标记
        //   std::cout << "No intersection found for central line point lane_id= "
        //             << lane_id << " is left=" << is_left_boundary << " index=" << i
        //             << " maxsize=" << central_line.size() << std::endl;
        // }
        if (min_dist > 5.0)
        {
          std::cout << "Warning: large offset detected for lane_id= " << lane_id
                    << " is left=" << is_left_boundary << " index=" << i
                    << " ,offset=" << min_dist << std::endl;
        }
      }

      return offsets;
    }
    //==============================================================================//

  }; // namespace attribute_calc
} // namespace geditor
