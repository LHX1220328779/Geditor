
#pragma once

#include "core/geometry.h"

namespace geditor {

class GeoBSplineCurve3 : public Geometry {
 public:
  GeoBSplineCurve3();

  virtual ~GeoBSplineCurve3();

 public:
  void Hermite(std::vector<Point3d> &items) override;

  int CopyGeometry(double offset, Geometry *line) override;

  bool CheckCurvature(const std::vector<Point3d> &mousePoint, int ctrlBegin,
                      int ctrlEnd);

  int OptimizeCurvature(const Point3d &point);

  int IsPointInEdge(const Point3d &point, Point3d &outPnt, double tolerance,
                    double &fal) override;

  void MoveGeometry(double x, double y, double z);

  int OnPoint(const Point3d &Q, double tolerance);

  double Length() const;

  double GetNeartPoint(const Point3d &point) const override;

  const void *GetDataPtr() const;

  const int GetDataSize() const;

 private:
  //������Ƶ�
  void ControlPoint(const std::vector<Point3d> &mousePoint,
                    std::vector<Point3d> &controlPoint);

  //���²�������
  void OnChange();

 private:
  std::vector<Point3d> m_controlPoint;
  std::vector<Point3d> m_samplePoint;

  bool m_bNeedSample;
  std::vector<Point3d> direct_;
};

}  // namespace geditor
