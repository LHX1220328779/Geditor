#include "core/compute_curve.h"

namespace geditor {

ComputeCurve::ComputeCurve() {}

ComputeCurve::~ComputeCurve() {}

double ComputeCurve::dist_get(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;

  double dist = sqrt(dx * dx + dy * dy);
  return dist;
}

void ComputeCurve::FittingCurveBy2Vector(const std::vector<Point3d> &items,
                                         std::vector<sPoint> &PathPoints) {
  std::vector<sPoint> PathPointsSmooth;

  for (int i = 0; i < items.size(); i++) {
    sPoint spnt;
    spnt.x = items[i].x;
    spnt.y = items[i].y;
    spnt.curvature = 0;

    PathPointsSmooth.push_back(spnt);
    PathPoints.push_back(spnt);
  }

  double FitLength = 1.0;  //ȡ�� ·�� ������ģ��Ϊ 2.0m
  double CurveEnd = 0;

  int PathPointSize = PathPointsSmooth.size();
  if (PathPointSize == 0) return;

  /********************************************************************************/
  double CalcLength = 0;
  sPoint SamplePoint[3];
  int i;
  int FitStart = 0;
  int FitEnd = 0;

  SamplePoint[0] = PathPointsSmooth[0];
  for (i = 0; i < PathPointSize; ++i) {
    CalcLength = dist_get(SamplePoint[0].x, SamplePoint[0].y,
                          PathPointsSmooth[i].x, PathPointsSmooth[i].y);

    if (CalcLength >= FitLength) {
      SamplePoint[1] = PathPointsSmooth[i];
      FitStart = i;
      break;
    }
  }
  if (FitStart == 0) {
    for (i = 0; i < PathPointSize; ++i) {
      if (PathPoints[i].curvature < 0) PathPoints[i].curvature = CurveEnd;
    }
    return;
  }
  //��ʼ������ȡ����
  for (i = FitStart; i < PathPointSize; ++i) {
    double PointDist = FitLength;
    int m;
    //�����м��
    SamplePoint[1] = PathPointsSmooth[i];
    //�����������յ�
    for (m = i; m > 0; --m) {
      CalcLength = dist_get(SamplePoint[1].x, SamplePoint[1].y,
                            PathPointsSmooth[m].x, PathPointsSmooth[m].y);
      if (CalcLength >= FitLength) {
        PointDist = CalcLength;
        SamplePoint[0] = PathPointsSmooth[m];
        break;
      }
    }
    if (m == 0) SamplePoint[0] = PathPointsSmooth[0];
    //�����������յ�
    for (m = i; m < PathPointSize; ++m) {
      CalcLength = dist_get(SamplePoint[1].x, SamplePoint[1].y,
                            PathPointsSmooth[m].x, PathPointsSmooth[m].y);

      if (CalcLength >= FitLength) {
        PointDist = CalcLength;
        SamplePoint[2] = PathPointsSmooth[m];
        break;
      }
    }

    if (m == PathPointSize) {
      FitEnd = i;
      if (PathPoints[FitEnd - 1].curvature >= 0) {
        CurveEnd = PathPoints[FitEnd - 1].curvature;
      }
      break;
    }
    //���� ·�� ����
    // Vector3d vecA, vecB;
    sPoint vecA, vecB, vecC;
    /*double angleAB = 0.0;
    // calculate the vec A and B
    vecA.x = samplepoint[0].x - samplepoint[1].x;
    vecA.y = samplepoint[0].y - samplepoint[1].y;
    vecB.x = samplepoint[2].x - samplepoint[1].x;
    vecB.y = samplepoint[2].y - samplepoint[1].y;
    */
    vecA = SamplePoint[0];
    vecB = SamplePoint[1];
    vecC = SamplePoint[2];
    double c = sqrt((vecA.x - vecB.x) * (vecA.x - vecB.x) +
                    (vecA.y - vecB.y) * (vecA.y - vecB.y));
    double b = sqrt((vecA.x - vecC.x) * (vecA.x - vecC.x) +
                    (vecA.y - vecC.y) * (vecA.y - vecC.y));
    double a = sqrt((vecC.x - vecB.x) * (vecC.x - vecB.x) +
                    (vecC.y - vecB.y) * (vecC.y - vecB.y));

    PathPoints[i].curvature = fabs((vecB.x - vecA.x) * (vecB.y - vecC.y) -
                                   (vecB.y - vecA.y) * (vecB.x - vecC.x)) *
                              2 / a / b / c;
    // PathPoints[i].curvature = Curve;
  }

  if (FitStart == FitEnd) {
    if (PathPoints[FitStart].curvature < 0)
      PathPoints[FitStart].curvature = CurveEnd;
  }

  for (i = 0; i < FitStart; ++i) {
    if (PathPoints[i].curvature < 0) {
      for (int j = i; j <= FitStart; ++j) {
        if (PathPoints[j].curvature >= 0) {
          PathPoints[i].curvature = PathPoints[j].curvature;
          break;
        }
      }
    }
  }
}

}  // namespace geditor
