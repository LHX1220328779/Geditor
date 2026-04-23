
#include "core/bline.h"

#include <cmath>
#include <vector>

namespace geditor {

double GetN0(double t) {
  //(a - b)^3 = a^3 - 3a^2b + 3ab^2 - b^3;
  double dValue = 1.0 / 6 * (1 - 3 * t + 3 * t * t - t * t * t);
  return dValue;
}

double GetN1(double t) {
  double dValue = (1.0 / 6) * (4 - 6 * t * t + 3 * t * t * t);
  return dValue;
}

double GetN2(double t) {
  double dValue = (1.0 / 6) * (1 + 3 * t + 3 * t * t - 3 * t * t * t);
  return dValue;
}

double GetN3(double t) {
  double dValue = 1.0 / 6 * (t * t * t);
  return dValue;
}

void SamplePoint(std::vector<Point3d> &linePoint, const Point3d &p0,
                 const Point3d &p1, const Point3d &p2, const Point3d &p3,
                 double d) {
  double dis = Point3d::Distance(p2, p1);
  int nPointCount = dis / d + 1;
  if (nPointCount < 2) nPointCount = 2;
  for (int j = 0; j <= nPointCount; j++) {
    double t = 1.0 * j / nPointCount;

    double xPt =
        GetN0(t) * p0.x + GetN1(t) * p1.x + GetN2(t) * p2.x + GetN3(t) * p3.x;
    double yPt =
        GetN0(t) * p0.y + GetN1(t) * p1.y + GetN2(t) * p2.y + GetN3(t) * p3.y;

    Point3d pt;
    pt.x = xPt;
    pt.y = yPt;
    pt.z = 0;
    if (j == 0 || j == nPointCount) pt.z = 0.001;
    if (linePoint.size() > 0 && Point3d::Distance(pt, linePoint.back()) < 0.01)
      continue;
    linePoint.push_back(pt);
  }
}

void B3Line(const std::vector<Point3d> &controlPoint,
            std::vector<Point3d> &linePoint, double d) {
  //	���ۿ��Ƶ���
  int nPt = controlPoint.size();

  //	���ݼ�����������ۿ��Ƶ㣬��������B��������
  linePoint.clear();
  for (int i = 1; i < nPt - 3; i++) {
    SamplePoint(linePoint, controlPoint[i], controlPoint[i + 1],
                controlPoint[i + 2], controlPoint[i + 3], d);
  }
}

int IsOnLine(const std::vector<Point3d> &controlPoint,
             std::vector<Point3d> &mousePoint, const Point3d &pt, int &nIndex,
             bool bInsert) {
  int nPointCount = 100;
  const double Distance = 5.0;
  const size_t nMousePointCount = mousePoint.size();
  if (nMousePointCount < 2) return 0;
  std::vector<int> list;  // �������������ܵĶ�

  for (int m = 0; m < nMousePointCount; m++) {
    Point3d pt1 = mousePoint[m];

    double x = pt1.x;
    double y = pt1.y;
    double xx = pt.x;
    double yy = pt.y;

    if (sqrt((x - xx) * (x - xx) + (y - yy) * (y - yy)) < Distance) {
      nIndex = m;
      return 1;
    }
  }

  for (int i = 1; i < nMousePointCount; i++) {
    Point3d lt = mousePoint[i - 1];
    Point3d rb = mousePoint[i];

    Rect rect;
    if (lt.x < rb.x) {
      rect.left = lt.x;
      rect.top = lt.y;
      rect.bottom = rb.y;
      rect.right = rb.x;
    } else {
      rect.top = rb.y;
      rect.left = rb.x;
      rect.bottom = lt.y;
      rect.right = lt.x;
    }
    list.push_back(i);
  }

  if (list.size() <= 0) return 0;
  return 0;
}

int IsOnLine(std::vector<Point3d> &controlPoint,
             std::vector<Point3d> &mousePoint, const Point3d &pt, int &nIndex,
             bool bInsert) {
  int nPointCount = 100;
  const double Distance = 5.0;
  //	�鿴��������һ������,
  //����ж�����ܵĶΣ�Ҫ�����������һ�ε��ж�
  const size_t nMousePointCount = mousePoint.size();
  if (nMousePointCount < 2) return 0;
  std::vector<int> list;  // �������������ܵĶ�

  for (int m = 0; m < nMousePointCount; m++) {
    Point3d pt1 = mousePoint[m];

    double x = pt1.x;
    double y = pt1.y;
    double xx = pt.x;
    double yy = pt.y;

    if (sqrt((x - xx) * (x - xx) + (y - yy) * (y - yy)) < Distance) {
      nIndex = m;
      return 1;
    }
  }

  for (int i = 0; i < nMousePointCount - 1; i++) {
    Point3d lt = mousePoint[i];
    Point3d rb = mousePoint[i + 1];

    Rect rect;
    if (lt.x < rb.x) {
      rect.left = lt.x;
      rect.top = lt.y;
      rect.bottom = rb.y;
      rect.right = rb.x;
    } else {
      rect.top = rb.y;
      rect.left = rb.x;
      rect.bottom = lt.y;
      rect.right = lt.x;
    }
    list.push_back(i);
  }

  if (list.size() <= 0) return 0;

  const size_t nListCount = list.size();
  for (int i = 0; i < nListCount; i++) {
    int nIndex1 = list[i];
    double x[4] = {0.0};
    double y[4] = {0.0};

    for (int k = nIndex1, j = 0; k < nIndex1 + 4; k++, j++) {
      Point3d pt1 = controlPoint[k];
      x[j] = pt1.x;
      y[j] = pt1.y;
    }

    Point3d pt1 = mousePoint[i];
    Point3d pt2 = mousePoint[i + 1];
    double xlong = fabs((double)(pt1.x - pt2.x));
    double ylong = fabs((double)(pt1.y - pt2.y));

    if (xlong > ylong)
      nPointCount = 2 * xlong;
    else
      nPointCount = 2 * ylong;

    for (int j = 0; j <= nPointCount; j++) {
      double t = 1.0 * j / nPointCount;
      double t2 = t * t;
      double t3 = t * t * t;

      double xPt = 1.0 / 6 * (1 - 3 * t + 3 * t2 - t3) * x[0] +
                   1.0 / 6 * (4 - 6 * t2 + 3 * t3) * x[1] +
                   1.0 / 6 * (1 + 3 * t + 3 * t2 - 3 * t3) * x[2] +
                   1.0 / 6 * t3 * x[3];
      double yPt = 1.0 / 6 * (1 - 3 * t + 3 * t2 - t3) * y[0] +
                   1.0 / 6 * (4 - 6 * t2 + 3 * t3) * y[1] +
                   1.0 / 6 * (1 + 3 * t + 3 * t2 - 3 * t3) * y[2] +
                   1.0 / 6 * t3 * y[3];

      double xx = pt.x;
      double yy = pt.y;

      double dDistance =
          sqrt((xPt - xx) * (xPt - xx) + (yPt - yy) * (yPt - yy));
      if (dDistance <= Distance) {
        // nIndex = nIndex;
        if (bInsert) {
          //	����һ���µĿ��ƵĲ��붯��
          std::vector<Point3d> temp = mousePoint;
          int nCount = temp.size();
          mousePoint.clear();

          for (int m = 0, n = 0; m < nCount; m++, n++) {
            if (m == nIndex1) {
              mousePoint.push_back(pt);
              n++;
            }

            Point3d p = temp[m];
            mousePoint.push_back(p);
          }

          nIndex = nIndex1;
          return 1;
        } else {
          nIndex = -1;
          return -1;
        }
      }
    }
  }

  return 0;
}

bool EditMousePoint(std::vector<Point3d> &mousePoint, int nIndex,
                    const Point3d &movePt) {
  int n = mousePoint.size();
  if (nIndex < n) {
    mousePoint[nIndex] = movePt;
    return true;
  }

  return false;
}

std::vector<Point3d> FindXY(const std::vector<Point3d> &controlPoint,
                            const std::vector<Point3d> &mousePoint, int nXY,
                            bool bIsX) {
  const double Distance = 1.5;
  const size_t nPt = controlPoint.size();
  int nPointCount = 100;

  std::vector<Point3d> result;
  for (int i = 2; i <= nPt - 3; i++) {
    Point3d pt1 = mousePoint[i - 2];
    Point3d pt2 = mousePoint[i - 1];
    double xlong = fabs((double)(pt1.x - pt2.x));
    double ylong = fabs((double)(pt1.y - pt2.y));

    if (xlong > ylong)
      nPointCount = 2 * xlong;
    else
      nPointCount = 2 * ylong;

    for (int j = 0; j <= nPointCount; j++) {
      double t = 1.0 * j / nPointCount;
      double t2 = t * t;
      double t3 = t * t * t;

      double xPt =
          1.0 / 6 * (1 - 3 * t + 3 * t2 - t3) * controlPoint[i - 1].x +
          1.0 / 6 * (4 - 6 * t2 + 3 * t3) * controlPoint[i].x +
          1.0 / 6 * (1 + 3 * t + 3 * t2 - 3 * t3) * controlPoint[i + 1].x +
          1.0 / 6 * t3 * controlPoint[i + 2].x;
      double yPt =
          1.0 / 6 * (1 - 3 * t + 3 * t2 - t3) * controlPoint[i - 1].y +
          1.0 / 6 * (4 - 6 * t2 + 3 * t3) * controlPoint[i].y +
          1.0 / 6 * (1 + 3 * t + 3 * t2 - 3 * t3) * controlPoint[i + 1].y +
          1.0 / 6 * t3 * controlPoint[i + 2].y;
      bool bFind = false;
      if (bIsX) {
        if (fabs(nXY - xPt) < Distance) {
          xPt = nXY;
          bFind = true;
        }
      } else {
        if (fabs(nXY - yPt) < Distance) {
          yPt = nXY;
          bFind = true;
        }
      }

      if (bFind) {
        Point3d p;
        p.x = xPt;
        p.y = yPt;
        result.push_back(p);
        break;
      }
    }
  }

  return result;
}

}  // namespace geditor
