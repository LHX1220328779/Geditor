
#include "core/car_body_check.h"
#include "core/compute_curve.h"
#include "core/point3d.h"

namespace geditor {

std::vector<Point3d> CarBodyCheck::GetAABBPoints(const Point3d &pt,
                                                 const Point3d &dir) {
  // 1,p����복����ǰ�˵ľ��룺1.15��
  // 2,P����복�����εľ��룺0.3��
  // 3,�복��0.45��

  //
  //  ��С�׳�ͨ�������·��ʱ��ĳ���ģ��
  //
  //	 	                             pPointA             pPointB
  //    pPointA             pPointB       --------------------
  //	    --------------------            |         |        |
  //	    |         |        |		    |         |        |
  //	    |         |        |		    |         |        |
  //	    |		  |	       |		    |    1.35 |	       |
  //	    |	  1.15|        |		    |	      |        |
  //	    |		  |  	   |		    |		  |  	   |
  //	    |		  | 	   |		    |		  | 	   |
  //	    |	0.51  |	p2     |		    |	0.51  |	p2     |
  //	    |---------*--------|		    |---------*--------|
  //	    |		  |	       |		    |		  |
  //| 	    |	   0.3|	       |		    |	   0.3|	       |
  //      |         |        |		    |         |        |
  //	    --------------------		    --------------------
  //	 pPointC            pPointD		 pPointC            pPointD
  //            ��ͨ����                      SN100����

  double L1 = 1.35;
  // double L1 = 1.15;
  double L2 = 0.30;
  double L3 = 0.51;

  std::vector<Point3d> BoundingBoxPoints;

  double xx = dir.x;
  double yy = dir.y;

  double length = sqrt(xx * xx + yy * yy);

  double dirx = xx / length;
  double diry = yy / length;

  double P1X = pt.x + dirx * L1;
  double P1Y = pt.y + diry * L1;

  double P2X = pt.x - dirx * L2;
  double P2Y = pt.y - diry * L2;

  Point3d pointA;
  Point3d pointB;
  Point3d pointC;
  Point3d pointD;

  pointA.x = P1X - diry * L3;
  pointA.y = P1Y + dirx * L3;

  pointB.x = P1X + diry * L3;
  pointB.y = P1Y - dirx * L3;

  pointC.x = P2X - diry * L3;
  pointC.y = P2Y + dirx * L3;

  pointD.x = P2X + diry * L3;
  pointD.y = P2Y - dirx * L3;

  BoundingBoxPoints.push_back(pointA);
  BoundingBoxPoints.push_back(pointB);
  BoundingBoxPoints.push_back(pointD);
  BoundingBoxPoints.push_back(pointC);

  return BoundingBoxPoints;
}

double CarBodyCheck::GetCurvatureError(double curvature) {
  const double rd[] = {200, 50, 10, 5, 2, 1.5};
  // const double er[] = { 0.05, 0.10, 0.15, 0.20, 0.25, 0.35 };
  const double er[] = {0.05 + 0.0024, 0.10 + 0.0071, 0.15 + 0.0246,
                       0.20 + 0.0414, 0.25 + 0.0587, 0.35 + 0.0717};
  const double locer = 0.15;

  //--------------------------
  double raidus = 0.0;

  //��ֹ��0����
  if (curvature < 1.0 / rd[0]) {
    return er[0] + locer;
  } else {
    raidus = 1.0 / curvature;
  }

  //�������
  double error = 0.0;
  if (raidus > rd[0]) {
    error = 0.05;
  } else if (raidus > rd[1]) {
    error = er[0] + (raidus - rd[0]) * (er[0] - er[1]) / (rd[0] - rd[1]);
  } else if (raidus > rd[2]) {
    error = er[1] + (raidus - rd[1]) * (er[1] - er[2]) / (rd[1] - rd[2]);
  } else if (raidus > rd[3]) {
    error = er[2] + (raidus - rd[2]) * (er[2] - er[3]) / (rd[2] - rd[3]);
  } else if (raidus > rd[4]) {
    error = er[3] + (raidus - rd[3]) * (er[3] - er[4]) / (rd[3] - rd[4]);
  } else if (raidus > rd[5]) {
    error = er[4] + (raidus - rd[4]) * (er[4] - er[5]) / (rd[4] - rd[5]);
  } else {
    error = er[5];
  }

  return error + locer;
}

void CarBodyCheck::OffsetRoadPath(const std::vector<sPoint> &items,
                                  std::vector<V3d> &outArray0,
                                  std::vector<V3d> &outArray1) {
  //-------------------------------------------
  std::vector<V3d> positon;
  std::vector<V3d> normal;

  const double deltAngle = 15.0f * 0.01745329251994329576923690768489;

  //�����߶η���
  int iCount = items.size();
  for (int i = 0; i < iCount; i++) {
    V3d orginPt(items[i].x, items[i].y, 0.0);
    double v_width = GetCurvatureError(items[i].curvature);
    //------------------------------------
    V3d vUp(0, 0, 1);
    if (i == 0) {
      V3d vRight;

      V3d vDir;
      vDir[0] = items[i + 1].x - items[i].x;
      vDir[1] = items[i + 1].y - items[i].y;
      vDir[2] = 0;

      V3d vCross = vDir.cross(vUp);

      vRight = Normalize(vCross);
      vRight[2] = 1;

      outArray0.push_back(orginPt + vRight * v_width);
      outArray1.push_back(orginPt - vRight * v_width);
    } else if (i == items.size() - 1) {
      V3d vRight;
      V3d vDir;
      vDir[0] = items[i].x - items[i - 1].x;
      vDir[1] = items[i].y - items[i - 1].y;
      vDir[2] = 0;

      V3d vCross = vDir.cross(vUp);
      vRight = Normalize(vCross);
      int sz = positon.size();

      vRight[2] = 1;
      positon.push_back(orginPt);
      positon.push_back(orginPt);
      normal.push_back(vRight);
      normal.push_back(-vRight);

      outArray0.push_back(orginPt + vRight * v_width);
      outArray1.push_back(orginPt - vRight * v_width);
    } else {
      V3d vDir1;
      vDir1[0] = items[i + 1].x - items[i].x;
      vDir1[1] = items[i + 1].y - items[i].y;
      vDir1[2] = 0;

      V3d vDir2;
      vDir2[0] = items[i].x - items[i - 1].x;
      vDir2[1] = items[i].y - items[i - 1].y;
      vDir2[2] = 0;

      V3d temp1 = vDir1.cross(vUp);
      temp1.normalize();

      V3d temp2 = vDir2.cross(vUp);
      temp2.normalize();

      V3d jonin_normal = (temp1 + temp2);
      jonin_normal.normalize();

      //----------------------------------------------
      V3d v_temp_norma = Normalize(vDir2);
      float dot_val = v_temp_norma.dot(jonin_normal);

      double angle = acos(abs(dot_val));
      if (angle < deltAngle) {
        LOG(INFO) << "fuck!";
      } else {
        double tmpWidth = 1.0 / sin(angle);
        int sz = positon.size();

        jonin_normal = jonin_normal * tmpWidth;
        jonin_normal[2] = 1;

        outArray0.push_back(orginPt + jonin_normal * v_width);
        outArray1.push_back(orginPt - jonin_normal * v_width);

        positon.push_back(orginPt);
        positon.push_back(orginPt);
        normal.push_back(jonin_normal);
        normal.push_back(-jonin_normal);
      }
    }
  }
}

// outArray0 ************** right line
// outArray1 ************** line

void CarBodyCheck::WriteOffPath(const std::vector<sPoint> &pathPoints,
                                std::vector<Point3d> &vPointSetItem,
                                const std::vector<V3d> &outArray0,
                                const std::vector<V3d> &outArray1) {
  //���������ڼ��
  FILE *poffsetfile = fopen("E:\\poffsetfile.txt", "wb");
  if (poffsetfile) {
    fprintf(poffsetfile, "MULTILINESTRING((");
    int nPathCount = pathPoints.size();
    for (int j = 0; j < nPathCount; j++) {
      sPoint pnt = pathPoints[j];
      if (j < pathPoints.size() - 1) {
        fprintf(poffsetfile, "%f %f,", pnt.x, pnt.y);
      } else {
        fprintf(poffsetfile, "%f %f", pnt.x, pnt.y);
      }
    }

    fprintf(poffsetfile, "),");

    fprintf(poffsetfile, "(");
    int nOutCountO = outArray0.size();
    for (int j = 0; j < nOutCountO; j++) {
      V3d pnt = outArray0[j];

      if (j < outArray0.size() - 1) {
        fprintf(poffsetfile, "%f %f,", pnt[0], pnt[1]);
      } else {
        fprintf(poffsetfile, "%f %f", pnt[0], pnt[1]);
      }
    }
    fprintf(poffsetfile, "),");

    fprintf(poffsetfile, "(");
    int nOutCount1 = outArray1.size();
    for (int j = 0; j < nOutCount1; j++) {
      V3d pnt = outArray1[j];

      if (j < outArray1.size() - 1) {
        fprintf(poffsetfile, "%f %f,", pnt[0], pnt[1]);
      } else {
        fprintf(poffsetfile, "%f %f", pnt[0], pnt[1]);
      }
    }
    fprintf(poffsetfile, "))\r\n");

    fclose(poffsetfile);
  }

  //======================================
  //��ƫ�ƺ�ĳ���ģ�ͽ��м��
  //======================================

  FILE *poffsetcarfile = fopen("E:\\poffsetcarfile.txt", "wb");
  if (poffsetcarfile) {
    for (int j = 0; j < outArray1.size(); j++) {
      if (j + 1 == outArray1.size()) {
        break;
      }

      double dirX = vPointSetItem[j + 1].x - vPointSetItem[j].x;
      double dirY = vPointSetItem[j + 1].y - vPointSetItem[j].y;

      Point3d pointX = Point3d(outArray1[j][0], outArray1[j][1], 0.0);

      std::vector<Point3d> mapCornerPoints =
          GetAABBPoints(pointX, Point3d(dirX, dirY, 0.0));

      fprintf(poffsetcarfile, "LineString(");
      for (int i = 0; i < 4; i++) {
        Point3d pointXY = mapCornerPoints[i];
        fprintf(poffsetcarfile, "%f %f,", pointXY.x, pointXY.y);
      }
      Point3d pointXY = mapCornerPoints[0];
      fprintf(poffsetcarfile, "%f %f)\r\n", pointXY.x, pointXY.y);
    }

    for (int j = 0; j < outArray0.size(); j++) {
      if (j + 1 == outArray0.size()) {
        break;
      }

      double dirX = vPointSetItem[j + 1].x - vPointSetItem[j].x;
      double dirY = vPointSetItem[j + 1].y - vPointSetItem[j].y;

      Point3d pointX = Point3d(outArray0[j][0], outArray0[j][1], 0.0);

      std::vector<Point3d> mapCornerPoints =
          GetAABBPoints(pointX, Point3d(dirX, dirY, 0.0));

      fprintf(poffsetcarfile, "LineString(");
      for (int i = 0; i < 4; i++) {
        Point3d pointXY = mapCornerPoints[i];
        fprintf(poffsetcarfile, "%f %f,", pointXY.x, pointXY.y);
      }
      Point3d pointXY = mapCornerPoints[0];
      fprintf(poffsetcarfile, "%f %f)\r\n", pointXY.x, pointXY.y);
    }
    fclose(poffsetcarfile);
  }

  FILE *pTestfile = fopen("E:\\test.txt", "wb");
  if (pTestfile) {
    // fprintf(poffsetcarfile, "LineString(");

    for (int j = 0; j < outArray1.size(); j++) {
      if (j + 1 == outArray1.size()) {
        break;
      }

      double dirX = vPointSetItem[j + 1].x - vPointSetItem[j].x;
      double dirY = vPointSetItem[j + 1].y - vPointSetItem[j].y;

      Point3d pointX = Point3d(outArray1[j][0], outArray1[j][1], 0.0);

      std::vector<Point3d> mapCornerPoints =
          GetAABBPoints(pointX, Point3d(dirX, dirY, 0.0));

      Point3d pointXY = mapCornerPoints[0];

      if (j + 1 < outArray1.size()) {
        //	fprintf(poffsetcarfile, "%f %f,", pointXY.x, pointXY.y);
      } else {
        // fprintf(poffsetcarfile, "%f %f)\r\n", pointXY.x, pointXY.y);
      }
    }

    //======================================================

    std::vector<Point3d> vRight0;
    std::vector<Point3d> vRight1;
    std::vector<Point3d> vLeft0;
    std::vector<Point3d> vLeft1;

    for (int j = 0; j < outArray0.size() - 1; j++) {
      //���㷽��
      double dirX = vPointSetItem[j + 1].x - vPointSetItem[j].x;
      double dirY = vPointSetItem[j + 1].y - vPointSetItem[j].y;

      //λ��
      Point3d pointX = Point3d(outArray0[j][0], outArray0[j][1], 0.0);

      //���㳵���ĸ���
      std::vector<Point3d> mapCornerPoints =
          GetAABBPoints(pointX, Point3d(dirX, dirY, 0.0));

      {
        vRight1.push_back(mapCornerPoints[1]);  //���Ͻ�
        vRight0.push_back(mapCornerPoints[2]);  //���½�

        vLeft0.push_back(mapCornerPoints[0]);  //���Ͻ�
        vLeft1.push_back(mapCornerPoints[1]);  //���½�
      }
    }

    fprintf(poffsetcarfile, "LineString(");
    for (int i = 0; i < vRight0.size(); i++) {
      if (i < vRight0.size() - 1) {
        fprintf(poffsetcarfile, "%f %f,", vRight0[i].x, vRight0[i].y);
      } else {
        fprintf(poffsetcarfile, "%f %f)\r\n", vRight0[i].x, vRight0[i].y);
      }
    }

    fprintf(poffsetcarfile, "LineString(");
    for (int i = 0; i < vRight1.size(); i++) {
      if (i < vRight1.size() - 1) {
        fprintf(poffsetcarfile, "%f %f,", vRight1[i].x, vRight1[i].y);
      } else {
        fprintf(poffsetcarfile, "%f %f)\r\n", vRight1[i].x, vRight1[i].y);
      }
    }

    fprintf(poffsetcarfile, "LineString(");
    for (int i = 0; i < vLeft0.size(); i++) {
      if (i < vLeft0.size() - 1) {
        fprintf(poffsetcarfile, "%f %f,", vLeft0[i].x, vLeft0[i].y);
      } else {
        fprintf(poffsetcarfile, "%f %f)\r\n", vLeft0[i].x, vLeft0[i].y);
      }
    }

    fprintf(poffsetcarfile, "LineString(");
    for (int i = 0; i < vLeft1.size(); i++) {
      if (i < vLeft1.size() - 1) {
        fprintf(poffsetcarfile, "%f %f,", vLeft1[i].x, vLeft1[i].y);
      } else {
        fprintf(poffsetcarfile, "%f %f)\r\n", vLeft1[i].x, vLeft1[i].y);
      }
    }

    fclose(pTestfile);
  }
}

}  // namespace geditor
