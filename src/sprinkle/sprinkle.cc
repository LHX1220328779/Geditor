
#include "sprinkle/sprinkle.h"

namespace geditor {
namespace sprinkle {

Sprinkle::Sprinkle(std::vector<Polygon> &Input, SprinkleConf &Conf,
                   std::vector<Point2D> &SprinklePoints,
                   std::vector<std::pair<int, int>> &Tprelationship) {
  SpParm = Conf;
  // Whether the polygon vertex is empty
  if (Input.empty()) {
    Conf.CompleteStatus = false;
    Conf.ErrorCode = -1;
    return;
  }

  for (int i = 0; i < Input.size(); ++i) {
    // Whether polygon vertices can form a valid polygon
    if (3 > Input[i].PolyginSize) {
      Conf.CompleteStatus = false;
      Conf.ErrorCode = -1;
      return;
    }
    // Initialization boundary value
    // std::cout << Input[0].Vertex[8].x << " " << Input[1].Vertex[8].x <<
    // std::endl;
    PyParam.Up = Input[i].Vertex[0].y;
    PyParam.Down = Input[i].Vertex[0].y;
    PyParam.Left = Input[i].Vertex[0].x;
    PyParam.Right = Input[i].Vertex[0].x;
    // Get the boundary coordinate value
    for (std::vector<Point2D>::iterator ai = Input[i].Vertex.begin();
         ai != Input[i].Vertex.end(); ++ai) {
      if (PyParam.Up < (*ai).y) PyParam.Up = (*ai).y;
      if (PyParam.Down > (*ai).y) PyParam.Down = (*ai).y;
      if (PyParam.Right < (*ai).x) PyParam.Right = (*ai).x;
      if (PyParam.Left > (*ai).x) PyParam.Left = (*ai).x;
    }
    PyParam.Height = PyParam.Up - PyParam.Down;
    PyParam.Width = PyParam.Right - PyParam.Left;
    // std::cout <<  PyParam.Height << " " << PyParam.Width << std::endl;
    GridSprinkle(SprinklePoints, Input[i].Vertex, i);
  }

  if (SprinklePoints.size() == 0) {
    Conf.CompleteStatus = false;
    Conf.ErrorCode = -2;
    return;
  }

  SpParm.ErrorCode = GetTopological(SprinklePoints, Input, Tprelationship);

  SpParm.CompleteStatus = true;
  Conf = SpParm;
  return;
}

Sprinkle::~Sprinkle() {}

void Sprinkle::GridSprinkle(std::vector<Point2D> &SprinklePoints,
                            std::vector<Point2D> PolyonPoints, int Zone) {
  Point2D Point;
  Point2D PointNo;
  SprinkleData TempData;
  TempData.PolygonLabel = Zone;
  TempData.StartLabel = SprinklePoints.size();
  std::random_device rd;
  float RANGR = (float)SpParm.Range;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> g_rand(0, SpParm.Range);
  int GridRows = floor(PyParam.Height / SpParm.CellSize);
  int GridCols = floor(PyParam.Width / SpParm.CellSize);
  for (int i = 0; i < GridRows; ++i) {
    for (int j = 0; j < GridCols; ++j) {
      for (int n = 0; n < SpParm.PointCounts; ++n) {
        Point.x = PyParam.Left +
                  (j + 0.25 + 0.5 * (g_rand(gen) / RANGR)) * SpParm.CellSize;
        Point.y = PyParam.Down +
                  (i + 0.25 + 0.5 * (g_rand(gen) / RANGR)) * SpParm.CellSize;
        // std::cout << Point.x << " " << Point.y << " " << SpParm.CellSize <<
        // std::endl;
        if (0 == Zone) {
          if (0x02 == IsInPolygon(PolyonPoints, Point)) {
            Point.Label = SprinklePoints.size();
            Point.Zone = Zone;
            SprinklePoints.push_back(Point);
          }
        } else {
          if ((!RemovePointInOverlapArea(SprinklePoints, Point)) &&
              (0x02 == IsInPolygon(PolyonPoints, Point))) {
            Point.Label = SprinklePoints.size();
            Point.Zone = Zone;
            SprinklePoints.push_back(Point);
          }
        }
      }
    }
  }
  TempData.EndLabel = SprinklePoints.size();
  if (TempData.EndLabel == TempData.StartLabel) {
    SpParm.ErrorCode = 1;
    TempData.IsEmpty = true;
  } else {
    TempData.EndLabel = TempData.EndLabel - 1;
    SpParm.ErrorCode = 0;
    TempData.IsEmpty = false;
  }
  SprinklePointsOfSize.push_back(TempData);
  // std::cout << TempData.StartLabel << " " << TempData.EndLabel << " " <<
  // TempData.PolygonLabel << std::endl;
}

// Remove excess points in the overlap area
bool Sprinkle::RemovePointInOverlapArea(std::vector<Point2D> &SprinklePoints,
                                        Point2D Point) {
  for (std::vector<Point2D>::iterator i = SprinklePoints.begin();
       i != SprinklePoints.end(); ++i) {
    if (Distance(*i, Point) <= SpParm.Distance) return true;
  }
  return false;
}

int Sprinkle::GetTopological(std::vector<Point2D> &SprinklePoints,
                             std::vector<Polygon> &Input,
                             std::vector<std::pair<int, int>> &Output) {
  // start k-d tree
  KdTree *KdTree_ = new KdTree;
  CreateKdTree(KdTree_, SprinklePoints, 0);
  // PrintKdTree(KdTree_, 0);
  // KNN
  for (std::vector<SprinkleData>::iterator v = SprinklePointsOfSize.begin();
       v != SprinklePointsOfSize.end(); ++v) {
    // std::cout << "Label : " << (*v).PolygonLabel << std::endl;
    for (int i = (*v).StartLabel; i < (*v).EndLabel; ++i) {
      // std::cout << "points_label : " << i << std::endl;
      std::vector<int> TempPoint;
      std::vector<int> TempTP;
      TempPoint = SearchNearestNeighborBBF(
          KdTree_, SprinklePoints[i], SpParm.MaxDistance, SpParm.MinDistance);
      // std::cout << SprinklePoints[i].x << " " << SprinklePoints[i].y << " "
      // << SprinklePoints[i].Label << std::endl; std::cout << TempPoint.size()
      // << std::endl;
      // std::pair<std::map<int, std::vector<int>>::iterator, bool> ret;
      // ret = Output.insert(std::pair<int, std::vector<int>>(tp_no,
      // TempPoint)); if (ret.second == false) {
      //     std::cout << "element " << tp_no << " already existed" <<
      //     std::endl; return 0x03;
      //  }
      //  tp_no++;
      if (0 == TempPoint.size()) continue;
      for (std::vector<int>::iterator Q = TempPoint.begin();
           Q != TempPoint.end(); ++Q) {
        if (!IsLinePolyIntersect(SprinklePoints[i], SprinklePoints[*Q],
                                 Input)) {
          TempTP.push_back(*Q);
          // std::cout << *Q << std::endl;
        }
      }
      for (std::vector<int>::iterator P = TempTP.begin(); P != TempTP.end();
           ++P) {
        std::pair<int, int> TP;
        int TPFlag = false;
        for (std::vector<std::pair<int, int>>::iterator G = Output.begin();
             G != Output.end(); ++G) {
          if ((*G).first == *P && (*G).second == SprinklePoints[i].Label) {
            TPFlag = true;
            break;
          }
        }
        if (!TPFlag) {
          TP = std::make_pair(SprinklePoints[i].Label, *P);
          Output.push_back(TP);
        }
      }
      std::vector<int>().swap(TempPoint);
      std::vector<int>().swap(TempTP);
    }
  }
  KdTreeRelease(KdTree_);
  return 0;
}

int Sprinkle::IsInPolygon(std::vector<Point2D> PolyonPoints, Point2D Point) {
  // 0x00 : outside   0x01 : Point on the polygon  0x02 : inside
  // The ray slope is 0
  bool flag = false;
  int i, j;

  for (i = 0, j = PolyonPoints.size() - 1; i < PolyonPoints.size(); j = i++) {
    int ix = PolyonPoints[i].x;
    int iy = PolyonPoints[i].y;
    int jx = PolyonPoints[j].x;
    int jy = PolyonPoints[j].y;
    float re = PointToSegDist(PolyonPoints[i], PolyonPoints[j], Point);
    if (re <= SpParm.DisBoundary) return 0x01;
    // Point coincides with polygon edge
    if ((ix == Point.x && iy == Point.y) || (jx == Point.x && jy == Point.y)) {
      return 0x01;
    }
    // Determine if both ends of the line segment are on both sides of the ray
    if ((iy < Point.y && jy >= Point.y) || (iy >= Point.y && jy < Point.y)) {
      // The x coordinate of the point on the line segment that is the same as
      // the ray y coordinate
      int TempX = ix + (Point.y - iy) * (jx - ix) / (jy - iy);
      // Point on the polygon
      if (TempX == Point.x) {
        return 0x01;
      }
      // Ray through polygon
      if (TempX > Point.x) {
        flag = !flag;
      }
    }
  }
  // Return result
  return flag ? 0x02 : 0x00;
}

bool Sprinkle::IsLinePolyIntersect(Point2D StartPoint, Point2D EndPoint,
                                   std::vector<Polygon> InputPolyon) {
  double DeltaX = EndPoint.x - StartPoint.x;
  double DeltaY = EndPoint.y - StartPoint.y;
  double m;
  Point2D Temp;
  bool SameFlag = (StartPoint.Zone == EndPoint.Zone);
  if (fabs(DeltaX) > fabs(DeltaY))
    m = DeltaX;
  else
    m = DeltaY;
  double Step = 0.5 * fabs(1 / m);
  // std::cout << step << std::endl;
  for (double Percent = 0.0; Percent <= 1.0; Percent += Step) {
    Temp.x = DeltaX * Percent + StartPoint.x;
    Temp.y = DeltaY * Percent + StartPoint.y;
    if (SameFlag) {
      if (0x00 == IsInPolygon(InputPolyon[StartPoint.Zone].Vertex, Temp))
        return true;
    } else {
      if (0x00 == IsInPolygon(InputPolyon[StartPoint.Zone].Vertex, Temp) &&
          0x00 == IsInPolygon(InputPolyon[StartPoint.Zone].Vertex, Temp))
        return true;
    }
  }
  return false;
}

// Egde: P1 and P2;  point: Pt
float Sprinkle::PointToSegDist(const Point2D &P1, const Point2D &P2,
                               const Point2D &Pt) {
  float FDot = (P2.x - P1.x) * (Pt.x - P1.x) + (P2.y - P1.y) * (Pt.y - P1.y);
  if (FDot <= 0.0f) {
    return sqrt((P1.x - Pt.x) * (P1.x - Pt.x) + (P1.y - Pt.y) * (P1.y - Pt.y));
  }
  float D2AB = (P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y);
  if (FDot >= D2AB) {
    return sqrt((P2.x - Pt.x) * (P2.x - Pt.x) + (P2.y - Pt.y) * (P2.y - Pt.y));
  }

  float U = FDot / D2AB;
  float AC_x = P1.x + (P2.x - P1.x) * U;
  float AC_y = P1.y + (P2.y - P1.y) * U;
  return sqrt((Pt.x - AC_x) * (Pt.x - AC_x) + (Pt.y - AC_y) * (Pt.y - AC_y));
}

double Sprinkle::Distance(Point2D PA, Point2D PB) {
  return sqrt(pow(PA.x - PB.x, 2) + pow(PA.y - PB.y, 2));
}
}  // namespace sprinkle
}  // namespace geditor
