
#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#define ull_int unsigned long long

namespace geditor {
namespace sprinkle {

struct Point2D {
  double x;
  double y;
  int Label;
  int Zone;

  Point2D() {}

  Point2D(double xx, double yy) : x(xx), y(yy) {}
};

struct Polygon {
  int PolyginSize;
  std::vector<Point2D> Vertex;
};

struct PolygonParm {
  double Up;
  double Down;
  double Left;
  double Right;
  double Height;
  double Width;
};

struct SprinkleData {
  int PolygonLabel;
  int StartLabel;
  int EndLabel;
  bool IsEmpty;
};

struct SprinkleConf {
  int Proportion;
  int PointCounts;
  double Fproportion;
  double Range;
  double DisBoundary;
  double Distance;
  double CellSize;
  bool CompleteStatus;
  int ErrorCode;
  double MaxDistance;
  double MinDistance;
};

struct KdTree {
 public:
  Point2D Root;
  int SortDim;
  KdTree *Parent;
  KdTree *LeftChild;
  KdTree *RightChild;
  bool CreateFlag;

  // API
  KdTree() {
    Parent = LeftChild = RightChild = NULL;
    CreateFlag = false;
  }

  ~KdTree(){};

  bool IsEmpty() { return CreateFlag == false; }

  bool IsLeaf() {
    return CreateFlag == true && RightChild == NULL && LeftChild == NULL;
  }

  bool IsRoot() { return CreateFlag == true && Parent == NULL; }

  bool IsLeft() {
    return Parent->LeftChild->Root.x == Root.x &&
           Parent->LeftChild->Root.y == Root.y;
  }

  bool IsRight() {
    return Parent->RightChild->Root.x == Root.x &&
           Parent->RightChild->Root.y == Root.y;
  }
  // void CreateKdTree(KdTree* Tree, std::vector<Point2D> Data, unsigned Depth);
  // void PrintKdTree(KdTree* Tree, unsigned Depth);
  // double MeasureDistance(Point2D Point1, Point2D Point2, unsigned Method);
  // Point2D FindMiddleValue(std::vector<Point2D> Vec,unsigned Method);
  // Point2D SearchNearestNeighbor(Point2D Goal, KdTree* Tree);
  // std::vector<int> SearchNearestNeighborBBF(KdTree* Tree, Point2D Point);
  // void KdTreeRelease(KdTree* Tree);
};

struct PriorityInfo {
  KdTree *Tree;
  double Distance;
  double HyperPlaneDistance;
};

// API
int Partition(std::vector<Point2D> &Input, int StartIndex, int EndIndex,
              unsigned Method);

void QuickSort(std::vector<Point2D> &Input, int StartIndex, int EndIndex,
               unsigned Method);

void CreateKdTree(KdTree *Tree, std::vector<Point2D> Data, unsigned Depth);

void PrintKdTree(KdTree *Tree, unsigned Depth);

double MeasureDistance(Point2D Point1, Point2D Point2, unsigned Method);

Point2D FindMiddleValue(std::vector<Point2D> Vec, unsigned Method);

Point2D SearchNearestNeighbor(Point2D Goal, KdTree *Tree);

std::vector<int> SearchNearestNeighborBBF(KdTree *Tree, Point2D Point,
                                          double MaxDistance,
                                          double MinDistance);

void KdTreeRelease(KdTree *Tree);

}  // namespace sprinkle
}  // namespace geditor
