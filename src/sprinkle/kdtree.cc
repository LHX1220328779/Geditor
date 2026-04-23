
#include "sprinkle/sprinkle_tool.h"

namespace geditor {

namespace sprinkle {

// overload priority_queue
// return true : A has a lower priority than B
bool operator<(PriorityInfo A, PriorityInfo B) {
  if (A.Distance == B.Distance)
    return A.HyperPlaneDistance > B.HyperPlaneDistance;
  else
    return A.Distance > B.Distance;
}

int Partition(std::vector<Point2D> &Input, int StartIndex, int EndIndex,
              unsigned Method) {
  Point2D Temp = Input[EndIndex];
  int i = StartIndex - 1;
  for (int j = StartIndex; j < EndIndex; j++) {
    if (0 == Method) {
      if (Input[j].x < Temp.x) {
        Point2D Point;
        i++;
        Point = Input[i];
        Input[i] = Input[j];
        Input[j] = Point;
      }
    } else {
      if (Input[j].y < Temp.y) {
        Point2D Point;
        i++;
        Point = Input[i];
        Input[i] = Input[j];
        Input[j] = Point;
      }
    }
  }
  Input[EndIndex] = Input[i + 1];
  Input[i + 1] = Temp;
  return i + 1;
}

void QuickSort(std::vector<Point2D> &Input, int StartIndex, int EndIndex,
               unsigned Method) {
  if (StartIndex < EndIndex) {
    int q = Partition(Input, StartIndex, EndIndex, Method);
    QuickSort(Input, StartIndex, q - 1, Method);
    QuickSort(Input, q + 1, EndIndex, Method);
  }
}

Point2D FindMiddleValue(std::vector<Point2D> Vec, unsigned Method) {
  QuickSort(Vec, 0, Vec.size() - 1, Method);
  // for (int i = 0; i < Vec.size(); ++i){
  // 	std::cout << Vec[i].x << "   " << Vec[i].y << std::endl;
  // }
  auto Pos = Vec.size() / 2;
  return Vec[Pos];
}

void CreateKdTree(KdTree *Tree, std::vector<Point2D> Data, unsigned Depth) {
  unsigned SamplesNum = Data.size();
  unsigned k = (sizeof(Point2D) - sizeof(int)) / sizeof(double);
  // std::cout << " ----k---- " << k  << std::endl;
  unsigned SplitAttribute = Depth % k;
  // std::cout << " SplitAttribute : " << SplitAttribute << std::endl;
  if (SamplesNum == 0) {
    return;
  }
  if (SamplesNum == 1) {
    Tree->Root = Data[0];
    Tree->SortDim = SplitAttribute;
    // std::cout << "Leaf->Root : " << Tree->Root.x << " " << Tree->Root.y <<
    // std::endl;
    Tree->CreateFlag = true;
    return;
  }
  Point2D SplitValue = FindMiddleValue(Data, SplitAttribute);
  // std::cout << "SplitValue : " << SplitValue.x << " " << SplitValue.y <<
  // std::endl;
  std::vector<Point2D> SubSet1;
  std::vector<Point2D> SubSet2;
  for (unsigned i = 0; i < SamplesNum; ++i) {
    // Point2D   0 : x   1 :  y
    switch (SplitAttribute) {
      case 0:
        if (Data[i].x == SplitValue.x && Data[i].y == SplitValue.y &&
            !(Tree->CreateFlag)) {
          Tree->Root = Data[i];
          Tree->SortDim = SplitAttribute;
          // std::cout << "Tree->Root : " << Tree->Root.x << " " << Tree->Root.y
          // << std::endl;
          Tree->CreateFlag = true;
        } else {
          if (Data[i].x < SplitValue.x)
            SubSet1.push_back(Data[i]);
          else
            SubSet2.push_back(Data[i]);
        }
        break;
      case 1:
        if (Data[i].x == SplitValue.x && Data[i].y == SplitValue.y &&
            !(Tree->CreateFlag)) {
          Tree->Root = Data[i];
          Tree->SortDim = SplitAttribute;
          // std::cout << "Tree->Root : " << Tree->Root.x << " " << Tree->Root.y
          // << std::endl;
          Tree->CreateFlag = true;
        } else {
          if (Data[i].y < SplitValue.y)
            SubSet1.push_back(Data[i]);
          else
            SubSet2.push_back(Data[i]);
        }
        break;
      default:
        break;
    }
  }
  Tree->LeftChild = new KdTree;
  Tree->LeftChild->Parent = Tree;
  Tree->RightChild = new KdTree;
  Tree->RightChild->Parent = Tree;
  CreateKdTree(Tree->LeftChild, SubSet1, Depth + 1);
  CreateKdTree(Tree->RightChild, SubSet2, Depth + 1);
}

void KdTreeRelease(KdTree *Tree) {
  if (!Tree) return;
  KdTreeRelease(Tree->LeftChild);
  KdTreeRelease(Tree->RightChild);
  free(Tree);
}

// debug
void PrintKdTree(KdTree *Tree, unsigned Depth) {
  std::cout << "\t";
  std::cout << Tree->Root.x << "," << Tree->Root.y << std::endl;
  std::cout << "CreateFlag : " << Tree->CreateFlag << std::endl;
  if (Tree->LeftChild == NULL && Tree->RightChild == NULL)
    return;
  else {
    if (Tree->LeftChild != NULL && Tree->LeftChild->CreateFlag) {
      for (unsigned i = 0; i < Depth + 1; ++i) std::cout << "\t";
      std::cout << " left:";
      PrintKdTree(Tree->LeftChild, Depth + 1);
    }
    std::cout << std::endl;
    if (Tree->RightChild != NULL && Tree->RightChild->CreateFlag) {
      for (unsigned i = 0; i < Depth + 1; ++i) std::cout << "\t";
      std::cout << "right:";
      PrintKdTree(Tree->RightChild, Depth + 1);
    }
    std::cout << std::endl;
  }
}

double MeasureDistance(Point2D Point1, Point2D Point2, unsigned Method) {
  double res = 0;
  switch (Method) {
    // European distance
    case 0:
      res = pow((Point1.x - Point2.x), 2) + pow((Point1.y - Point2.y), 2);
      return sqrt(res);
      // Manhattan distance
    case 1:
      res = abs(Point1.x - Point2.x) + abs(Point1.y - Point2.y);
      return res;
    default:
      std::cerr << "Invalid method!!" << std::endl;
      return -1;
  }
  return 0.0;
}

Point2D SearchNearestNeighbor(Point2D Goal, KdTree *Tree) {
  // step1
  unsigned k = (sizeof(Point2D) - sizeof(int)) / sizeof(double);
  unsigned d = 0;
  KdTree *CurrentTree = Tree;
  Point2D CurrentNearest = CurrentTree->Root;
  while (!CurrentTree->IsLeaf()) {
    unsigned Index = d % k;
    switch (Index) {
      case 0:
        if (CurrentTree->RightChild->IsEmpty() || Goal.x < CurrentNearest.x) {
          CurrentTree = CurrentTree->LeftChild;
        } else {
          CurrentTree = CurrentTree->RightChild;
        }
        break;
      case 1:
        if (CurrentTree->RightChild->IsEmpty() || Goal.y < CurrentNearest.y) {
          CurrentTree = CurrentTree->LeftChild;
        } else {
          CurrentTree = CurrentTree->RightChild;
        }
        break;
      default:
        break;
    }
    ++d;
  }
  CurrentNearest = CurrentTree->Root;
  // step2
  double CurrentDistance = MeasureDistance(Goal, CurrentNearest, 0);
  KdTree *SearchDistrict;
  if (CurrentTree->IsLeft()) {
    if (CurrentTree->Parent->RightChild == NULL)
      SearchDistrict = CurrentTree;
    else
      SearchDistrict = CurrentTree->Parent->RightChild;
  } else {
    SearchDistrict = CurrentTree->Parent->LeftChild;
  }
  while (SearchDistrict->Parent != NULL) {
    double DistrictDistance = 0;
    switch ((d + 1) % k) {
      case 0:
        DistrictDistance = abs(Goal.x - SearchDistrict->Parent->Root.x);
        break;
      case 1:
        DistrictDistance = abs(Goal.y - SearchDistrict->Parent->Root.y);
        break;
      default:
        break;
    }
    if (DistrictDistance < CurrentDistance) {
      double ParentDistance =
          MeasureDistance(Goal, SearchDistrict->Parent->Root, 0);
      if (ParentDistance < CurrentDistance) {
        CurrentDistance = ParentDistance;
        CurrentTree = SearchDistrict->Parent;
        CurrentNearest = CurrentTree->Root;
      }
      if (!SearchDistrict->IsEmpty()) {
        double RootDistance = MeasureDistance(Goal, SearchDistrict->Root, 0);
        if (RootDistance < CurrentDistance) {
          CurrentDistance = RootDistance;
          CurrentTree = SearchDistrict;
          CurrentNearest = CurrentTree->Root;
        }
      }
      if (SearchDistrict->LeftChild != NULL) {
        double LeftDistance =
            MeasureDistance(Goal, SearchDistrict->LeftChild->Root, 0);
        if (LeftDistance < CurrentDistance) {
          CurrentDistance = LeftDistance;
          CurrentTree = SearchDistrict;
          CurrentNearest = CurrentTree->Root;
        }
      }
      if (SearchDistrict->RightChild != NULL) {
        double RightDistance =
            MeasureDistance(Goal, SearchDistrict->RightChild->Root, 0);
        if (RightDistance < CurrentDistance) {
          CurrentDistance = RightDistance;
          CurrentTree = SearchDistrict;
          CurrentNearest = CurrentTree->Root;
        }
      }
    }
    if (SearchDistrict->Parent->Parent != NULL) {
      SearchDistrict = SearchDistrict->Parent->IsLeft()
                           ? SearchDistrict->Parent->Parent->RightChild
                           : SearchDistrict->Parent->Parent->LeftChild;
    } else {
      SearchDistrict = SearchDistrict->Parent;
    }
    ++d;
  }
  return CurrentNearest;
}

std::vector<int> SearchNearestNeighborBBF(KdTree *Tree, Point2D Point,
                                          double MaxDistance,
                                          double MinDistance) {
  std::vector<int> result;
  result.clear();
  if (Tree == NULL) return result;
  KdTree *P = Tree;
  std::priority_queue<PriorityInfo> PriQueue;
  PriorityInfo PInfo;
  if (0 == P->SortDim) {
    PInfo.Tree = P;
    PInfo.Distance = MeasureDistance(Point, Tree->Root, 0);
    PInfo.HyperPlaneDistance = fabs(Point.x - Tree->Root.x);
    PriQueue.push(PInfo);
  } else {
    PInfo.Tree = P;
    PInfo.Distance = MeasureDistance(Point, Tree->Root, 0);
    PInfo.HyperPlaneDistance = fabs(Point.y - Tree->Root.y);
    PriQueue.push(PInfo);
  }
  int t = 0;
  while (!PriQueue.empty()) {
    t++;
    PriorityInfo Tmp = PriQueue.top();
    PriQueue.pop();
    int SortDim = Tmp.Tree->SortDim;
    double PDis;
    if (0 == SortDim)
      PDis = fabs(Point.x - Tmp.Tree->Root.x);
    else
      PDis = fabs(Point.y - Tmp.Tree->Root.y);
    double TmpDis = Tmp.Distance;
    if (TmpDis <= MaxDistance && TmpDis >= MinDistance) {
      result.push_back(Tmp.Tree->Root.Label);
      // std::cout << TmpDis << "  " << Tmp.Tree->Root.Label << std::endl;
    }
    KdTree *Q = Tmp.Tree;
    while (Q->RightChild != NULL || Q->LeftChild != NULL) {
      t++;
      int SD = Q->SortDim;
      double PointData, QRootData;
      if (0 == SD) {
        PointData = Point.x;
        QRootData = Q->Root.x;
      } else {
        PointData = Point.y;
        QRootData = Q->Root.y;
      }

      if (PointData <= QRootData) {
        if (Q->LeftChild != NULL && Q->LeftChild->CreateFlag) {
          if (Q->RightChild != NULL && Q->RightChild->CreateFlag &&
              fabs(PointData - QRootData) < MaxDistance) {
            int RST = Q->RightChild->SortDim;
            if (0 == RST) {
              PInfo.Tree = Q->RightChild;
              PInfo.Distance = MeasureDistance(Point, Q->RightChild->Root, 0);
              // std::cout << PInfo.Tree->Root.Label << std::endl;
              PInfo.HyperPlaneDistance = fabs(Point.x - Q->RightChild->Root.x);
              PriQueue.push(PInfo);
            } else {
              PInfo.Tree = Q->RightChild;
              PInfo.Distance = MeasureDistance(Point, Q->RightChild->Root, 0);
              // std::cout << PInfo.Tree->Root.Label << std::endl;
              PInfo.HyperPlaneDistance = fabs(Point.y - Q->RightChild->Root.y);
              PriQueue.push(PInfo);
            }
          }
          Q = Q->LeftChild;
        } else {
          if (Q->RightChild != NULL && Q->RightChild->CreateFlag) {
            double RDis = MeasureDistance(Point, Q->RightChild->Root, 0);
            // updata result
            if (RDis <= MaxDistance && RDis >= MinDistance) {
              result.push_back(Q->RightChild->Root.Label);
              // std::cout << RDis << "  " << Q->RightChild->Root.Label <<
              // std::endl;
            }
          }
          break;
        }
      } else {
        if (Q->RightChild != NULL && Q->RightChild->CreateFlag) {
          if (Q->LeftChild != NULL && Q->LeftChild->CreateFlag &&
              fabs(PointData - QRootData) < MaxDistance) {
            int LST = Q->LeftChild->SortDim;
            if (0 == LST) {
              PInfo.Tree = Q->LeftChild;
              PInfo.Distance = MeasureDistance(Point, Q->LeftChild->Root, 0);
              // std::cout << PInfo.Tree->Root.Label << std::endl;
              PInfo.HyperPlaneDistance = fabs(Point.x - Q->LeftChild->Root.x);
              PriQueue.push(PInfo);
            } else {
              PInfo.Tree = Q->LeftChild;
              PInfo.Distance = MeasureDistance(Point, Q->LeftChild->Root, 0);
              // std::cout << PInfo.Tree->Root.Label << std::endl;
              PInfo.HyperPlaneDistance = fabs(Point.y - Q->LeftChild->Root.y);
              PriQueue.push(PInfo);
            }
          }
          Q = Q->RightChild;
        } else {
          if (Q->LeftChild != NULL && Q->LeftChild->CreateFlag) {
            double LDis = MeasureDistance(Point, Q->LeftChild->Root, 0);
            // updata result
            if (LDis <= MaxDistance && LDis >= MinDistance) {
              result.push_back(Q->LeftChild->Root.Label);
              // std::cout << LDis << "  " << Q->LeftChild->Root.Label <<
              // std::endl;
            }
          }
          break;
        }
      }
      double GDis = MeasureDistance(Point, Q->Root, 0);
      // updata result
      if (GDis <= MaxDistance && GDis >= MinDistance) {
        result.push_back(Q->Root.Label);
        // std::cout << GDis << "  " << Q->Root.Label << std::endl;
      }
    }
    // if (t > 600)
    // 	break;
  }
  // std::cout << "--------**********-----------" << t << std::endl;
  return result;
}
}  // namespace sprinkle
}  // namespace geditor
