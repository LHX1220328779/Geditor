#ifndef IVPATHPLANNING_PATHPLANNER_DDPLANNER_H
#define IVPATHPLANNING_PATHPLANNER_DDPLANNER_H

#include <opencv2/opencv.hpp>
#include "coverage/pathplanner/blanket/commontypes.h"
#include "coverage/pathplanner/blanket/obsdetection.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

struct RRTNode {
  ccpp::Pointf cur;
  std::vector<ccpp::Pointf> curve;
  float cost;
  RRTNode* parent;
};

class DDplanner {
 public:
  DDplanner(const ccpp::Pointf& start, const ccpp::Pointf& goal,
            const cv::Mat& bmap);
  DDplanner();
  ~DDplanner();
  std::vector<ccpp::Pointf> RRTPlanning();

 private:
  bool Initialize();
  RRTNode* GetRandomNode();
  RRTNode* GetNearestNode(RRTNode* node);
  std::vector<ccpp::Pointf> DubinsConfig(RRTNode* start, RRTNode* end);
  float DubinsLength(RRTNode* start, RRTNode* end);
  bool CollisionDetection(std::vector<ccpp::Pointf>& curve);
  std::vector<int> GetNearNodes(RRTNode* node, std::vector<RRTNode*>& list);
  RRTNode* chooseparent(RRTNode* node, std::vector<RRTNode*>& nearnodes);
  bool Rewire(RRTNode* node, std::vector<int>& nearnum);
  bool ReachGoal(RRTNode* node);
  int GetBestIndex();
  std::vector<ccpp::Pointf> gen_final_course(int bestindex);

 private:
  ccpp::Pointf start_;
  ccpp::Pointf goal_;
  int min_xrand_;
  int max_xrand_;
  int min_yrand_;
  int max_yrand_;
  int goal_sample_rate_;
  int max_iter_;

  cv::Mat binary_map_;
  ObsDetection* obsdect_;
  std::vector<RRTNode*> nodelist_;
  RRTNode* root_;
  RRTNode* lastnode_;
  float goal_dist_limit_;
  float goal_yaw_limit_;
};

#endif
