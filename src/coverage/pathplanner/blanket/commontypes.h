#ifndef IVPATHPLANNING_PATHPLANNER_COMMONTYPES_H
#define IVPATHPLANNING_PATHPLANNER_COMMONTYPES_H

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

const double C_PI = 3.1415926535898;
const double C_HALF_RAD = cos(45.0 * C_PI / 180.0);  // cos45
// 7.5 degree
const float dx[] = {5.23598, 5.22105, 5.22105};
const float dy[] = {0, 0.3422, -0.3422};
const float dt[] = {0, 0.1309, -0.1309};

// 15 degree
// const float dx[] = {5.23598*10, 5.22105*10, 5.22105*10};
// const float dy[] = {0, 0.3422*10, -0.3422*10};
// const float dt[] = {0, 0.1309*2, -0.1309*2};

namespace ccpp {
struct Segment {
  int start_num;
  int end_num;
  int cur_pixel;
  int region_num;
  Segment() : start_num(0), end_num(0), cur_pixel(0), region_num(0) {}
  Segment(int start_num_, int end_num_, int cur_pixel_, int region_num_)
      : start_num(start_num_),
        end_num(end_num_),
        cur_pixel(cur_pixel_),
        region_num(region_num_) {}
  Segment& operator=(const Segment& segment) {
    start_num = segment.start_num;
    end_num = segment.end_num;
    cur_pixel = segment.cur_pixel;
    region_num = segment.region_num;
    return *this;
  }
  Segment(const Segment& segment) {
    start_num = segment.start_num;
    end_num = segment.end_num;
    cur_pixel = segment.cur_pixel;
    region_num = segment.region_num;
  }
};

struct Slice {
  int segment_num;
  int init_pixel;
  // std::vector<int> intersection;
  std::vector<Segment> intersection;
  Slice() : segment_num(0), init_pixel(0) {}
  Slice(int segment_num_, int init_pixel_, std::vector<Segment> intersection_)
      : segment_num(segment_num_),
        init_pixel(init_pixel_),
        intersection(intersection_) {}
  Slice& operator=(const Slice& slice) {
    segment_num = slice.segment_num;
    init_pixel = slice.init_pixel;
    intersection = slice.intersection;
    return *this;
  }
  Slice(const Slice& slice) {
    segment_num = slice.segment_num;
    init_pixel = slice.init_pixel;
    intersection = slice.intersection;
  }
};

struct Pointi {
  int x;
  int y;
  float yaw;
  float s;
  Pointi() : x(0.0), y(0.0), yaw(0.0), s(0.0) {}
  Pointi(int _x, int _y, float _yaw) : x(_x), y(_y), yaw(_yaw) {}
  Pointi(int _x, int _y, float _yaw, int _s) : x(_x), y(_y), yaw(_yaw), s(_s) {}
  Pointi& operator=(const Pointi& pointi) {
    x = pointi.x;
    y = pointi.y;
    yaw = pointi.yaw;
    s = pointi.s;
    return *this;
  }
  Pointi(const Pointi& pointi) {
    x = pointi.x;
    y = pointi.y;
    yaw = pointi.yaw;
    s = pointi.s;
  }
};

struct Pointf {
  float x;
  float y;
  float yaw;
  float s;
  double v;
  Pointf() : x(0.0), y(0.0), yaw(0.0), s(0.0) {}
  Pointf(float _x, float _y, float _yaw) : x(_x), y(_y), yaw(_yaw) {}
  Pointf(float _x, float _y, float _yaw, float _s)
      : x(_x), y(_y), yaw(_yaw), s(_s) {}
  Pointf(Pointi p) {
    x = static_cast<float>(p.x);
    y = static_cast<float>(p.y);
    yaw = static_cast<float>(p.yaw);
    s = static_cast<float>(p.s);
  }
  Pointf& operator=(const Pointf& pointf) {
    x = pointf.x;
    y = pointf.y;
    yaw = pointf.yaw;
    s = pointf.s;
    return *this;
  }
  Pointf(const Pointf& pointf) {
    x = pointf.x;
    y = pointf.y;
    yaw = pointf.yaw;
    s = pointf.s;
  }
};

struct Node3D {
  float x;
  float y;
  float yaw;
  float g;
  float h;
  bool o;
  bool c;
  Node3D* pre;
  Node3D() : Node3D(0, 0, 0) {}
  Node3D(const Pointf& p) : Node3D(p.x, p.y, p.yaw) {}
  Node3D(float _x, float _y, float _yaw) {
    x = _x;
    y = _y;
    yaw = _yaw;
    g = 0;
    h = 0;
    o = false;
    c = false;
    pre = NULL;
  }
  bool operator==(const Node3D& rhs) {
    return fabs(this->x - rhs.x) < 1e-3 && fabs(this->y - rhs.y) < 1e-3 &&
           fabs(this->yaw - rhs.yaw) < 1e-3;
  }
  Node3D& operator=(const Node3D& rhs) {
    this->x = rhs.x;
    this->y = rhs.y;
    this->yaw = rhs.yaw;
    this->g = rhs.g;
    this->h = rhs.h;
    this->o = rhs.o;
    this->c = rhs.c;
    this->pre = rhs.pre;
    return *this;
  }
};

struct cmp {
  bool operator()(const Node3D l, const Node3D r) { return l.h > r.h; }
};

template <typename T>
struct comparefunc {
  bool operator()(const T& l, const T& r) const {
    return fabs(l.x - r.x) < 1e-3 && fabs(l.y - r.y) < 1e-3 &&
           fabs(l.yaw - r.yaw) < 1e-3;
  }
};

template <typename T>
struct keyfunc {
  std::size_t operator()(const T& t) const {
    std::hash<float> val;
    return std::size_t(val(t.x + t.y + 2 * t.yaw));
  }
};

typedef std::priority_queue<Node3D, std::vector<Node3D>, cmp> PriorityList;
typedef std::unordered_map<Node3D, Node3D, keyfunc<Node3D>,
                           comparefunc<Node3D> >
    UnorderT;
typedef std::unordered_map<Node3D, float, keyfunc<Node3D>, comparefunc<Node3D> >
    UnorderF;

namespace cvcarmodel {
static const float car_width = 0.8;
static const float half_carwidth = car_width / 2.0;
static const float car_length = 1.2;
static const float front2base = 1.0;
static const float tail2base = 0.2;
static const float car_radius = 2.0;

static const int cv_car_width = static_cast<int>(car_width * 20.0);
static const int cv_half_car_width = static_cast<int>(half_carwidth * 20.0);
static const int cv_car_length = static_cast<int>(car_length * 20.0);
static const int cv_front2base = static_cast<int>(front2base * 20.0);
static const int cv_tail2base = static_cast<int>(tail2base * 20.0);
static const int cv_car_radius = static_cast<int>(car_radius * 20.0);
}  // namespace cvcarmodel
}  // namespace ccpp

#endif
