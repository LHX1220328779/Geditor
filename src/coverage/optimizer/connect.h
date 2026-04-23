#ifndef __TOOLBOX_OPTIMIZER_CONNECT_H__
#define __TOOLBOX_OPTIMIZER_CONNECT_H__

#include "coverage/geometry/geoheader.h"
#include "coverage/optimizer/meanfilt.h"

#include <memory>
#include <vector>

using geometry::Site;
using geometry::SiteVec;

namespace optimizer {
class Connect {
 public:
  Connect() = default;

  ~Connect() = default;

 public:
  void GetPath(SiteVec &list, SiteVec &list1, SiteVec &list2);

  void GetPath(std::vector<std::shared_ptr<Site>> &list,
               std::vector<std::shared_ptr<Site>> &list1,
               std::vector<std::shared_ptr<Site>> &list2);

 private:
  void GetAnchors(SiteVec &list1, SiteVec &list2);

  void GetAnchors(std::vector<std::shared_ptr<Site>> &list1,
                  std::vector<std::shared_ptr<Site>> &list2);

 private:
  MeanFilt mfl;
  SiteVec connect_path;
  SiteVec::iterator list1_connect_;
  SiteVec::iterator list2_connect_;
};

}  // namespace optimizer
#endif  // __TOOLBOX_OPTIMIZER_CONNECT_H__