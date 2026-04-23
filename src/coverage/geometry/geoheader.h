#ifndef __COMMON_GEOMETRY_GEOHEADER_H__
#define __COMMON_GEOMETRY_GEOHEADER_H__
#include "site.h"

#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace geometry {

using SiteVec = std::vector<Site>;

using SiteSet = std::set<Site, Site::compare>;

template <typename T>
using KeySiteMap = std::map<T, Site>;

template <typename T>
using SiteValueMap = std::map<Site, T, Site::compare>;

template <typename T>
using SiteValueUnorderedMap =
    std::unordered_map<Site, T, Site::hash_key, Site::hash_equal>;

template <typename T>
using SiteValueMultiMap = std::multimap<Site, T, Site::compare>;

}  // namespace geometry
#endif  // __COMMON_GEOMETRY_GEOHEADER_H__