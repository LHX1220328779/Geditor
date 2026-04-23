#ifndef __IVPATHPLANNER_LINKED_LIST_H__
#define __IVPATHPLANNER_LINKED_LIST_H__

#include <memory>
#include <vector>

#include "coverage/geometry/site.h"

using geometry::Site;

namespace optimizer {

/**
 * @brief : this is basic element of linked list
 * @param : none
 * @return: none
 **/
class LinkNode {
 public:
  LinkNode() {}

  LinkNode(Site &site) : data_(site) {}

  ~LinkNode() = default;

 public:
  Site data_;
  std::shared_ptr<LinkNode> next_;
};

class LinkedList {
 public:
  LinkedList() { size_ = 0; }

  ~LinkedList() = default;

 public:
  /**
   * @brief : insert a new node at the end of this node
   * @param : a site, to be the data of last link node
   *          vector, the values that you want to insert to the tail of this
   *                  list
   * @return: none
   **/
  void push_back(Site site);

  void push_back(std::vector<Site> to_push);

  void push_back(std::vector<std::shared_ptr<Site>> to_push);

  /**
   * @brief : reset this list as an empty list
   * @param : none
   * @return: none
   **/
  void clear() { head_ = nullptr; }

  /**
   * @brief : determin if the list is empty
   * @param : none
   * @return: ture if not empty, false in others
   **/
  bool empty() { return (head_) ? true : false; }

  /**
   * @brief : get the number of nodes
   * @param : none
   * @return: int, the size of this list
   **/
  int size() { return size_; }

  /**
   * @brief : get the data of a specific node
   * @param : int, the pose of node that you want to get value
   * @return: the data of No. pos Node
   **/
  Site at(int pos);

  /**
   * @brief : insert a node into a specified location
   *          if pos is biger than the size of this list, insert the value to
   *          the tail of this list
   * @param : int pos,the location that you want to insert,
   *          Site data, to be the data of the inserted node
   *          vector to_insert, the values that you want to insert are stored in
   *                            the vector
   * @return: none
   **/
  void insert(int pos, Site &data);

  void insert(int pos, std::vector<Site> to_insert);

  void insert(int pos, std::vector<std::shared_ptr<Site>> to_insert);

  /**
   * @brief : erase elements from this list
   * @param : int start, end, the places of node that you want to erase
   * @return: none
   **/
  void erase(int pos);

  void erase(int start, int end);

  /**
   * @brief : find the nearest site in this list
   * @param : Site, to compare site
   *          double, error accuracy, default = 0.1
   * @return: LinkNode, the pointer of node with the nearest site
   **/
  std::shared_ptr<LinkNode> nearsest(Site to_compare, double err = 0.1);

  /**
   * @brief : the first node ofa specified data value
   * @param : Site, the site to compare
   * @return: LinkNode, the pointer of node with the same site
   **/
  std::shared_ptr<LinkNode> find(Site to_compare);

 private:
  /**
   * @brief : the head of linked list
   * @param : none
   * @return: none
   **/
  std::shared_ptr<LinkNode> head_;
  int size_;
};

}  // namespace optimizer

#endif  // __IVPATHPLANNER_LINKED_LIST_H__
