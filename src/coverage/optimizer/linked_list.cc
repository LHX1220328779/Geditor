#include "optimizer/linked_list.h"

#include <iostream>

using geometry::Site;

namespace optimizer {

void LinkedList::push_back(Site site) {
  if (!head_) {
    head_ = std::shared_ptr<LinkNode>(new LinkNode(site));
    size_ = 1;
    return;
  }

  std::shared_ptr<LinkNode> p = head_;
  while (p->next_) p = p->next_;

  p->next_ = std::shared_ptr<LinkNode>(new LinkNode(site));

  ++size_;
}

void LinkedList::push_back(std::vector<Site> to_push) {
  if (to_push.size() == 0) {
    // ROS_ERROR_STREAM("CAN'T PUSH AN EMPTY VECTOR");
    std::cout << "CAN'T PUSH AN EMPTY VECTOR" << std::endl;
    return;
  } else if (!head_) {
    head_ = std::shared_ptr<LinkNode>(new LinkNode(to_push.front()));
    std::shared_ptr<LinkNode> p = head_;
    ++size_;

    for (int i = 1; i < to_push.size(); ++i) {
      p->next_ = std::shared_ptr<LinkNode>(new LinkNode(to_push[i]));
      p = p->next_;
      ++size_;
    }
    return;
  } else {
    std::shared_ptr<LinkNode> p = head_;
    while (p->next_) p = p->next_;

    for (auto &s : to_push) {
      p->next_ = std::shared_ptr<LinkNode>(new LinkNode(s));
      ++size_;
    }
  }
}

void LinkedList::push_back(std::vector<std::shared_ptr<Site>> to_push) {
  if (to_push.empty()) {
    // ROS_ERROR_STREAM("CAN'T PUSH AN EMPTY VECTOR");
    std::cout << "CAN'T PUSH AN EMPTY VECTOR" << std::endl;
    return;
  } else if (!head_) {
    head_ = std::shared_ptr<LinkNode>(new LinkNode(*to_push.front()));
    std::shared_ptr<LinkNode> p = head_;
    ++size_;

    for (int i = 1; i < to_push.size(); ++i) {
      p->next_ = std::shared_ptr<LinkNode>(new LinkNode(*to_push[i]));
      p = p->next_;
      ++size_;
    }
    return;
  } else {
    std::shared_ptr<LinkNode> p = head_;
    while (p->next_) p = p->next_;

    for (auto &s : to_push) {
      p->next_ = std::shared_ptr<LinkNode>(new LinkNode(*s));
      ++size_;
    }
  }
}

Site LinkedList::at(int pos) {
  if (0 == size_) {
    // ROS_ERROR_STREAM("THE LIST IS EMPTY");
    std::cout << "THE LIST IS EMPTY" << std::endl;
    return Site();
  }

  if (pos == 0) return head_->data_;

  if (pos > 0) {
    if (pos > size_ - 1) {
      // ROS_ERROR_STREAM("OUT OF RANGE");
      std::cout << "OUT OF RANGE" << std::endl;
      return Site();
    } else {
      std::shared_ptr<LinkNode> p = head_;
      int j = 0;
      while (p && j < pos) {
        ++j;
        p = p->next_;
      }
      return p->data_;
    }
  } else {
    if (std::abs(pos) > size_) {
      // ROS_ERROR_STREAM("OUT OF RANGE");
      std::cout << "OUT OF RANGE" << std::endl;
      return Site();
    } else {
      int j = 0;
      int real_pos = size_ + pos;
      std::shared_ptr<LinkNode> p = head_;
      while (p && j < real_pos) {
        ++j;
        p = p->next_;
      }
      return p->data_;
    }
  }
}

void LinkedList::insert(int pos, Site &data) {
  if (pos < 0) {
    // ROS_ERROR_STREAM("POS COULDN'T SMALLER THAN ZERO");
    std::cout << "POS COULDN'T SMALLER THAN ZERO" << std::endl;
    return;
  }

  if (0 == pos) {
    std::shared_ptr<LinkNode> new_value(new LinkNode(data));
    new_value->next_ = head_;
    head_ = new_value;
    ++size_;
    return;
  }
  // find pre pointer of target index
  std::shared_ptr<LinkNode> p = head_;
  for (int i = 1; i < pos; ++i) {
    if (p) {
      p = p->next_;
    } else {
      // ROS_ERROR_STREAM("OUT OF RANGE");
      std::cout << "OUT OF RANGE" << std::endl;
      return;
    }
  }
  if (!p) {
    // ROS_ERROR_STREAM("OUT OF RANGE");
    std::cout << "OUT OF RANGE" << std::endl;
    return;
  }
  // insert a new value
  std::shared_ptr<LinkNode> new_value(new LinkNode(data));
  if (p->next_) {
    new_value->next_ = p->next_;
    p->next_ = new_value;
  } else {
    p->next_ = new_value;
  }
  ++size_;
}

void LinkedList::insert(int pos, std::vector<Site> to_insert) {
  if (pos < 0) {
    // ROS_ERROR_STREAM("POS COULDN'T SMALLER THAN ZERO");
    std::cout << "POS COULDN'T SMALLER THAN ZERO" << std::endl;
    return;
  }

  if (to_insert.empty()) return;

  if (0 == pos) {
    std::shared_ptr<LinkNode> new_value(new LinkNode(to_insert.front()));
    std::shared_ptr<LinkNode> puppet = new_value;
    for (int i = 1; i < to_insert.size(); ++i) {
      puppet->next_ = std::shared_ptr<LinkNode>(new LinkNode(to_insert[i]));
      puppet = puppet->next_;
    }
    puppet->next_ = head_;
    head_ = new_value;
    return;
  }

  // find pre pointer of target index
  std::shared_ptr<LinkNode> pre_ptr = head_;
  for (int i = 1; i < pos; ++i) {
    if (pre_ptr) {
      pre_ptr = pre_ptr->next_;
    } else {
      // ROS_ERROR_STREAM("OUT OF RANGE")
      std::cout << "OUT OF RANGE" << std::endl;
      return;
    }
  }
  if (!pre_ptr) {
    // ROS_ERROR_STREAM("OUT OF RANGE")
    std::cout << "OUT OF RANGE" << std::endl;
    return;
  }
  // insert new value
  std::shared_ptr<LinkNode> new_value(new LinkNode(to_insert.front()));
  std::shared_ptr<LinkNode> puppet = new_value;
  for (int i = 1; i < to_insert.size(); ++i) {
    puppet->next_ = std::shared_ptr<LinkNode>(new LinkNode(to_insert[i]));
    puppet = puppet->next_;
  }
  if (pre_ptr->next_) {
    puppet->next_ = pre_ptr->next_;
    pre_ptr->next_ = new_value;
  } else {
    pre_ptr->next_ = new_value;
  }
}

void LinkedList::insert(int pos, std::vector<std::shared_ptr<Site>> to_insert) {
}

void LinkedList::erase(int pos) {}

void LinkedList::erase(int start, int end) {}

std::shared_ptr<LinkNode> LinkedList::nearsest(Site to_compare, double err) {
  return nullptr;
}

std::shared_ptr<LinkNode> LinkedList::find(Site to_compare) { return nullptr; }

}  // namespace optimizer