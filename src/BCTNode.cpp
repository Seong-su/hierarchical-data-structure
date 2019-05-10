#include <forward_list>
#include <iostream>
#include <string>
#include <unordered_set>

#include "BCTNode.hpp"

class BCTNode {
 public:
  explicit BCTNode(const std::string &source_node,
                   const std::string &target_node)
      : parent_(nullptr) {
    BCTNode *source_root = forest_.find_root_contains_node(source_node);
    BCTNode *target_root = forest_.find_root_contains_node(target_node);

    if (source_root && target_root) {
      left_ = source_root;
      right_ = target_root;

      left_->parent_ = this;
      right_->parent_ = this;

      forest_.erase_root(source_root);
      forest_.erase_root(target_root);
      forest_.insert_root(this);
    } else if (source_root) {
    } else if (target_root) {
    }
  }
  ~BCTNode() {}

  bool contains_node(const std::string &name) {
    return name_content_.find(name) != name_content_.end();
  }

 private:
  // BCTNode *merge(BCTNode *node) {}

  static BCTForest forest_;

  std::unordered_set<std::string>
      name_content_;  // memory overhead and performance warning
  double cost_;
  BCTNode *parent_, *left_, *right_;
};

class BCTForest {
 public:
  BCTNode *find_root_contains_node(const std::string &name) {
    BCTNode *root = nullptr;
    for (auto itr = trees.begin(); itr != trees.end(); ++itr) {
      if ((*itr)->contains_node(name)) {
        root = *itr;
      }
    }

    return root;
  }

  void erase_root(BCTNode *root) {
    if (!root) {
      return;
    }

    for (auto itr = trees.begin(); itr != trees.end(); ++itr) {
      if ((*itr) == root) {
        auto itr_before = std::prev(itr, 1);
        trees.erase_after(itr_before);
        break;
      }
    }
  }

  void insert_root(BCTNode *root) {
    trees.emplace_front(root);
  }

 private:
  std::forward_list<BCTNode *> trees;
};
