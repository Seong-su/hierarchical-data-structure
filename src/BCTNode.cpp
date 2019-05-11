#include <forward_list>
#include <string>
#include <unordered_set>

#include "BCTNode.hpp"

BCTNode::BCTNode(const std::string &source_node, const std::string &target_node)
    : parent_(nullptr) {
  BCTNode *source_root = forest_.find_root_contains_node(source_node);
  BCTNode *target_root = forest_.find_root_contains_node(target_node);

  if (source_root && target_root) {
    left_ = source_root;
    right_ = target_root;

    name_content_ = source_root->name_content_;
    name_content_.insert(target_root->name_content_.begin(),
                         target_root->name_content_.end());

    source_root->parent_ = this;
    target_root->parent_ = this;

    forest_.erase_root(source_root);
    forest_.erase_root(target_root);
    forest_.insert_root(this);
  } else if (source_root) {
    left_ = source_root;
    right_ = nullptr;

    name_content_ = source_root->name_content_;
    name_content_.insert(target_node);

    source_root->parent_ = this;

    forest_.erase_root(source_root);
    forest_.insert_root(this);
  } else if (target_root) {
    left_ = nullptr;
    right_ = target_root;

    name_content_ = target_root->name_content_;
    name_content_.insert(source_node);

    target_root->parent_ = this;

    forest_.erase_root(target_root);
    forest_.insert_root(this);
  } else {
    left_ = nullptr;
    right_ = nullptr;

    name_content_.insert(source_node);
    name_content_.insert(target_node);

    forest_.insert_root(this);
  }
}

BCTNode::~BCTNode() {}

bool BCTNode::contains_node(const std::string &name) {
  return name_content_.find(name) != name_content_.end();
}


BCTNode *BCTForest::find_root_contains_node(const std::string &name) {
  BCTNode *root = nullptr;
  for (auto itr = trees.begin(); itr != trees.end(); ++itr) {
    if ((*itr)->contains_node(name)) {
      root = *itr;
    }
  }

  return root;
}

void BCTForest::erase_root(BCTNode *root) {
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

void BCTForest::insert_root(BCTNode *root) { trees.emplace_front(root); }
