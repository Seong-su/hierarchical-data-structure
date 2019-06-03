#ifndef K_ARY_TREE_HPP
#define K_ARY_TREE_HPP

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

class KAryTree {
  using TreePtr = std::unique_ptr<KAryTree>;

 private:
  /* data */
 public:
  KAryTree() = delete;
  KAryTree(int fam_id);
  KAryTree(const std::string &name, int fam_id);
  ~KAryTree();

  std::unordered_set<std::string> name_content_;
  std::vector<TreePtr> trees_;
  std::string name_;
  unsigned int family_;

  bool contains(const std::string &node_name);
  void add_node(TreePtr node);
  void merge_tree(TreePtr tree);
};

KAryTree::KAryTree(int fam_id) : family_(fam_id) {}

KAryTree::KAryTree(const std::string &name, int fam_id) : family_(fam_id) {
  name_content_.insert(name);
}

KAryTree::~KAryTree() {}

bool KAryTree::contains(const std::string &node_name) {
  return name_content_.find(node_name) != name_content_.end();
}

void KAryTree::add_node(TreePtr node) {
  for (const auto &name : node->name_content_) {
    name_content_.insert(name);
  }
  trees_.push_back(std::move(node));
}

void KAryTree::merge_tree(TreePtr tree) {
  name_content_.insert(tree->name_content_.begin(), tree->name_content_.end());
  std::move(tree->trees_.begin(), tree->trees_.end(), std::back_inserter(trees_));
}

#endif  // K_ARY_TREE_HPP
