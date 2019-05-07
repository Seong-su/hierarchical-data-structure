#include <set>
#include <stack>
#include <string>

#include "BCTNode.hpp"

class BCTNode {
 public:
  explicit BCTNode(const std::string &source_node,
                   const std::string &target_node) {
    BCTNode *source_tree = nullptr, *target_tree = nullptr;

    while (!forest.trees.empty()) {
      BCTNode *tree_temp = forest.trees.top();
      forest.trees.pop();
      if (tree_temp->name_content_.find(source_node) !=
          tree_temp->name_content_.end()) {
        source_tree = tree_temp;
      }
      if (tree_temp->name_content_.find(target_node) !=
          tree_temp->name_content_.end()) {
        target_tree = tree_temp;
      }
      if (source_tree == nullptr && target_tree == nullptr) {
        forest.trees_updated.push(tree_temp);
      }
    }

    if (source_tree != nullptr && target_tree != nullptr) {
      BCTNode *new_root;
      forest.trees_updated.push(new_root);
    } else if (source_tree != nullptr) {
      forest.trees_updated.push(source_tree);
    } else if (target_tree != nullptr) {
      forest.trees_updated.push(target_tree);
    }

    forest.trees_updated.swap(forest.trees);
  }
  ~BCTNode() {}

 private:
  static BCTForest forest;

  std::set<std::string> name_content_;
  double cost_;
  BCTNode *parent_, *left_, *right_;
};

class BCTForest {
 public:
  std::stack<BCTNode *> trees;
  std::stack<BCTNode *> trees_updated;

 private:
}