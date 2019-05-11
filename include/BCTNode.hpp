#ifndef BCT_NODE_HPP_
#define BCT_NODE_HPP_

#include <forward_list>
#include <string>
#include <unordered_set>

class BCTForest;

class BCTNode {
 public:
  explicit BCTNode(const std::string &source_node,
                   const std::string &target_node);
  ~BCTNode();

  bool contains_node(const std::string &name);

  // static BCTForest &Forest() { return forest_; }

 private:
  static BCTForest forest_;

  std::unordered_set<std::string>
      name_content_;  // memory overhead and performance warning
  double cost_;
  BCTNode *parent_, *left_, *right_;
};

class BCTForest {
 public:
  BCTNode *find_root_contains_node(const std::string &name);
  void erase_root(BCTNode *root);
  void insert_root(BCTNode *root);

 private:
  std::forward_list<BCTNode *> trees;
};

#endif  // BCT_NODE_HPP_