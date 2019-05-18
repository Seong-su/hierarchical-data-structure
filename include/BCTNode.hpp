#ifndef BCT_NODE_HPP_
#define BCT_NODE_HPP_

#include <forward_list>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

class BCTNode {
 public:
  explicit BCTNode(const std::string &source_node,
                   const std::string &target_node, int id);
  ~BCTNode();

  bool contains_node(const std::string &name) const;

  const std::string &left_name() const { return left_name_; }
  const std::string &right_name() const { return right_name_; }
  std::unordered_set<std::string> &name_content() { return name_content_; }
  int id() const { return id_; }

  std::unique_ptr<BCTNode> left, right;

 private:
  std::unordered_set<std::string>
      name_content_;  // memory overhead and performance warning
  std::string left_name_, right_name_;
  double cost_;
  int id_;
};

class BCTForest {
 public:
  BCTForest();
  ~BCTForest();

  void insert(std::unique_ptr<BCTNode> node);

  void preorder(const std::unique_ptr<BCTNode> &root, int level,
                std::map<int, std::vector<int>> &map, std::ofstream &fout);

  void traversal();

 private:
  std::unique_ptr<BCTNode> find_root_contains_node(const std::string &name);
  void insert_root(std::unique_ptr<BCTNode> root);

  std::forward_list<std::unique_ptr<BCTNode>> trees;
};

#endif  // BCT_NODE_HPP_