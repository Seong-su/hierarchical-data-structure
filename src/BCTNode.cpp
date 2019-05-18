#include <algorithm>

#include "BCTNode.hpp"

// BCTNode

BCTNode::BCTNode(const std::string &source_node, const std::string &target_node,
                 int id)
    : id_(id) {
  left_name_ = source_node;
  right_name_ = target_node;

  name_content_.insert(source_node);
  name_content_.insert(target_node);
}

BCTNode::~BCTNode() {}

bool BCTNode::contains_node(const std::string &name) const {
  return name_content_.find(name) != name_content_.end();
}

// BCTForest

BCTForest::BCTForest() {}

BCTForest::~BCTForest() {}

void BCTForest::insert(std::unique_ptr<BCTNode> node) {
  const std::string &source_node = node->left_name();
  const std::string &target_node = node->right_name();

  std::unique_ptr<BCTNode> source_root = find_root_contains_node(source_node);
  std::unique_ptr<BCTNode> target_root = find_root_contains_node(target_node);

  if (source_root && target_root) {
    node->name_content() = source_root->name_content();
    node->name_content().insert(target_root->name_content().begin(),
                                target_root->name_content().end());

    node->left = std::move(source_root);
    node->right = std::move(target_root);

    insert_root(std::move(node));
  } else if (source_root) {
    node->name_content() = source_root->name_content();
    node->name_content().insert(target_node);

    node->left = std::move(source_root);

    insert_root(std::move(node));
  } else if (target_root) {
    node->name_content() = target_root->name_content();
    node->name_content().insert(source_node);

    node->right = std::move(target_root);

    insert_root(std::move(node));
  } else {
    insert_root(std::move(node));
  }
}

void BCTForest::preorder(const std::unique_ptr<BCTNode> &root, int level,
                         std::map<int, std::vector<int>> &map,
                         std::ofstream &fout) {
  if (root == nullptr) return;

  map[level].push_back(root->id());

  std::string left_id, right_id;
  if (root->left == nullptr) {
    left_id = root->left_name();
  } else {
    left_id = std::to_string(root->left->id());
  }
  if (root->right == nullptr) {
    right_id = root->right_name();
  } else {
    right_id = std::to_string(root->right->id());
  }

  fout << "\"" << root->id() << "\" -- \"" << left_id << "\"\n";
  fout << "\"" << root->id() << "\" -- \"" << right_id << "\"\n";

  preorder(root->left, level + 1, map, fout);
  preorder(root->right, level + 1, map, fout);
}

void BCTForest::traversal() {
  std::map<int, std::vector<int>> map;

  std::ofstream fout("bin/bct.dot");
  fout << "graph BCT {\n"
       << " ranksep=0.5\n"
       << " size=\"50,10\"\n"
       << " ratio=\"fill\"\n"
       << " edge[dir=none]\n"
       << " node[shape=\"box3d\", width=1.5, height=0.5, "
          "fontname=\"Arial\"]\n";

  for (const auto &root : trees) {
    preorder(root, 1, map, fout);
  }

  fout << "}\n";
}

std::unique_ptr<BCTNode> BCTForest::find_root_contains_node(
    const std::string &name) {
  auto itr_find = std::find_if(trees.begin(), trees.end(),
                               [&name](const std::unique_ptr<BCTNode> &p_node) {
                                 return p_node->contains_node(name);
                               });

  std::unique_ptr<BCTNode> root;
  if (itr_find != trees.end()) {
    root = std::move(*itr_find);

    std::forward_list<std::unique_ptr<BCTNode>> temp_trees;
    temp_trees.splice_after(temp_trees.before_begin(), trees,
                            trees.before_begin(), itr_find);
    trees.pop_front();
    trees.splice_after(trees.before_begin(), temp_trees);
  }

  return root;
}

void BCTForest::insert_root(std::unique_ptr<BCTNode> root) {
  trees.emplace_front(std::move(root));
}
