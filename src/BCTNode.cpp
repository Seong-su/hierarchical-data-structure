#include "BCTNode.hpp"

// BCTNode

BCTNode::BCTNode(const std::string &source_node, const std::string &target_node,
                 int id)
    : parent(nullptr), left(nullptr), right(nullptr), id_(id) {
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

void BCTForest::insert(BCTNode *node) {
  const std::string &source_node = node->left_name();
  const std::string &target_node = node->right_name();

  BCTNode *source_root = find_root_contains_node(source_node);
  BCTNode *target_root = find_root_contains_node(target_node);

  if (source_root && target_root) {
    node->left = source_root;
    node->right = target_root;

    node->name_content() = source_root->name_content();
    node->name_content().insert(target_root->name_content().begin(),
                                target_root->name_content().end());

    source_root->parent = node;
    target_root->parent = node;

    erase_root(source_root);
    erase_root(target_root);
    insert_root(node);
  } else if (source_root) {
    node->left = source_root;

    node->name_content() = source_root->name_content();
    node->name_content().insert(target_node);

    source_root->parent = node;

    erase_root(source_root);
    insert_root(node);
  } else if (target_root) {
    node->right = target_root;

    node->name_content() = target_root->name_content();
    node->name_content().insert(source_node);

    target_root->parent = node;

    erase_root(target_root);
    insert_root(node);
  } else {
    insert_root(node);
  }
}

void BCTForest::preorder(BCTNode *root, int level,
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

  // for (auto it : map) {
  //   fout << "Level " << it.first << ": ";
  //   for (const auto &i : it.second) fout << i << " ";

  //   fout << "\n";
  // }

  fout << "}\n";
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

  trees.remove(root);
}

void BCTForest::insert_root(BCTNode *root) { trees.emplace_front(root); }
