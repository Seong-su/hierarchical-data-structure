#include <algorithm>

#include "BoundingBox.hpp"

BoundingBox::BoundingBox(int id, const std::string &name, aiMesh *mesh)
    : id_(id) {
  set_name(name);
  uint64_t num_vertices = mesh->mNumVertices;
  if (num_vertices > 0) {
    init_max(mesh->mVertices[0]);
    init_min(mesh->mVertices[0]);

    std::for_each(mesh->mVertices, mesh->mVertices + num_vertices,
                  [this](const aiVector3D &vertex) {
                    this->find_max(vertex);
                    this->find_min(vertex);
                  });

    find_center();
    find_radius();
  }
}

BoundingBox::BoundingBox(BoundingBox *const b1, BoundingBox *const b2) {
  set_name(b1->name_ + "-" + b2->name_);
  init_max(b1->max_);
  init_min(b1->min_);

  find_max(b2->max_);
  find_min(b2->min_);

  find_center();
  find_radius();
}

BoundingBox::~BoundingBox() {}

void BoundingBox::init_max(const aiVector3D &point) { max_ = point; }

void BoundingBox::init_min(const aiVector3D &point) { min_ = point; }

void BoundingBox::find_max(const aiVector3D &point) {
  if (max_.x < point.x) max_.x = point.x;
  if (max_.y < point.y) max_.y = point.y;
  if (max_.z < point.z) max_.z = point.z;
}

void BoundingBox::find_min(const aiVector3D &point) {
  if (min_.x > point.x) min_.x = point.x;
  if (min_.y > point.y) min_.y = point.y;
  if (min_.z > point.z) min_.z = point.z;
}

void BoundingBox::find_center() {
  center_ = (max_ + min_);
  center_ /= 2;
}

void BoundingBox::find_radius() {
  aiVector3D diameter_vec = max_ - min_;
  radius_ = diameter_vec.Length() / 2;
}
