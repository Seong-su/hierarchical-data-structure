#ifndef BOUNDINGBOX_HPP_
#define BOUNDINGBOX_HPP_

#include <string>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

class BoundingBox {
 public:
  explicit BoundingBox(int id, const std::string &name, aiMesh *mesh);
  explicit BoundingBox(BoundingBox *const b1, BoundingBox *const b2);
  ~BoundingBox();

  static inline int id(const BoundingBox &box) { return box.id_; }
  static inline std::string name(const BoundingBox &box) { return box.name_; }
  inline aiVector3D max() const { return max_; }
  inline aiVector3D min() const { return min_; }
  inline aiVector3D center() const { return center_; }
  inline double radius() const { return radius_; }

 private:
  inline void set_name(const std::string &name) { name_ = name; }

  void init_max(const aiVector3D &point);
  void init_min(const aiVector3D &point);
  void find_max(const aiVector3D &point);
  void find_min(const aiVector3D &point);
  void find_center();
  void find_radius();
  
 public:
  int id_;
  std::string name_;
  aiVector3D max_, min_, center_;
  double radius_;
};

#endif  // BOUNDINGBOX_HPP_