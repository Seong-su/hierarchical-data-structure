#ifndef CONNECTION_HPP_
#define CONNECTION_HPP_

#include "BoundingBox.hpp"

class Connection {
 public:
  explicit Connection(BoundingBox *const b1, BoundingBox *const b2);
  ~Connection();

  double attraction() const { return attraction_; }
  double cost() const { return cost_; }

 public:
  double attraction_, cost_;

 private:
  friend inline bool operator<(const Connection &e1, const Connection &e2) {
    return e1.cost() < e2.cost();
  }
};

#endif  // CONNECTION_HPP_