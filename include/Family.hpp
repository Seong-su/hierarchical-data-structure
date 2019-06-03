#ifndef FAMILY_HPP_
#define FAMILY_HPP_

#include <cmath>
#include <unordered_set>
#include <vector>

class Family {
 private:
  double total_sum_;

  bool criteria(double weight);
  void add_member(double weight, int edge_id);

 public:
  Family(/* args */);
  ~Family();

  bool add(double weight, int edge_id);

  std::unordered_set<int> edge_id_;
  std::vector<double> weights_;
  double mean_value_, standard_deviation_;
};

#endif  // FAMILY_HPP_