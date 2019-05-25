#ifndef FAMILY_HPP_
#define FAMILY_HPP_

#include <cmath>
#include <vector>

class Family {
 private:
  double total_sum_;

  bool criteria(double weight);
  void add_member(double weight);

 public:
  Family(/* args */);
  ~Family();

  bool add(double weight);

  std::vector<double> weights_;
  double mean_value_, standard_deviation_;
};

#endif  // FAMILY_HPP_