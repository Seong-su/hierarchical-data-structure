#include "Family.hpp"

Family::Family(/* args */)
    : total_sum_{0.0}, mean_value_{0.0}, standard_deviation_{0.0} {}

Family::~Family() {}

void Family::add_member(double weight) {
  weights_.push_back(weight);
  total_sum_ += weight;
  mean_value_ = total_sum_ / weights_.size();

  double numerator = 0.0;
  for (const auto &weight_in_list : weights_) {
    numerator += (weight_in_list - mean_value_);
  }
  standard_deviation_ = std::sqrt(numerator / weights_.size());
}

bool Family::criteria(double weight) {
  return weight < mean_value_ + 2 * standard_deviation_;
}

bool Family::add(double weight) {
  if (weights_.size() == 0 || weights_.size() == 1) {
    add_member(weight);
    return true;
  }

  if (criteria(weight)) {
    add_member(weight);
    return true;
  }
  return false;
}