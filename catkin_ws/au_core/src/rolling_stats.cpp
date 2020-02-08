#include <au_core/rolling_stats.hpp>
#include <algorithm>
#include <cmath>

namespace au_core {

RollingStats::RollingStats(size_t size) : queue_size_(size) { reset(); }

void RollingStats::reset() {
  sum_squared_ = 0;
  mean_ = 0;
}

void RollingStats::reset(size_t size) {
  queue_size_ = size;
  // clear queue
  std::deque<double> empty;
  std::swap(queue_, empty);
  reset();
}

void RollingStats::push(double new_num) {
  if (queue_.size() >= queue_size_) {
    removeOld();
  }
  addNew(new_num);
}

void RollingStats::addNew(double new_num) {
  queue_.push_back(new_num);
  double delta = new_num - mean_;
  double newMean = mean_ + delta / queue_.size();
  sum_squared_ += delta * (new_num - newMean);
  mean_ = newMean;
}

void RollingStats::removeOld() {
  // remove an element
  double old = queue_.front();

  double newMean = (queue_.size() * mean_ - old) / (queue_.size() - 1);
  sum_squared_ -= (old - mean_) * (old - newMean);
  mean_ = newMean;
  queue_.pop_front();
}

double RollingStats::last() const {
  if (queue_.empty()) return 0;
  return queue_.back();
}

double RollingStats::average() const { return mean_; }

double RollingStats::variance() const {
  if (queue_.size() <= 1) return 0;
  return sum_squared_ / (queue_.size() - 1);
}

double RollingStats::stddev() const { return std::sqrt(variance()); }

}  // namespace au_core
