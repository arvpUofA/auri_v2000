#ifndef __ROLL_AVG__
#define __ROLL_AVG__

#include <deque>
#include <iostream>

namespace au_core {

class RollingStats {
 public:
  explicit RollingStats(size_t);

  void push(double);

  double last() const;
  double average() const;
  double variance() const;
  double stddev() const;

  void reset();
  void reset(size_t);

 private:
  std::deque<double> queue_;
  size_t queue_size_;
  double mean_;
  double sum_squared_;

  void addNew(double);
  void removeOld();
};

}  // namespace au_core

#endif
