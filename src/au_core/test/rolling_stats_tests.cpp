#include <au_core/rolling_stats.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

using namespace au_core;

std::vector<double> test_data = {
    64,  172, 32,  153, 134, 162, 178, 10,  28,  141, 16,  68,  86,  5,   114,
    81,  47,  106, 135, 69,  59,  71,  158, 148, 39,  190, 57,  168, 101, 22,
    182, 82,  154, 8,   166, 108, 103, 156, 97,  87,  118, 21,  13,  102, 56,
    132, 20,  165, 107, 155, 194, 199, 74,  187, 192, 11,  51,  125, 160, 7,
    46,  52,  43,  48,  139, 80,  90,  70,  188, 55,  41,  138, 26,  88,  50,
    15,  176, 145, 98,  159, 195, 94,  58,  61,  33,  119, 128, 112, 151, 49,
    117, 130, 100, 99,  189, 161, 19,  76,  171, 12,  174, 197, 60,  167, 45,
    14,  120, 31,  96,  140, 121, 92,  115, 196, 169, 37,  198, 149, 175, 124,
    34,  129, 84,  109, 123, 75,  18,  104, 36,  24,  38,  44,  66,  177, 91,
    77,  180, 183, 131, 122, 126, 89,  42,  105, 110, 95,  9,   72,  54,  144,
    35,  136, 137, 170, 127, 173, 23,  185, 163, 62,  113, 142, 30,  191, 111,
    53,  116, 193, 164, 143, 179, 83,  157, 27,  133, 146, 181, 17,  150, 73,
    63,  184, 186, 78,  147, 79,  6,   29,  40,  93,  85,  65,  67,  152, 25};
const double abs_error = 0.0000001;

void loadVector(RollingStats& rollingSum, std::vector<double>& v) {
  for (auto it = v.begin(); it != v.end(); ++it) {
    rollingSum.push(*it);
  }
}

double mean(std::vector<double>& v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();
  return mean;
}

double variance(std::vector<double>& v) {
  double m = mean(v);
  double stdDev = 0;
  for (auto it = v.begin(); it != v.end(); ++it) {
    stdDev += std::pow((*it) - m, 2);
  }
  return stdDev / (v.size() - 1);
}

TEST(RollingStatsTest, reset) {
  au_core::RollingStats rollingStats(5);
  loadVector(rollingStats, test_data);
  rollingStats.reset();
  EXPECT_NEAR(rollingStats.average(), 0, abs_error);
  EXPECT_NEAR(rollingStats.variance(), 0, abs_error);
  EXPECT_NEAR(rollingStats.stddev(), 0, abs_error);
}

TEST(RollingStatsTest, mean) {
  au_core::RollingStats rollingStats(test_data.size());
  loadVector(rollingStats, test_data);
  EXPECT_NEAR(rollingStats.average(), mean(test_data), abs_error);
}

TEST(RollingStatsTest, mean_overflow) {
  au_core::RollingStats rollingStats(50);
  loadVector(rollingStats, test_data);
  std::vector<double> used_test_data(test_data.end() - 50, test_data.end());
  EXPECT_NEAR(rollingStats.average(), mean(used_test_data), abs_error);
}

TEST(RollingStatsTest, variance) {
  au_core::RollingStats rollingStats(test_data.size());
  loadVector(rollingStats, test_data);
  EXPECT_NEAR(rollingStats.variance(), variance(test_data), abs_error);
}

TEST(RollingStatsTest, variance_overflow) {
  au_core::RollingStats rollingStats(50);
  loadVector(rollingStats, test_data);
  std::vector<double> used_test_data(test_data.end() - 50, test_data.end());
  EXPECT_NEAR(rollingStats.variance(), variance(used_test_data), abs_error);
}

TEST(RollingStatsTest, stddev) {
  au_core::RollingStats rollingStats(test_data.size());
  loadVector(rollingStats, test_data);
  EXPECT_NEAR(rollingStats.stddev(), std::sqrt(variance(test_data)), abs_error);
}

TEST(RollingStatsTest, stddev_overflow) {
  au_core::RollingStats rollingStats(50);
  loadVector(rollingStats, test_data);
  std::vector<double> used_test_data(test_data.end() - 50, test_data.end());
  EXPECT_NEAR(rollingStats.stddev(), std::sqrt(variance(used_test_data)),
              abs_error);
}

// Run all the tests
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
