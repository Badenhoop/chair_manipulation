#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_STATISTICS_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_STATISTICS_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <string>
#include <ros/ros.h>

namespace chair_manipulation
{
namespace statistics
{
inline double min(const std::vector<double>& v)
{
  auto it = std::min_element(v.begin(), v.end());
  return *it;
}

inline double max(const std::vector<double>& v)
{
  auto it = std::max_element(v.begin(), v.end());
  return *it;
}

inline double mean(const std::vector<double>& v)
{
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  return sum / v.size();
}

inline double stddev(const std::vector<double>& v)
{
  double v_mean = mean(v);
  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), [&](double x) { return x - v_mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return std::sqrt(sq_sum / v.size());
}

inline double median(std::vector<double>& v)
{
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}

inline void debugSummary(std::vector<double>& v, const std::string& label)
{
  ROS_DEBUG_STREAM_NAMED("statistics", "=== Summary of " << label << " ===");
  ROS_DEBUG_STREAM_NAMED("statistics", "\tmin: " << min(v));
  ROS_DEBUG_STREAM_NAMED("statistics", "\tmax: " << max(v));
  ROS_DEBUG_STREAM_NAMED("statistics", "\tmean: " << mean(v));
  ROS_DEBUG_STREAM_NAMED("statistics", "\tstddev: " << stddev(v));
  ROS_DEBUG_STREAM_NAMED("statistics", "\tmedian: " << median(v));
}

}  // namespace statistics
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_STATISTICS_H
