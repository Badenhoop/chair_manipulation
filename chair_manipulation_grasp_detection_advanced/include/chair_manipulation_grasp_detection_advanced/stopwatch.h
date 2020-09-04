#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_STOPWATCH_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_STOPWATCH_H

#include <chrono>

namespace chair_manipulation
{
class Stopwatch
{
public:
  using Clock = std::chrono::high_resolution_clock;
  using Time = Clock::time_point;

  void start()
  {
    start_ = Clock::now();
  }

  void stop()
  {
    stop_ = Clock::now();
  }

  double elapsedSeconds() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(stop_ - start_).count() / 1000.;
  }

private:
  Time start_;
  Time stop_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_STOPWATCH_H
