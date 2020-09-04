#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_EXCEPTION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_EXCEPTION_H

#include <stdexcept>

namespace chair_manipulation
{
namespace exception
{
struct Runtime : std::runtime_error
{
  explicit Runtime(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

struct IO : Runtime
{
  explicit IO(const std::string& msg) : Runtime(msg)
  {
  }
};

struct Parameter : Runtime
{
  explicit Parameter(const std::string& msg) : Runtime(msg)
  {
  }
};

struct IllegalState : Runtime
{
  explicit IllegalState(const std::string& msg) : Runtime(msg)
  {
  }
};

struct IllegalArgument : Runtime
{
  explicit IllegalArgument(const std::string& msg) : Runtime(msg)
  {
  }
};

struct PointCloud : Runtime
{
  explicit PointCloud(const std::string& msg) : Runtime(msg)
  {
  }
};

struct TF : Runtime
{
  explicit TF(const std::string& msg) : Runtime(msg)
  {
  }
};

}  // namespace exception
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_EXCEPTION_H
