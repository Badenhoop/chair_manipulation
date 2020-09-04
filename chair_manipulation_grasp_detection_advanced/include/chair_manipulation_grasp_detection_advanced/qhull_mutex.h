#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_QHULL_MUTEX_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_QHULL_MUTEX_H

#include <mutex>

namespace chair_manipulation
{
/**
 * The global mutex used to synchronize access to QHull
 */
extern std::mutex qhull_mutex;

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_QHULL_MUTEX_H
