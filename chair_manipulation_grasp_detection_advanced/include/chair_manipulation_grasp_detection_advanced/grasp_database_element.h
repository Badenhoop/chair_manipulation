#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_ELEMENT_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_ELEMENT_H

#include "model.h"
#include "multi_arm_grasp.h"

namespace chair_manipulation
{
struct GraspDatabaseElement
{
  std::string mesh_filename_;
  std::string point_cloud_filename_;
  ModelConstPtr model_;
  std::vector<MultiArmGrasp> grasps_;
};

using GraspDatabaseElementPtr = std::shared_ptr<GraspDatabaseElement>;
using GraspDatabaseElementConstPtr = std::shared_ptr<GraspDatabaseElement>;

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_ELEMENT_H
