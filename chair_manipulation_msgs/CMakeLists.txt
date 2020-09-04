cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_msgs)

set(MSG_DEPS
  std_msgs
  actionlib_msgs
)

set(MSG_FILES
)

set(SRV_FILES
)

set(ACT_FILES
  GraspExecution.action
  GripperCommand.action
)

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

add_action_files(DIRECTORY action FILES ${ACT_FILES})
#add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPDS})
