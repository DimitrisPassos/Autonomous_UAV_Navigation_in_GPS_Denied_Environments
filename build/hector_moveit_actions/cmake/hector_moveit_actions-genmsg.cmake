# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hector_moveit_actions: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ihector_moveit_actions:/home/dimitris/thesis/devel/share/hector_moveit_actions/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hector_moveit_actions_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" "hector_moveit_actions/ExecuteDroneTrajectoryGoal:hector_moveit_actions/ExecuteDroneTrajectoryActionResult:hector_moveit_actions/ExecuteDroneTrajectoryActionGoal:geometry_msgs/Point:actionlib_msgs/GoalStatus:hector_moveit_actions/ExecuteDroneTrajectoryActionFeedback:geometry_msgs/Quaternion:actionlib_msgs/GoalID:geometry_msgs/Pose:hector_moveit_actions/ExecuteDroneTrajectoryFeedback:hector_moveit_actions/ExecuteDroneTrajectoryResult:std_msgs/Header"
)

get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" "hector_moveit_actions/ExecuteDroneTrajectoryGoal:geometry_msgs/Point:geometry_msgs/Quaternion:actionlib_msgs/GoalID:geometry_msgs/Pose:std_msgs/Header"
)

get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" "hector_moveit_actions/ExecuteDroneTrajectoryResult:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" "actionlib_msgs/GoalStatus:geometry_msgs/Point:geometry_msgs/Quaternion:actionlib_msgs/GoalID:geometry_msgs/Pose:hector_moveit_actions/ExecuteDroneTrajectoryFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" ""
)

get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" NAME_WE)
add_custom_target(_hector_moveit_actions_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hector_moveit_actions" "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_cpp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
)

### Generating Services

### Generating Module File
_generate_module_cpp(hector_moveit_actions
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hector_moveit_actions_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hector_moveit_actions_generate_messages hector_moveit_actions_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_cpp _hector_moveit_actions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hector_moveit_actions_gencpp)
add_dependencies(hector_moveit_actions_gencpp hector_moveit_actions_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hector_moveit_actions_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_eus(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
)

### Generating Services

### Generating Module File
_generate_module_eus(hector_moveit_actions
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hector_moveit_actions_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hector_moveit_actions_generate_messages hector_moveit_actions_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_eus _hector_moveit_actions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hector_moveit_actions_geneus)
add_dependencies(hector_moveit_actions_geneus hector_moveit_actions_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hector_moveit_actions_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_lisp(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
)

### Generating Services

### Generating Module File
_generate_module_lisp(hector_moveit_actions
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hector_moveit_actions_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hector_moveit_actions_generate_messages hector_moveit_actions_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_lisp _hector_moveit_actions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hector_moveit_actions_genlisp)
add_dependencies(hector_moveit_actions_genlisp hector_moveit_actions_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hector_moveit_actions_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_nodejs(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hector_moveit_actions
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hector_moveit_actions_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hector_moveit_actions_generate_messages hector_moveit_actions_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_nodejs _hector_moveit_actions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hector_moveit_actions_gennodejs)
add_dependencies(hector_moveit_actions_gennodejs hector_moveit_actions_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hector_moveit_actions_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)
_generate_msg_py(hector_moveit_actions
  "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
)

### Generating Services

### Generating Module File
_generate_module_py(hector_moveit_actions
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hector_moveit_actions_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hector_moveit_actions_generate_messages hector_moveit_actions_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryAction.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryGoal.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryResult.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dimitris/thesis/devel/share/hector_moveit_actions/msg/ExecuteDroneTrajectoryFeedback.msg" NAME_WE)
add_dependencies(hector_moveit_actions_generate_messages_py _hector_moveit_actions_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hector_moveit_actions_genpy)
add_dependencies(hector_moveit_actions_genpy hector_moveit_actions_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hector_moveit_actions_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hector_moveit_actions
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(hector_moveit_actions_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hector_moveit_actions_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hector_moveit_actions
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(hector_moveit_actions_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hector_moveit_actions_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hector_moveit_actions
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(hector_moveit_actions_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hector_moveit_actions_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hector_moveit_actions
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(hector_moveit_actions_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hector_moveit_actions_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hector_moveit_actions
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(hector_moveit_actions_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hector_moveit_actions_generate_messages_py geometry_msgs_generate_messages_py)
endif()
