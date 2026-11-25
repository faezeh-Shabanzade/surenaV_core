cmake_minimum_required(VERSION 3.0.2)
project(surenaV_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  position_controllers
  velocity_controllers
  joint_trajectory_controller
  robot_state_publisher
  rospy
  std_msgs
)

catkin_package()

# ---------------------------------------------------------------------------
# Generate the position/velocity URDF variants from the mechanical source of
# truth (surenaV_description/urdf/surenaV.urdf = full body + hands) at build
# time. Only the <hardwareInterface> tags inside the transmissions differ.
# ---------------------------------------------------------------------------
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/urdf/surenaV_position.urdf
         ${CMAKE_CURRENT_SOURCE_DIR}/urdf/surenaV_velocity.urdf
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_urdf_variants.py
          --source ${CMAKE_CURRENT_SOURCE_DIR}/../surenaV_description/urdf/surenaV.urdf
          --outdir ${CMAKE_CURRENT_SOURCE_DIR}/urdf
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../surenaV_description/urdf/surenaV.urdf
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_urdf_variants.py
  COMMENT "Generating surenaV position/velocity URDF variants"
)
add_custom_target(surenaV_urdf_variants ALL
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/surenaV_position.urdf
          ${CMAKE_CURRENT_SOURCE_DIR}/urdf/surenaV_velocity.urdf
)

# ---------------------------------------------------------------------------
# Install scripts (ROS nodes and the PyBullet control library).
# ---------------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/pybullet_ros_node.py
  scripts/pybullet_position_control.py
  scripts/pybullet_velocity_control.py
  scripts/switch_control_mode.py
  scripts/generate_urdf_variants.py
  scripts/validate_urdf_and_configs.py
  scripts/demo_arm_movement.py
  scripts/arm_movement_demo.py
  scripts/arm_trajectory_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  config/position_controllers.yaml
  config/velocity_controllers.yaml
  config/trajectory_controllers.yaml
  config/joint_limits.yaml
  launch/gazebo_position.launch
  launch/gazebo_velocity.launch
  launch/pybullet_position.launch
  launch/pybullet_velocity.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ---------------------------------------------------------------------------
# HARDWARE INTERFACE TEMPLATE (real robot).
# Uncomment once src/hardware_interface/epos_driver_stub.h has a real
# implementation (see src/hardware_interface/HARDWARE_INFO_NEEDED.md):
#
# find_package(catkin REQUIRED COMPONENTS hardware_interface joint_limits_interface)
# include_directories(${catkin_INCLUDE_DIRS})
# add_library(surenaV_hw_interface
#   src/hardware_interface/surenaV_hw_interface.cpp)
# add_dependencies(surenaV_hw_interface ${catkin_EXPORTED_TARGETS})
# ---------------------------------------------------------------------------
