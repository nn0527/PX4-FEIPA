cmake_minimum_required(VERSION 3.0.2)
project(px4_wall_ceiling_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  # Unified attachment runtime contracts.
  AttachmentControlCandidate.msg
  AttachmentMechanismCommand.msg
  AttachmentMechanismStatus.msg
  CeilingAttachmentStatus.msg
  OperatorCommand.msg
  SensorHealth.msg
  WallPerchStatus.msg
  # Legacy generic distance-control contracts.
  BehaviorCommand.msg
  ControllerStatus.msg
  ControlSetpoint.msg
  SupervisorState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  mavros_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  mavros_msgs
  message_runtime
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  # Unified runtime.
  scripts/bench/ceiling_motor_bench_output_node.py
  scripts/decisions/ceiling_attachment_decision_node.py
  scripts/decisions/wall_perch_decision_node.py
  scripts/operator/operator_gate_node.py
  scripts/operator/terminal_operator_node.py
  scripts/operator/flight_mode_manager_node.py
  scripts/outputs/attachment_mechanism_output_node.py
  scripts/outputs/attachment_output_mux_node.py
  scripts/sensors/lidar_distance_node.py
  scripts/sensors/sensor_range_manager_node.py
  scripts/standalone/ceiling_attachment_output_node.py
  scripts/test_support/mock_attachment_mechanism_node.py
  scripts/test_support/mock_flight_inputs_node.py
  scripts/test_support/ceiling_attachment_state_monitor.py
  scripts/test_support/wall_perch_state_monitor.py
  # Legacy generic distance-control chain.
  scripts/legacy/ceiling_controller_node.py
  scripts/legacy/ceiling_fsm_node.py
  scripts/legacy/flight_supervisor_node.py
  scripts/legacy/setpoint_mux_node.py
  scripts/legacy/wall_controller_node.py
  scripts/legacy/wall_fsm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_attachment_safety.py)
  catkin_add_nosetests(test/test_ceiling_motor_bench.py)
  catkin_add_nosetests(test/test_ceiling_state_monitor.py)
  catkin_add_nosetests(test/test_flight_handover.py)
endif()
