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
  std_srvs
)

catkin_python_setup()

add_message_files(
  FILES
  # Unified attachment runtime contracts.
  AttachmentControlCandidate.msg
  AttachmentMechanismCommand.msg
  AttachmentMechanismStatus.msg
  CeilingAttachmentStatus.msg
  CeilingRcAssist.msg
  OperatorCommand.msg
  SensorHealth.msg
  WallPerchStatus.msg
  WallSensorFusionStatus.msg
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
  std_srvs
)

catkin_install_python(PROGRAMS
  # Unified runtime.
  scripts/decisions/ceiling_attachment_decision_node.py
  scripts/decisions/wall_perch_decision_node.py
  scripts/operator/rc_takeover.py
  scripts/operator/terminal_operator_node.py
  scripts/operator/ceiling_failsafe_guard_node.py
  scripts/operator/flight_mode_manager_node.py
  scripts/operator/wall_console_server.py
  scripts/operator/ceiling_wall_console.py
  scripts/outputs/attachment_mechanism_output_node.py
  scripts/outputs/attachment_output_mux_node.py
  scripts/sensors/lidar_distance_node.py
  scripts/sensors/sensor_range_manager_node.py
  scripts/sensors/wall_sensor_fusion_manager_node.py
  scripts/test_support/attachment_flight_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_recover_retreat.py)
  catkin_add_nosetests(test/test_ceiling_wall_console.py)
  catkin_add_nosetests(test/test_rc_takeover.py)
  catkin_add_nosetests(test/test_detach_vertical.py)
  catkin_add_nosetests(test/test_detach_support_handoff.py)
  catkin_add_nosetests(test/test_support_then_range.py)
  catkin_add_nosetests(test/test_departure_demo.py)
  catkin_add_nosetests(test/test_range_height_feedback.py)
  catkin_add_nosetests(test/test_range_height_integration.py)
  catkin_add_nosetests(test/test_hold_config_latency.py)
  catkin_add_nosetests(test/test_detach_output_race.py)
  catkin_add_nosetests(test/test_height_continuation.py)
  catkin_add_nosetests(test/test_bounded_detach_integration.py)
  catkin_add_nosetests(test/test_detach_vertical_authority.py)
  catkin_add_nosetests(test/test_detach_state_machine.py)
  catkin_add_nosetests(test/test_detach_braking.py)
  catkin_add_nosetests(test/test_detach_braking_integration.py)
  catkin_add_nosetests(test/test_detach_braking_replay.py)
  catkin_add_nosetests(test/test_detach_output_continuity.py)
  catkin_add_nosetests(test/test_recovery_horizontal_departure.py)
  catkin_add_nosetests(test/test_source_freshness.py)
  catkin_add_nosetests(test/test_candidate_latency.py)
  catkin_add_nosetests(test/test_flip_latency.py)
  catkin_add_nosetests(test/test_flip_vertical_support.py)
  catkin_add_nosetests(test/test_approach_takeover_thrust.py)
  catkin_add_nosetests(test/test_async_logging.py)
  catkin_add_nosetests(test/test_attachment_range_loss.py)
  catkin_add_nosetests(test/test_range_error_258.py)
  catkin_add_nosetests(test/test_detach_current_attitude.py)
  catkin_add_nosetests(test/test_hold_down_velocity.py)
  catkin_add_nosetests(test/test_manual_wall_recovery.py)
  catkin_add_nosetests(test/test_hold_range_loss.py)
  catkin_add_nosetests(test/test_sensor_error_reporting.py)
  catkin_add_nosetests(test/test_attachment_safety.py)
  catkin_add_nosetests(test/test_flight_handover.py)
  catkin_add_nosetests(test/test_ceiling_rc_assist.py)
  catkin_add_nosetests(test/test_auto_wall_approach.py)
  catkin_add_nosetests(test/test_single_frame_wall_flip.py)
  catkin_add_nosetests(test/test_front_only_late_top.py)
  catkin_add_nosetests(test/test_recovery_attitude_limits.py)
  catkin_add_nosetests(test/test_straight_retreat.py)
  catkin_add_nosetests(test/test_straight_retreat_integration.py)
  catkin_add_nosetests(test/test_attachment_flight_logger.py)
  catkin_add_nosetests(test/test_wall_console.py)
  catkin_add_nosetests(test/test_ceiling_console_flow.py)
  catkin_add_nosetests(test/test_ceiling_approach_timeout.py)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_wall_horizontal_velocity.py)
  catkin_add_nosetests(test/test_horizontal_equilibrium.py)
  catkin_add_nosetests(test/test_horizontal_pi.py)
  catkin_add_nosetests(test/test_flow_timestamps.py)
  catkin_add_nosetests(test/test_wall_timeout_handover.py)
  catkin_add_nosetests(test/test_wall_sensor_fusion.py)
  catkin_add_nosetests(test/test_wall_fusion_handover.py)
  catkin_add_nosetests(test/test_fusion_async_io.py)
  catkin_add_nosetests(test/test_fusion_dispatch_recovery.py)
  catkin_add_nosetests(test/test_horizontal_reacquisition_epoch.py)
  catkin_add_nosetests(test/test_range_recovery_damping.py)
  catkin_add_nosetests(test/test_departure_output_timing.py)
  catkin_add_nosetests(test/test_range_control_gain.py)
  catkin_add_nosetests(test/test_recovery_continuous_control.py)
  catkin_add_nosetests(test/test_recovery_tilt_guard.py)
  catkin_add_nosetests(test/test_recovery_a_response.py)
  catkin_add_nosetests(test/test_rc_wall_toggle.py)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_ceiling_baseline_tuning.py)
  catkin_add_nosetests(test/test_ceiling_ch12_handover.py)
  catkin_add_nosetests(test/test_ceiling_closed_loop_ascent.py)
  catkin_add_nosetests(test/test_ceiling_detach_height_velocity.py)
  catkin_add_nosetests(test/test_ceiling_entry_ownership.py)
  catkin_add_nosetests(test/test_ceiling_fault_land.py)
  catkin_add_nosetests(test/test_ceiling_flow_ascent.py)
  catkin_add_nosetests(test/test_ceiling_fused_height.py)
  catkin_add_nosetests(test/test_ceiling_hold_abort.py)
  catkin_add_nosetests(test/test_ceiling_hold_link_guard.py)
  catkin_add_nosetests(test/test_ceiling_motion.py)
  catkin_add_nosetests(test/test_ceiling_owner_confirmation.py)
  catkin_add_nosetests(test/test_ceiling_rc_coordination.py)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_rc_startup_gesture.py)
  catkin_add_nosetests(test/test_rc_startup_business_stop.py)
  catkin_add_nosetests(test/test_rc_stop_ch12.py)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_incline_attachment.py)
  catkin_add_nosetests(test/test_incline_integration.py)
  catkin_add_nosetests(test/test_incline_return_parity.py)
  catkin_add_nosetests(test/test_incline_approach_parity.py)
  catkin_add_nosetests(test/test_incline_console.py)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_console_only_rc.py)
endif()
