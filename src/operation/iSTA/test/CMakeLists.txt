if(BUILD_TESTING)
  add_executable(ista_driver_fixture_probe DriverFixtureProbe.cpp)
  target_link_libraries(ista_driver_fixture_probe PRIVATE ista_delay_calculator)

  add_executable(ista_min_slew_config_test MinSlewConfigTest.cpp
      ${PROJECT_SOURCE_DIR}/src/interface/python/py_ista/py_ista_utils.cpp)
  target_include_directories(ista_min_slew_config_test PRIVATE
      ${PROJECT_SOURCE_DIR}/src/interface/python/py_ista ${PROJECT_SOURCE_DIR}/src/utility/json)
  target_link_libraries(ista_min_slew_config_test PRIVATE tcl_ista ista_interface)
  add_test(NAME ista_min_slew_config_test COMMAND ista_min_slew_config_test)

  add_executable(ista_min_slew_policy_test MinSlewPolicyTest.cpp)
  target_link_libraries(ista_min_slew_policy_test PRIVATE ista_delay_calculator)
  add_test(NAME ista_min_slew_policy_test COMMAND ista_min_slew_policy_test)

  add_executable(ista_input_port_waveform_test InputPortWaveformTest.cpp)
  target_link_libraries(ista_input_port_waveform_test PRIVATE ista_delay_calculator)
  add_test(NAME ista_input_port_waveform_test COMMAND ista_input_port_waveform_test)

  add_executable(ista_sdf_data_slew_test SDFDataSlewTest.cpp)
  target_link_libraries(ista_sdf_data_slew_test PRIVATE ista_sdf_writer)
  add_test(NAME ista_sdf_data_slew_test COMMAND ista_sdf_data_slew_test)

  add_executable(ista_sdf_conditional_period_test SDFConditionalPeriodTest.cpp)
  target_link_libraries(ista_sdf_conditional_period_test PRIVATE ista_sdf_writer)
  add_test(NAME ista_sdf_conditional_period_test COMMAND ista_sdf_conditional_period_test)

  add_executable(ista_liberty_bus_timing_test LibertyBusTimingTest.cpp)
  target_link_libraries(ista_liberty_bus_timing_test PRIVATE liberty)
  add_test(NAME ista_liberty_bus_timing_test COMMAND ista_liberty_bus_timing_test)

  add_executable(ista_liberty_fanout_test LibertyFanoutTest.cpp)
  target_link_libraries(ista_liberty_fanout_test PRIVATE liberty)
  add_test(NAME ista_liberty_fanout_test COMMAND ista_liberty_fanout_test)

  add_executable(ista_sdc_timing_semantics_test SdcTimingSemanticsTest.cpp)
  target_link_libraries(ista_sdc_timing_semantics_test PRIVATE
      ista_sdc_command ista_clock_propagator ista_timing_propagator ista_timing_analyzer ista_timing_reporter)
  add_test(NAME ista_sdc_timing_semantics_test COMMAND ista_sdc_timing_semantics_test)

  add_executable(ista_arc_timing_arc_delay_test
      ArcTimingArcDelayTest.cpp
  )

  target_link_libraries(ista_arc_timing_arc_delay_test
      PRIVATE
      ista_data_manager
  )

  add_test(NAME ista_arc_timing_arc_delay_test COMMAND ista_arc_timing_arc_delay_test)
endif()
