# Note that this does not find any ros2 package in particular, but determines whether a distro is sourced properly
# Can be extended to handle supported / unsupported distros

unset(ROS2_FOUND)

if (NOT DEFINED ENV{ROS_DISTRO} OR NOT DEFINED ENV{AMENT_PREFIX_PATH})   
    set(ROS2_FOUND FALSE)
    return()
endif()

set(ROS2_FOUND TRUE)

# set all the new variables from ros as advanced
mark_as_advanced(FORCE 
    AMENT_CMAKE_ENVIRONMENT_GENERATION
    AMENT_CMAKE_ENVIRONMENT_PACKAGE_GENERATION
    AMENT_CMAKE_ENVIRONMENT_PARENT_PREFIX_PATH_GENERATION
    AMENT_CMAKE_SYMLINK_INSTALL
    AMENT_CMAKE_UNINSTALL_TARGET
    AMENT_TEST_RESULTS_DIR
    FastCDR_LIBRARY_DEBUG
    FastCDR_LIBRARY_RELEASE
    FastRTPS_INCLUDE_DIR
    FastRTPS_LIBRARY_DEBUG
    FastRTPS_LIBRARY_RELEASE
    _lib
    action_msgs_DIR
    ament_cmake_DIR
    ament_cmake_core_DIR
    ament_cmake_export_definitions_DIR
    ament_cmake_export_dependencies_DIR
    ament_cmake_export_include_directories_DIR
    ament_cmake_export_interfaces_DIR
    ament_cmake_export_libraries_DIR
    ament_cmake_export_link_flags_DIR
    ament_cmake_export_targets_DIR
    ament_cmake_gen_version_h_DIR
    ament_cmake_include_directories_DIR
    ament_cmake_libraries_DIR
    ament_cmake_python_DIR
    ament_cmake_target_dependencies_DIR
    ament_cmake_test_DIR
    ament_cmake_version_DIR
    ament_index_cpp_DIR
    builtin_interfaces_DIR
    chrono_ros_interfaces_DIR
    class_loader_DIR
    composition_interfaces_DIR
    fastcdr_DIR
    fastrtps_DIR
    fastrtps_cmake_module_DIR
    fmt_DIR
    foonathan_memory_DIR
    geometry_msgs_DIR
    libstatistics_collector_DIR
    libyaml_vendor_DIR
    message_filters_DIR
    rcl_DIR
    rcl_action_DIR
    rcl_interfaces_DIR
    rcl_logging_interface_DIR
    rcl_logging_spdlog_DIR
    rcl_yaml_param_parser_DIR
    rclcpp_DIR
    rclcpp_action_DIR
    rclcpp_components_DIR
    rcpputils_DIR
    rcutils_DIR
    rmw_DIR
    rmw_dds_common_DIR
    rmw_fastrtps_cpp_DIR
    rmw_fastrtps_shared_cpp_DIR
    rmw_implementation_DIR
    rmw_implementation_cmake_DIR
    rosgraph_msgs_DIR
    rosidl_adapter_DIR
    rosidl_cmake_DIR
    rosidl_default_runtime_DIR
    rosidl_generator_c_DIR
    rosidl_generator_cpp_DIR
    rosidl_runtime_c_DIR
    rosidl_runtime_cpp_DIR
    rosidl_typesupport_c_DIR
    rosidl_typesupport_cpp_DIR
    rosidl_typesupport_fastrtps_c_DIR
    rosidl_typesupport_fastrtps_cpp_DIR
    rosidl_typesupport_interface_DIR
    rosidl_typesupport_introspection_c_DIR
    rosidl_typesupport_introspection_cpp_DIR
    sensor_msgs_DIR
    spdlog_DIR
    spdlog_vendor_DIR
    statistics_msgs_DIR
    std_msgs_DIR
    tf2_DIR
    tf2_msgs_DIR
    tf2_ros_DIR
    tracetools_DIR
    yaml_DIR
    unique_identifier_msgs_DIR
)
