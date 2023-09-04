# cmake_minimum_required(VERSION 3.5)
# project(chrono_ros_interfaces)

# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   msg/ChVehicle.msg
# 	msg/ChDriverInputs.msg

#   srv/ChStartSimulation.srv
#   srv/ChStopSimulation.srv

#   DEPENDENCIES geometry_msgs std_msgs
# )

# set(INCLUDES "")
# list(APPEND INCLUDES ${CMAKE_BINARY_DIR}/src/chrono_ros/chrono_ros_interfaces/rosidl_generator_cpp/)
# list(APPEND INCLUDES ${std_msgs_INCLUDE_DIRS})
# list(APPEND INCLUDES ${geometry_msgs_INCLUDE_DIRS})
# set(CH_ROS_INCLUDES ${CH_ROS_INCLUDES} ${INCLUDES} PARENT_SCOPE)

# set(LIBRARIES)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# list(APPEND LIBRARIES ${cpp_typesupport_target})
# list(APPEND LIBRARIES ${std_msgs_LIBRARIES})
# list(APPEND LIBRARIES ${geometry_msgs_LIBRARIES})
# set(CH_ROS_LIBRARIES ${CH_ROS_LIBRARIES} ${LIBRARIES} PARENT_SCOPE)

cmake_minimum_required(VERSION 3.5)
project(chrono_ros_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Allow interfaces to be used for both C++ or Python
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	msg/ChDriverInputs.msg

  srv/ChStartSimulation.srv
  srv/ChStopSimulation.srv
  srv/ChResetSimulation.srv

  DEPENDENCIES geometry_msgs std_msgs
)


ament_package()