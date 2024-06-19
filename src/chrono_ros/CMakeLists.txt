#=============================================================================
# CMake configuration file for Chrono::ROS module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_ROS "Enable the Chrono::ROS module" OFF)

IF(NOT ENABLE_MODULE_ROS)
  return()
ENDIF()

message(STATUS "\n==== Chrono::ROS module ====\n")

set(CXX_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")
set(CH_ROS_INCLUDES "")
set(CH_ROS_LIBRARIES "ChronoEngine")
set(CH_ROS_LINKED_LIBRARIES "")

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# Find ROS
#-----------------------------------------------------------------------------

find_package(ROS2 QUIET)
if(ROS2_FOUND)
  message(STATUS "Found ROS 2.")
  message(STATUS "  ROS 2 distro is \"$ENV{ROS_DISTRO}\"")
else()
  message(SEND_ERROR "Chrono::ROS requires ROS 2, but ROS 2 was not found.")
  return()
endif()

# Update the includes directories to point to ROS includes
set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")
foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
    string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
    list(APPEND CH_ROS_INCLUDES "${_ros2_packages_path}/include")
endforeach()

# Find ROS packages
set(_required_ros_packages rclcpp rcl_interfaces std_msgs geometry_msgs sensor_msgs tf2_ros tf2_msgs)
set(_optional_ros_packages chrono_ros_interfaces)

foreach(_package IN LISTS _required_ros_packages)
  find_package(${_package} QUIET)
  if(NOT ${${_package}_FOUND})
    message(FATAL_ERROR "Could not find ROS package ${_package}")
  endif()
  list(APPEND CH_ROS_LINKED_LIBRARIES "${${_package}_LIBRARIES}")
  list(APPEND CH_ROS_INCLUDES "${${_package}_INCLUDE_DIRS}")
endforeach()
foreach(_package IN LISTS _optional_ros_packages)
  find_package(${_package} QUIET)
  if(NOT ${${_package}_FOUND})
    message(WARNING "Could not find ROS package ${_package}. Some features will be disabled.")
  else()
    list(APPEND CH_ROS_LINKED_LIBRARIES "${${_package}_LIBRARIES}")
    list(APPEND CH_ROS_INCLUDES "${${_package}_INCLUDE_DIRS}")
  endif()
endforeach()

set(CH_ROS_HAS_INTERFACES ${chrono_ros_interfaces_FOUND})

list(REMOVE_DUPLICATES CH_ROS_INCLUDES)
list(REMOVE_DUPLICATES CH_ROS_LINKED_LIBRARIES)

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigROS.h.in
	${PROJECT_BINARY_DIR}/chrono_ros/ChConfigROS.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_ros/ChConfigROS.h"
		DESTINATION include/chrono_ros)

# ------------
# Common Files
# ------------

set(CH_ROS_BASE_FILES
	ChApiROS.h

  ChROSManager.h
	ChROSManager.cpp

  ChROSInterface.h
  ChROSInterface.cpp
)
source_group("base" FILES ${CH_ROS_BASE_FILES})

set(CH_ROS_HANDLER_FILES
  ChROSHandler.h
  ChROSHandler.cpp

  handlers/ChROSClockHandler.h
  handlers/ChROSClockHandler.cpp
  handlers/ChROSBodyHandler.h
  handlers/ChROSBodyHandler.cpp
  handlers/ChROSTFHandler.h
  handlers/ChROSTFHandler.cpp

  handlers/ChROSHandlerUtilities.h
  handlers/ChROSHandlerUtilities.cpp
)
source_group("handler" FILES ${CH_ROS_HANDLER_FILES})

set(CH_ROS_VEHICLE_HANDLER_FILES "")
if (ENABLE_MODULE_VEHICLE)
  if (CH_ROS_HAS_INTERFACES)
    list(APPEND CH_ROS_VEHICLE_HANDLER_FILES 
      handlers/vehicle/ChROSDriverInputsHandler.h
      handlers/vehicle/ChROSDriverInputsHandler.cpp
    )
  endif()
endif()
source_group("vehicle_handler" FILES ${CH_ROS_VEHICLE_HANDLER_FILES})

set(CH_ROS_SENSOR_HANDLER_FILES "")
if (ENABLE_MODULE_SENSOR)
  set(CH_ROS_SENSOR_HANDLER_FILES
    handlers/sensor/ChROSCameraHandler.h
    handlers/sensor/ChROSCameraHandler.cpp
    handlers/sensor/ChROSLidarHandler.h
    handlers/sensor/ChROSLidarHandler.cpp
    handlers/sensor/ChROSAccelerometerHandler.h
    handlers/sensor/ChROSAccelerometerHandler.cpp
    handlers/sensor/ChROSGyroscopeHandler.h
    handlers/sensor/ChROSGyroscopeHandler.cpp
    handlers/sensor/ChROSMagnetometerHandler.h
    handlers/sensor/ChROSMagnetometerHandler.cpp
    handlers/sensor/ChROSIMUHandler.h
    handlers/sensor/ChROSIMUHandler.cpp
    handlers/sensor/ChROSGPSHandler.h
    handlers/sensor/ChROSGPSHandler.cpp

    handlers/sensor/ChROSSensorHandlerUtilities.h
    handlers/sensor/ChROSSensorHandlerUtilities.cpp
  )
endif()
source_group("sensor_handler" FILES ${CH_ROS_SENSOR_HANDLER_FILES})

set(CH_ROS_ROBOT_HANDLER_FILES
  handlers/robot/ChROSRobotModelHandler.h
  handlers/robot/ChROSRobotModelHandler.cpp
)
if (CH_ROS_HAS_INTERFACES)
  list(APPEND CH_ROS_ROBOT_HANDLER_FILES
    handlers/robot/viper/ChROSViperDCMotorControlHandler.h
    handlers/robot/viper/ChROSViperDCMotorControlHandler.cpp
  )
endif()
source_group("robot_handler" FILES ${CH_ROS_ROBOT_HANDLER_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_ros library
#-----------------------------------------------------------------------------

add_library(ChronoEngine_ros SHARED
	${CH_ROS_BASE_FILES}
	${CH_ROS_HANDLER_FILES}
	${CH_ROS_VEHICLE_HANDLER_FILES}
	${CH_ROS_SENSOR_HANDLER_FILES}
	${CH_ROS_ROBOT_HANDLER_FILES}
)

list(APPEND CH_ROS_LIBRARIES ChronoModels_robot)

# Update compile definitions on optionally found packages
if (CH_ROS_HAS_INTERFACES)
  target_compile_definitions(ChronoEngine_ros PUBLIC -DCHRONO_ROS_HAS_INTERFACES)
endif()

if (ENABLE_MODULE_VEHICLE)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_vehicle)
  if (ENABLE_MODULE_VEHICLE_MODELS)
    list(APPEND CH_ROS_LIBRARIES ChronoModels_vehicle)
  endif()
endif()

if (ENABLE_MODULE_SENSOR)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_sensor)
endif()

if (ENABLE_MODULE_PARSERS AND HAVE_URDF)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_parsers)
  target_compile_definitions(ChronoEngine_ros PUBLIC -DCHRONO_PARSERS_URDF)
endif()

# Set some variables to be visible outside this directory
set(CH_ROS_INCLUDES   "${CH_ROS_INCLUDES}"   PARENT_SCOPE)
set(CH_ROS_LIBRARIES  "${CH_ROS_LIBRARIES}"  PARENT_SCOPE)
set(CH_ROS_HAS_INTERFACES ${CH_ROS_HAS_INTERFACES} PARENT_SCOPE)

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")
endif()

set_target_properties(ChronoEngine_ros PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")
set_target_properties(ChronoEngine_ros PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(ChronoEngine_ros PRIVATE "CH_API_COMPILE_ROS")

target_include_directories(ChronoEngine_ros PUBLIC ${CH_ROS_INCLUDES})

target_link_libraries(ChronoEngine_ros ${CH_ROS_LIBRARIES} ${CH_ROS_LINKED_LIBRARIES})

install(TARGETS ChronoEngine_ros
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

#-------------------------------------------------------------------------------
# Install Chrono::ROS headers
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_ros
        FILES_MATCHING PATTERN "*.h" PATTERN "fbs" EXCLUDE)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CH_CONFIG_INSTALL_PATH "cmake")
else()
    set(CH_CONFIG_INSTALL_PATH "lib/cmake/Chrono")
endif()

install(FILES ${CMAKE_SOURCE_DIR}/cmake/FindROS2.cmake DESTINATION ${CH_CONFIG_INSTALL_PATH} RENAME ros2-config.cmake)
