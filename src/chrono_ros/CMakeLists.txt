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

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# Find ROS
#-----------------------------------------------------------------------------

find_package(ROS2 REQUIRED)
if (NOT ROS2_FOUND)
	message("Chrono::ROS depends on ROS 2, which can't be found. You may need to source the installation.")
  return()
endif()

# Update the includes directories to point to ROS includes
set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")
foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
    string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
    list(APPEND CH_ROS_INCLUDES "${_ros2_packages_path}/include")
endforeach()
set(_ros_packages rclcpp std_msgs geometry_msgs sensor_msgs chrono_ros_interfaces)
foreach(_package IN LISTS _ros_packages)
  find_package(${_package} REQUIRED)
  list(APPEND CH_ROS_LIBRARIES "${${_package}_LIBRARIES}")
  list(APPEND CH_ROS_INCLUDES "${${_package}_INCLUDE_DIRS}")
endforeach()

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

  handlers/ChROSHandlerUtilities.h
  handlers/ChROSHandlerUtilities.cpp
)
source_group("handler" FILES ${CH_ROS_HANDLER_FILES})

if (ENABLE_MODULE_VEHICLE)
  set(CH_ROS_VEHICLE_HANDLER_FILES
    handlers/vehicle/ChROSDriverInputsHandler.h
    handlers/vehicle/ChROSDriverInputsHandler.cpp
  )
else()
  set(CH_ROS_VEHICLE_HANDLER_FILES "")
endif()
source_group("vehicle_handler" FILES ${CH_ROS_VEHICLE_HANDLER_FILES})

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
    handlers/sensor/ChROSGPSHandler.h
    handlers/sensor/ChROSGPSHandler.cpp

    handlers/sensor/ChROSSensorHandlerUtilities.h
    handlers/sensor/ChROSSensorHandlerUtilities.cpp
  )
else()
  set(CH_ROS_SENSOR_HANDLER_FILES "")
endif()
source_group("sensor_handler" FILES ${CH_ROS_SENSOR_HANDLER_FILES})

if (ENABLE_MODULE_ROBOT_MODELS)
  set(CH_ROS_ROBOT_HANDLER_FILES
    # viper
    handlers/robot/viper/ViperHandler.h
    handlers/robot/viper/ViperHandler.cpp
    handlers/robot/viper/ViperDCMotorControlHandler.h
    handlers/robot/viper/ViperDCMotorControlHandler.cpp
  )
else()
  set(CH_ROS_ROBOT_HANDLER_FILES "")
endif()
source_group("robot_handler" FILES ${CH_ROS_ROBOT_HANDLER_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_ros library
#-----------------------------------------------------------------------------

if (ENABLE_MODULE_VEHICLE)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_vehicle)
  if (ENABLE_MODULE_VEHICLE_MODELS)
    list(APPEND CH_ROS_LIBRARIES ChronoModels_vehicle)
  endif()
endif()

if (ENABLE_MODULE_ROBOT_MODELS)
  list(APPEND CH_ROS_LIBRARIES ChronoModels_robot)
endif()

if (ENABLE_MODULE_SENSOR)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_sensor)
endif()
  
if (ENABLE_MODULE_IRRLICHT)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_irrlicht)
  if (ENABLE_MODULE_VEHICLE)
    list(APPEND CH_ROS_LIBRARIES ChronoEngine_vehicle_irrlicht)
  endif()

  include_directories(${CH_IRRLICHT_INCLUDES})
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
endif()

# Set some variables to be visible outside this directory
set(CH_ROS_INCLUDES   "${CH_ROS_INCLUDES}"   PARENT_SCOPE)
set(CH_ROS_LIBRARIES  "${CH_ROS_LIBRARIES}"  PARENT_SCOPE)

add_library(ChronoEngine_ros SHARED
	${CH_ROS_BASE_FILES}
	${CH_ROS_HANDLER_FILES}
	${CH_ROS_VEHICLE_HANDLER_FILES}
	${CH_ROS_SENSOR_HANDLER_FILES}
	${CH_ROS_ROBOT_HANDLER_FILES}
)

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

target_link_libraries(ChronoEngine_ros ${CH_ROS_LIBRARIES})

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
