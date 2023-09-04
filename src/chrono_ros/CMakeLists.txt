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

#-----------------------------------------------------------------------------
# Find ROS
#-----------------------------------------------------------------------------

find_package(ROS2 REQUIRED)
if (NOT ROS2_FOUND)
	message("Chrono::ROS depends on ROS 2, which can't be found. You may need to source the installation.")
  return()
endif()

#-----------------------------------------------------------------------------
# Build the chrono_ros_interfaces
#-----------------------------------------------------------------------------
if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/chrono_ros_interfaces/.git)
  message("The chrono_ros_interfaces submodule can't be found. Ensure you have initialized it using `git submodule update --init`.")
  return()
endif()

# Helper function to link to ros dependencies
function(target_depends_on_ros2_packages)
    find_package(ROS2 MODULE)
    if (NOT ROS2_FOUND)
        return()
    endif()
    message(DEBUG "Building ROS2 Gem with ros2 $ENV{ROS_DISTRO}")
    #TODO - compare to previous env since we need to rerun cmake if we source a different ros2 env!
    #TODO - can be done with a file that is in CONFIGURE_DEPENDS so that a change triggers build
    set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")

    # ros2 directories with libraries, e.g. /opt/ros/galactic/lib, locally built custom packages etc.
    set(_ros2_library_directories)
    set(_ros2_include_directories)
    set(_ros2_package_libraries)
    foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
        string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
        list(APPEND _ros2_library_directories "${_ros2_packages_path}/lib")
        list(APPEND _ros2_include_directories "${_ros2_packages_path}/include")
    endforeach()
    foreach(_package IN LISTS ARGN)
        message(DEBUG "Processing package: ${_package}")
        find_package(${_package} REQUIRED)
        list(APPEND _ros2_package_libraries "${${_package}_LIBRARIES}")
        list(APPEND _ros2_include_directories "${${_package}_INCLUDE_DIRS}")
    endforeach()
    set(CH_ROS_INCLUDES ${CH_ROS_INCLUDES} ${_ros2_include_directories} PARENT_SCOPE)
    set(CH_ROS_LIBRARIES ${CH_ROS_LIBRARIES} ${_ros2_package_libraries} PARENT_SCOPE)
endfunction()

# TODO: Is there a better way to do this?
execute_process(
    COMMAND colcon build --build-base ${CMAKE_BINARY_DIR}/src/chrono_ros/chrono_ros_interfaces --install-base ${CMAKE_BINARY_DIR}/src/chrono_ros/chrono_ros_interfaces/
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/chrono_ros_interfaces
)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_BINARY_DIR}/src/chrono_ros/chrono_ros_interfaces/chrono_ros_interfaces/share/chrono_ros_interfaces/cmake)

# Find our required dependencies
target_depends_on_ros2_packages(rclcpp chrono_ros_interfaces)

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
    handlers/ChROSCameraHandler.h
    handlers/ChROSCameraHandler.cpp
    handlers/ChROSLidarHandler.h
    handlers/ChROSLidarHandler.cpp
    handlers/ChROSAccelerometerHandler.h
    handlers/ChROSAccelerometerHandler.cpp
    handlers/ChROSGyroscopeHandler.h
    handlers/ChROSGyroscopeHandler.cpp
    handlers/ChROSMagnetometerHandler.h
    handlers/ChROSMagnetometerHandler.cpp
    handlers/ChROSGPSHandler.h
    handlers/ChROSGPSHandler.cpp
  )
else()
  set(CH_ROS_SENSOR_HANDLER_FILES "")
endif()
source_group("sensor_handler" FILES ${CH_ROS_SENSOR_HANDLER_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_ros library
#-----------------------------------------------------------------------------

if (ENABLE_MODULE_VEHICLE)
	list(APPEND CH_ROS_LIBRARIES ChronoEngine_vehicle)
  if (ENABLE_MODULE_VEHICLE_MODELS)
    list(APPEND CH_ROS_LIBRARIES ChronoModels_vehicle)
  endif()
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
