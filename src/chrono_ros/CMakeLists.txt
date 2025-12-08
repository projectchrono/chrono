#=============================================================================
# CMake configuration file for Chrono::ROS module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_ROS "Enable the Chrono::ROS module" OFF)

if(NOT CH_ENABLE_MODULE_ROS)
  return()
endif()

message(STATUS "\n==== Chrono::ROS module ====\n")

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# Find ROS packages
#-----------------------------------------------------------------------------

# RADU TODO: do we need this here? At least, we should get rid of the ament check
#            since we do not want to use the ament macros at all.

if(NOT DEFINED ENV{ROS_DISTRO} OR NOT DEFINED ENV{AMENT_PREFIX_PATH})   
  message("ERROR Chrono::ROS requires ROS 2, but ROS 2 was not found; disabling Chrono::ROS")
  set(CH_ENABLE_MODULE_ROS OFF CACHE BOOL "Enable the Chrono::ROS module" FORCE)
  return()
endif()

# Find required ROS2 packages
find_package(rclcpp QUIET)
find_package(tf2_ros QUIET)
find_package(rcl_interfaces QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(tf2_msgs QUIET)

if(rclcpp_FOUND AND tf2_ros_FOUND AND rcl_interfaces_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND AND sensor_msgs_FOUND AND tf2_msgs_FOUND)
  message(STATUS "Found ROS 2")
else()
  message("ERROR Chrono::ROS requires ROS 2, but ROS 2 was not found; disabling Chrono::ROS")
  set(CH_ENABLE_MODULE_ROS OFF CACHE BOOL "Enable the Chrono::ROS module" FORCE)
  return()
endif()

# Set list of ROS2 targets
set(ROS2_TARGETS 
    rclcpp::rclcpp
    tf2_ros::tf2_ros
    ${rcl_interfaces_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
)

# Optional package
find_package(chrono_ros_interfaces QUIET)

if(chrono_ros_interfaces_FOUND)
  set(CH_ROS_HAS_INTERFACES TRUE)
  set(ROS2_TARGETS ${ROS2_TARGETS} ${chrono_ros_interfaces_TARGETS})
else()
  set(CH_ROS_HAS_INTERFACES FALSE)
  message(STATUS "Could not find chrono_ros_interfaces; some features will be disabled")
endif()

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

if(WIN32)
    # On Windows, executables and DLLs must be in the same directory (bin) for PATH resolution
    set(CH_ROS_NODE_DESTINATION bin)
else()
    # On Linux, we install the node executable to lib because:
    # 1. It is an internal helper process, not a user-facing command (keeps bin/ clean)
    # 2. It simplifies finding the executable relative to the shared library
    # 3. It simplifies RPATH resolution (it sits next to the libraries it links against)
    set(CH_ROS_NODE_DESTINATION lib)
endif()

# Define paths for the configuration file
set(CH_ROS_NODE_BUILD_PATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/chrono_ros_node")
set(CH_ROS_NODE_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${CH_ROS_NODE_DESTINATION}/chrono_ros_node")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigROS.h.in ${PROJECT_BINARY_DIR}/chrono_ros/ChConfigROS.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_ros/ChConfigROS.h" DESTINATION include/chrono_ros)

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

set(CH_ROS_IPC_FILES
  ipc/ChROSIPCMessage.h
  ipc/ChROSSharedMemory.h
  ipc/ChROSSharedMemory.cpp
  ipc/ChROSRingBuffer.h
  ipc/ChROSRingBuffer.cpp
  ipc/ChROSIPCChannel.h
  ipc/ChROSIPCChannel.cpp
  ipc/ChROSSubprocessManager.h
  ipc/ChROSSubprocessManager.cpp
  
  ChROSIPCInterface.h
  ChROSIPCInterface.cpp
)
source_group("ipc" FILES ${CH_ROS_IPC_FILES})

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
if (CH_ENABLE_MODULE_VEHICLE)
  if (CH_ROS_HAS_INTERFACES)
    list(APPEND CH_ROS_VEHICLE_HANDLER_FILES 
      handlers/vehicle/ChROSDriverInputsHandler.h
      handlers/vehicle/ChROSDriverInputsHandler.cpp
    )
  endif()
endif()
source_group("vehicle_handler" FILES ${CH_ROS_VEHICLE_HANDLER_FILES})

set(CH_ROS_SENSOR_HANDLER_FILES "")
if (CH_ENABLE_MODULE_SENSOR)
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

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_ROS "")
if(CH_ENABLE_MODULE_VEHICLE)
  list (APPEND DEPENDENCIES_ROS "VEHICLE")
endif()
if(CH_ENABLE_MODULE_SENSOR)
  list (APPEND DEPENDENCIES_ROS "SENSOR")
endif()
if(CH_ENABLE_MODULE_PARSERS  AND HAVE_URDF)
  list (APPEND DEPENDENCIES_ROS "PARSERS")
endif()

set(DEPENDENCIES_ROS ${DEPENDENCIES_ROS} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Create the Chrono_ros library
#-----------------------------------------------------------------------------

add_library(Chrono_ros SHARED
            ${CH_ROS_BASE_FILES}
            ${CH_ROS_IPC_FILES}
            ${CH_ROS_HANDLER_FILES}
            ${CH_ROS_VEHICLE_HANDLER_FILES}
            ${CH_ROS_SENSOR_HANDLER_FILES}
            ${CH_ROS_ROBOT_HANDLER_FILES}
)
add_library(Chrono::ros ALIAS Chrono_ros)

set_target_properties(Chrono_ros PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_ros PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_ros PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_ros PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

set_target_properties(Chrono_ros PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(Chrono_ros PRIVATE "CH_API_COMPILE_ROS")

# Update compile definitions on optionally found packages
if (CH_ROS_HAS_INTERFACES)
  target_compile_definitions(Chrono_ros PUBLIC CHRONO_ROS_HAS_INTERFACES)
endif()

# Windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_options(Chrono_ros PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4661>) 
    target_compile_options(Chrono_ros PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4005>) 
endif()

# Link libraries
target_link_libraries(Chrono_ros PUBLIC Chrono_core)
target_link_libraries(Chrono_ros PUBLIC ChronoModels_robot)

if (CH_ENABLE_MODULE_VEHICLE)
  target_link_libraries(Chrono_ros PUBLIC Chrono_vehicle)
  if (CH_ENABLE_MODULE_VEHICLE_MODELS)
    target_link_libraries(Chrono_ros PUBLIC ChronoModels_vehicle)
  endif()
endif()

if (CH_ENABLE_MODULE_SENSOR)
  target_link_libraries(Chrono_ros PUBLIC Chrono_sensor)
endif()

if (CH_ENABLE_MODULE_PARSERS AND HAVE_URDF)
  target_compile_definitions(Chrono_ros PUBLIC CHRONO_PARSERS_URDF)
  target_link_libraries(Chrono_ros PUBLIC Chrono_parsers)
endif()

target_link_libraries(Chrono_ros PUBLIC ${ROS2_TARGETS})

# Set some variables to be visible outside this directory
set(CH_ROS_HAS_INTERFACES ${CH_ROS_HAS_INTERFACES} PARENT_SCOPE)

install(TARGETS Chrono_ros
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_ros
)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_ros
        FILES_MATCHING PATTERN "*.h" PATTERN "fbs" EXCLUDE)

#-----------------------------------------------------------------------------
# Create the ROS node subprocess executable
#-----------------------------------------------------------------------------

add_executable(chrono_ros_node chrono_ros_node.cpp)

target_link_libraries(chrono_ros_node 
    ${ROS2_TARGETS}
    Chrono_core
)

# Add the IPC source files directly to avoid circular dependencies
target_sources(chrono_ros_node PRIVATE
    ipc/ChROSSharedMemory.cpp
    ipc/ChROSRingBuffer.cpp  
    ipc/ChROSIPCChannel.cpp
    
    # Add handler utilities
    handlers/ChROSHandlerUtilities.cpp
    
    # Add handler ROS publishing implementations (only linked in subprocess)
    handlers/ChROSClockHandler_ros.cpp
    handlers/ChROSBodyHandler_ros.cpp
    handlers/ChROSTFHandler_ros.cpp
    handlers/vehicle/ChROSDriverInputsHandler_ros.cpp
    handlers/robot/viper/ChROSViperDCMotorControlHandler_ros.cpp
    handlers/sensor/ChROSCameraHandler_ros.cpp
    handlers/sensor/ChROSLidarHandler_ros.cpp
    handlers/sensor/ChROSAccelerometerHandler_ros.cpp
    handlers/sensor/ChROSGyroscopeHandler_ros.cpp
    handlers/sensor/ChROSMagnetometerHandler_ros.cpp
    handlers/sensor/ChROSIMUHandler_ros.cpp
    handlers/sensor/ChROSGPSHandler_ros.cpp
    handlers/robot/ChROSRobotModelHandler_ros.cpp
)

target_include_directories(chrono_ros_node PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${PROJECT_BINARY_DIR}
)

install(TARGETS chrono_ros_node
        EXPORT ChronoTargets
        RUNTIME DESTINATION ${CH_ROS_NODE_DESTINATION}
)