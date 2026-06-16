#=============================================================================
# CMake configuration file for the Chrono::ROS module
#
# Cannot be used stand-alone (it is loaded by the parent CMake configuration)
#
# Schema-driven architecture (see CLAUDE.md in this directory):
#   chrono_ros_core   internal STATIC lib - protocol, CDR codec, schema,
#                     message codec, shm transport. Depends on NEITHER
#                     Chrono NOR ROS.
#   Chrono_ros        the simulation-side SHARED lib - public API (manager,
#                     bridge, handlers). Links Chrono + chrono_ros_core.
#                     Contains ZERO ROS symbols (invariant I1).
#   chrono_ros_node   the bridge subprocess - generic rclcpp node. Links
#                     ROS + chrono_ros_core. Contains ZERO Chrono symbols
#                     (invariant I8).
#=============================================================================

option(CH_ENABLE_MODULE_ROS "Enable the Chrono::ROS module" OFF)

if(NOT CH_ENABLE_MODULE_ROS)
  return()
endif()

message(STATUS "\n==== Chrono::ROS module ====\n")

# C++17: required by rclcpp and used throughout the module
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# Find ROS 2 (required to build chrono_ros_node; the Chrono_ros library
# itself has no ROS dependency)
#-----------------------------------------------------------------------------

if(NOT DEFINED ENV{ROS_DISTRO} OR NOT DEFINED ENV{AMENT_PREFIX_PATH})
  message("ERROR Chrono::ROS requires a sourced ROS 2 environment to build chrono_ros_node; disabling Chrono::ROS")
  set(CH_ENABLE_MODULE_ROS OFF CACHE BOOL "Enable the Chrono::ROS module" FORCE)
  return()
endif()

find_package(rclcpp QUIET)
find_package(rcpputils QUIET)
find_package(rosidl_typesupport_introspection_cpp QUIET)
find_package(rosidl_runtime_cpp QUIET)

if(rclcpp_FOUND AND rcpputils_FOUND AND rosidl_typesupport_introspection_cpp_FOUND AND rosidl_runtime_cpp_FOUND)
  message(STATUS "Found ROS 2 ($ENV{ROS_DISTRO})")
else()
  message("ERROR Chrono::ROS requires ROS 2 (rclcpp, rcpputils, rosidl introspection), but it was not found; disabling Chrono::ROS")
  set(CH_ENABLE_MODULE_ROS OFF CACHE BOOL "Enable the Chrono::ROS module" FORCE)
  return()
endif()

# Message-type packages (std_msgs, sensor_msgs, custom packages, ...) are NOT
# build dependencies: the node loads their typesupport at runtime by name.

#-----------------------------------------------------------------------------
# Generated configuration (path of the node executable)
#-----------------------------------------------------------------------------

if(WIN32)
  # Executable next to the DLLs for PATH resolution
  set(CH_ROS_NODE_DESTINATION bin)
else()
  # Internal helper process: keep bin/ clean, simplify RPATH (next to libs)
  set(CH_ROS_NODE_DESTINATION lib)
endif()

set(CH_ROS_NODE_BUILD_PATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/chrono_ros_node")
set(CH_ROS_NODE_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/${CH_ROS_NODE_DESTINATION}/chrono_ros_node")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigROS.h.in ${PROJECT_BINARY_DIR}/chrono_ros/ChConfigROS.h)
install(FILES "${PROJECT_BINARY_DIR}/chrono_ros/ChConfigROS.h" DESTINATION include/chrono_ros)

#-----------------------------------------------------------------------------
# chrono_ros_core: internal static library (no Chrono, no ROS)
#-----------------------------------------------------------------------------

set(CH_ROS_CORE_FILES
  core/ChROSCdr.h
  core/ChROSCdr.cpp
  core/ChROSSchema.h
  core/ChROSSchema.cpp
  core/ChROSMessageCodec.h
  core/ChROSMessageCodec.cpp
  core/ChROSFrame.h
  core/ChROSControl.h
  core/ChROSControl.cpp
  core/ChROSQoSSpec.h

  core/transport/ChROSRingBuffer.h
  core/transport/ChROSRingBuffer.cpp
  core/transport/ChROSSharedMemory.h
  core/transport/ChROSSharedMemory.cpp
  core/transport/ChROSChannel.h
  core/transport/ChROSChannel.cpp
)
source_group("core" FILES ${CH_ROS_CORE_FILES})

add_library(chrono_ros_core STATIC ${CH_ROS_CORE_FILES})
set_target_properties(chrono_ros_core PROPERTIES POSITION_INDEPENDENT_CODE ON LINKER_LANGUAGE CXX)
target_include_directories(chrono_ros_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
if(MSVC)
  set_target_properties(chrono_ros_core PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()
# shm_open lives in librt on older glibc
if(UNIX AND NOT APPLE)
  target_link_libraries(chrono_ros_core PUBLIC rt)
endif()
find_package(Threads REQUIRED)
target_link_libraries(chrono_ros_core PUBLIC Threads::Threads)

#-----------------------------------------------------------------------------
# Chrono_ros: the simulation-side shared library (NO ROS dependency)
#-----------------------------------------------------------------------------

set(CH_ROS_API_FILES
  ChApiROS.h

  ChROSManager.h
  ChROSManager.cpp
  ChROSBridge.h
  ChROSBridge.cpp
  ChROSHandler.h
  ChROSHandler.cpp
  ChROSMessage.h
  ChROSMessage.cpp
  ChROSPublisher.h
  ChROSSubscription.h
  ChROSQoS.h
  ChROSNodeProcess.h
  ChROSNodeProcess.cpp
)
source_group("api" FILES ${CH_ROS_API_FILES})

# Built-in handlers (exemplary uses of the public API). Phase 5 ports these from
# Chrono 9.0 with matching call interfaces; sensor/vehicle/robot handlers and
# the URDF/sensor TF overloads return in later Phase-5 batches.
set(CH_ROS_HANDLER_FILES
  handlers/ChROSClockHandler.h
  handlers/ChROSClockHandler.cpp
  handlers/ChROSBodyHandler.h
  handlers/ChROSBodyHandler.cpp
  handlers/ChROSTFHandler.h
  handlers/ChROSTFHandler.cpp
)
source_group("handlers" FILES ${CH_ROS_HANDLER_FILES})
list(APPEND CH_ROS_API_FILES ${CH_ROS_HANDLER_FILES})

# Sensor handlers: compiled only when the Chrono::Sensor module is enabled (they
# accept a ChSensor and read its host buffers). Camera/lidar return in a later
# Phase-5 batch.
set(CH_ROS_SENSOR_HANDLER_FILES "")
if(CH_ENABLE_MODULE_SENSOR)
  set(CH_ROS_SENSOR_HANDLER_FILES
    handlers/sensor/ChROSSensorHandlerUtilities.h
    handlers/sensor/ChROSAccelerometerHandler.h
    handlers/sensor/ChROSAccelerometerHandler.cpp
    handlers/sensor/ChROSGyroscopeHandler.h
    handlers/sensor/ChROSGyroscopeHandler.cpp
    handlers/sensor/ChROSMagnetometerHandler.h
    handlers/sensor/ChROSMagnetometerHandler.cpp
    handlers/sensor/ChROSGPSHandler.h
    handlers/sensor/ChROSGPSHandler.cpp
    handlers/sensor/ChROSIMUHandler.h
    handlers/sensor/ChROSIMUHandler.cpp
  )
  source_group("handlers\\sensor" FILES ${CH_ROS_SENSOR_HANDLER_FILES})
  list(APPEND CH_ROS_API_FILES ${CH_ROS_SENSOR_HANDLER_FILES})
endif()

add_library(Chrono_ros SHARED ${CH_ROS_API_FILES})
add_library(Chrono::ros ALIAS Chrono_ros)

set_target_properties(Chrono_ros PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX} LINKER_LANGUAGE CXX)
if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_ros PROPERTIES COMPILE_FLAGS "/GL" LINK_FLAGS "/LTCG")
endif()
if(CH_STATIC)
  set_target_properties(Chrono_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
if(MSVC)
  set_target_properties(Chrono_ros PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_ros PRIVATE "CH_API_COMPILE_ROS")

target_link_libraries(Chrono_ros PUBLIC Chrono_core)
target_link_libraries(Chrono_ros PRIVATE chrono_ros_core)
if(UNIX AND NOT APPLE)
  target_link_libraries(Chrono_ros PRIVATE dl)  # dladdr in ChROSNodeProcess
endif()

# Sensor handlers link Chrono_sensor. PUBLIC: their public headers include
# chrono_sensor headers, so consumers (demos, the PyChrono module) need its
# include path. Chrono_sensor is sim-side and ROS-free, so this does not affect
# the isolation invariant (I1).
if(CH_ENABLE_MODULE_SENSOR)
  target_link_libraries(Chrono_ros PUBLIC Chrono_sensor)
endif()

# Module dependency bookkeeping for the parent build (module short-names).
set(DEPENDENCIES_ROS "")
if(CH_ENABLE_MODULE_SENSOR)
  list(APPEND DEPENDENCIES_ROS "SENSOR")
endif()
set(DEPENDENCIES_ROS ${DEPENDENCIES_ROS} PARENT_SCOPE)
set(CH_ROS_HAS_INTERFACES FALSE PARENT_SCOPE)

install(TARGETS Chrono_ros
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_ros)

# Public headers (the core headers are included by public API headers and are
# therefore installed as well; they remain dependency-free).
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_ros
        FILES_MATCHING PATTERN "*.h"
        PATTERN "node" EXCLUDE
        PATTERN "tests_dev" EXCLUDE)

#-----------------------------------------------------------------------------
# chrono_ros_node: the bridge subprocess (NO Chrono dependency)
#-----------------------------------------------------------------------------

add_executable(chrono_ros_node
  node/chrono_ros_node.cpp
  node/ChROSNodeTypeSupport.h
  node/ChROSNodeTypeSupport.cpp
)

target_include_directories(chrono_ros_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(chrono_ros_node
  chrono_ros_core
  rclcpp::rclcpp
  rcpputils::rcpputils
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp::rosidl_runtime_cpp
)
if(MSVC)
  set_target_properties(chrono_ros_node PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

install(TARGETS chrono_ros_node
        EXPORT ChronoTargets
        RUNTIME DESTINATION ${CH_ROS_NODE_DESTINATION})
