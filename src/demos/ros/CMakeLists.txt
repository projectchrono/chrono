# Demo programs for simulations with models from the Chrono::ROS library
# ------------------------------------------------------------------------------

message(STATUS "Demo programs for Chrono::ROS...")

# ------------------------------------------------------------------------------

set(DEMOS
    demo_ROS_test
)

# ------------------------------------------------------------------------------
# Include directories, compiler flags, libraries

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_ros")

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

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${CMAKE_BINARY_DIR}/src/chrono_ros/chrono_ros_interfaces/chrono_ros_interfaces/share/chrono_ros_interfaces/cmake)
target_depends_on_ros2_packages(rclcpp chrono_ros_interfaces std_msgs geometry_msgs)

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    set(CMAKE_CXX_STANDARD 17)

    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

if(ENABLE_MODULE_SENSOR)
  include_directories(${CH_SENSOR_INCLUDES})
  list(APPEND LIBS "ChronoEngine_sensor")
endif()

list(APPEND LIBS "${CH_ROS_LIBRARIES}")

#--------------------------------------------------------------
# OS-specific settings

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_include_directories(${PROGRAM} PRIVATE ${CH_ROS_INCLUDES})
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
