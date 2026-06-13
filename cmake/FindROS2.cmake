# Note that this does not find any ros2 package in particular, but determines whether a distro is sourced properly
# Can be extended to handle supported / unsupported distros

unset(ROS2_FOUND)

get_cmake_property(_initial_cache CACHE_VARIABLES)

if (NOT DEFINED ENV{ROS_DISTRO} OR NOT DEFINED ENV{AMENT_PREFIX_PATH})   
    set(ROS2_FOUND FALSE)
    return()
endif()

# Update the includes directories to point to ROS includes
set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")
foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
    string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
    list(APPEND ROS2_INCLUDE_DIRS "${_ros2_packages_path}/include")
endforeach()

# Find the ROS2 components/packages
foreach (COMPONENT IN LISTS ROS2_FIND_COMPONENTS)
    if(ROS2_FIND_REQUIRED_${COMPONENT})
        set(COMPONENT_REQ_TYPE "REQUIRED")
    else()
        set(COMPONENT_REQ_TYPE "OPTIONAL")
    endif()

    message(STATUS "   Requested component ${COMPONENT} (${COMPONENT_REQ_TYPE})")
    find_package(${COMPONENT} QUIET)

    if (${COMPONENT}_FOUND)
        list(APPEND ROS2_INCLUDE_DIRS "${${COMPONENT}_INCLUDE_DIRS}")
        list(APPEND ROS2_LIBRARIES "${${COMPONENT}_LIBRARIES}")
    else()
        if(ROS2_FIND_REQUIRED_${COMPONENT})
            message(FATAL_ERROR "Could not find required ROS package: ${COMPONENT}")
        else()
            message(STATUS "      Optional ROS package not found: ${COMPONENT}")
        endif()
    endif()
endforeach()

list(REMOVE_DUPLICATES ROS2_INCLUDE_DIRS)
list(REMOVE_DUPLICATES ROS2_LIBRARIES)

set(ROS2_FOUND TRUE)

# Get the final cache variables and identify new ones
# Only mark new variables as advanced, if necessary
get_cmake_property(_final_cache CACHE_VARIABLES)
if (NOT "${_final_cache}" STREQUAL "${_initial_cache}")
    list(REMOVE_ITEM _final_cache ${_initial_cache})
    message(STATUS "Marking new variables as advanced: ${_final_cache}")
    mark_as_advanced(FORCE ${_final_cache})
endif()