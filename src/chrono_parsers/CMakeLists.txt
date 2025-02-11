#=============================================================================
# CMake configuration file for the Chrono::Parsers module
#=============================================================================

option(ENABLE_MODULE_PARSERS "Enable the Chrono Parsers module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_PARSERS)
    mark_as_advanced(FORCE urdfdom_DIR)
    mark_as_advanced(FORCE urdfdom_headers_DIR)
    mark_as_advanced(FORCE console_bridge_DIR)
    mark_as_advanced(FORCE tinyxml2_DIR)
    return()
endif()

MESSAGE(STATUS "\n==== Chrono Parsers module ====\n")

mark_as_advanced(CLEAR urdfdom_DIR)
mark_as_advanced(CLEAR urdfdom_headers_DIR)
mark_as_advanced(CLEAR console_bridge_DIR)
mark_as_advanced(CLEAR tinyxml2_DIR)


# Create target

set(ChronoEngine_PARSERS_SOURCES 
    ChParserAdams.cpp
    ChParserAdamsTokenizer.yy.cpp
    ChParserOpenSim.cpp
    ChRobotActuation.cpp
)

set(ChronoEngine_PARSERS_HEADERS
    ChApiParsers.h
    ChParserAdams.h 
    ChParserOpenSim.h
    ChRobotActuation.h
)

add_library(ChronoEngine_parsers ${ChronoEngine_PARSERS_SOURCES} ${ChronoEngine_PARSERS_HEADERS})

add_library(ChronoEngine::ChronoEngine_parsers ALIAS ChronoEngine_parsers)

target_compile_definitions(ChronoEngine_parsers PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_PARSERS>)

target_link_libraries(ChronoEngine_parsers PRIVATE ChronoEngine)


# Include URDF parser if found --------------------------------------------

set(HAVE_URDF FALSE)

find_package(urdfdom QUIET)
if(urdfdom_FOUND)
    find_package(urdfdom_headers REQUIRED)
    find_package(console_bridge REQUIRED)
    find_package(tinyxml2 REQUIRED)

    if(urdfdom_headers_FOUND AND console_bridge_FOUND AND tinyxml2_FOUND)
        set(HAVE_URDF TRUE)
        message(STATUS "Found URDFDOM and dependencies")
        message(STATUS "  URDFDOM include dirs: ${urdfdom_INCLUDE_DIRS}")
    else()
        message(STATUS "A dependency of the package urdfdom NOT found\n  The Chrono URDF parser will NOT be built.")
    endif()

    target_sources(ChronoEngine_parsers PRIVATE ChParserURDF.h ChParserURDF.cpp)
    target_link_libraries(ChronoEngine_parsers PUBLIC urdfdom::urdfdom_model)
    target_link_libraries(ChronoEngine_parsers PUBLIC urdfdom::urdfdom_sensor)
    target_link_libraries(ChronoEngine_parsers PUBLIC tinyxml2::tinyxml2)
else()
    message(STATUS "Package urdfdom NOT found\n  The Chrono URDF parser will NOT be built.")
endif()

SET(HAVE_URDF "${HAVE_URDF}" PARENT_SCOPE)


# Include ROS parser if found --------------------------------------------

if(ENABLE_MODULE_ROS)

  set(HAVE_ROS FALSE)

  find_package(ROS2 QUIET
    COMPONENTS ament_index_cpp
  )

  if (ROS2_FOUND)
    find_package(ament_index_cpp QUIET)
    if (ament_index_cpp_FOUND)
    set(HAVE_ROS TRUE)
    message(STATUS "Found ROS 2")
    message(STATUS "  ROS 2 distro is \"$ENV{ROS_DISTRO}\"")  
    set(_ament_prefix_path "$ENV{AMENT_PREFIX_PATH}")
    foreach(_ros2_packages_path IN LISTS _ament_prefix_path)
           string(REPLACE ":" ";" _ros2_packages_path ${_ros2_packages_path})
           target_include_directories(ChronoEngine_parsers PRIVATE "${_ros2_packages_path}/include")
    endforeach()
    target_compile_definitions(ChronoEngine_parsers PUBLIC $<$<COMPILE_LANGUAGE:CXX>:HAVE_ROS>)
    target_link_libraries(ChronoEngine_parsers PRIVATE ament_index_cpp::ament_index_cpp)
    endif()
  else()
    set(HAVE_ROS FALSE)
    message(STATUS "Package ROS2 NOT found\n  ROS support disabled.")
  endif()

  set(HAVE_ROS "${HAVE_ROS}" PARENT_SCOPE)

endif()

# Include Python parser if found -----------------------------------------------

set(HAVE_PYTHON FALSE)

set(Python_ADDITIONAL_VERSIONS 3.4) # TODO: DARIOM: is still in use with FindPython3?
find_package(Python3 QUIET COMPONENTS Interpreter Development)

if(Python3_Interpreter_FOUND AND Python3_Development_FOUND)
    set(HAVE_PYTHON TRUE)
    message(STATUS "Found Python and dependencies")
    message(STATUS "  Python3_Interpreter_FOUND: ${Python3_Interpreter_FOUND}")
    message(STATUS "  Python3_Development_FOUND: ${Python3_Development_FOUND}")

    target_sources(ChronoEngine_parsers PRIVATE ChParserPython.h ChParserPython.cpp)
    target_link_libraries(ChronoEngine_parsers PRIVATE Python3::Python)
    # target_link_libraries(ChronoEngine_parsers PRIVATE Python3::Module)

else()
    message(STATUS "Package Python NOT found\n  The Chrono Python parser will NOT be built.")
endif()

SET(HAVE_PYTHON "${HAVE_PYTHON}" PARENT_SCOPE)

install(TARGETS ChronoEngine_parsers
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_parsers)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${ChronoEngine_PARSERS_HEADERS}
        DESTINATION include/chrono_parsers)
