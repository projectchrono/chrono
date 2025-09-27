#=============================================================================
# CMake configuration file for the Chrono::Parsers module
#=============================================================================

option(CH_ENABLE_MODULE_PARSERS "Enable the Chrono Parsers module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_PARSERS)
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

set(Chrono_PARSERS_FILES 
    ChApiParsers.h
    ChParserAdams.h 
    ChParserAdams.cpp
    ChParserAdamsTokenizer.yy.cpp
    ChParserOpenSim.h
    ChParserOpenSim.cpp
    ChRobotActuation.h
    ChRobotActuation.cpp
)
source_group("" FILES ${Chrono_PARSERS_FILES})

set(Chrono_PARSERS_YAML_FILES
    yaml/ChParserYAML.h
    yaml/ChParserYAML.cpp
    yaml/ChParserMbsYAML.h
    yaml/ChParserMbsYAML.cpp
)
if(CH_ENABLE_MODULE_FSI)
    set(Chrono_PARSERS_YAML_FILES ${Chrono_PARSERS_YAML_FILES}
        yaml/ChParserFsiYAML.h
        yaml/ChParserFsiYAML.cpp
        yaml/ChParserCfdYAML.h
        yaml/ChParserCfdYAML.cpp
        yaml/ChParserSphYAML.h
        yaml/ChParserSphYAML.cpp
    )
endif()
if(CH_ENABLE_MODULE_VEHICLE)
    set(Chrono_PARSERS_YAML_FILES ${Chrono_PARSERS_YAML_FILES}
        yaml/ChParserVehicleYAML.h
        yaml/ChParserVehicleYAML.cpp
    )
endif()
source_group("yaml" FILES ${Chrono_PARSERS_YAML_FILES})

# ------------------------------------------------------------------------------
# Fetch yaml-cpp from third party directory
# ------------------------------------------------------------------------------

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/yaml-cpp"
)
FetchContent_MakeAvailable(yaml-cpp)

mark_as_advanced(FORCE YAML_CPP_INSTALL)
mark_as_advanced(FORCE YAML_CPP_INSTALL_CMAKEDIR)

# Disable warnings
if (MSVC)
  target_compile_options(yaml-cpp PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4251>)
  target_compile_options(yaml-cpp PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4275>)
  target_compile_options(yaml-cpp PUBLIC $<$<COMPILE_LANGUAGE:CXX>:/wd4267>)
endif()

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_PARSERS "")

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list (APPEND DEPENDENCIES_PARSERS "PARDISOMKL")
endif()
if(CH_ENABLE_MODULE_MUMPS)
  list (APPEND DEPENDENCIES_PARSERS "MUMPS")
endif()
if(CH_ENABLE_MODULE_ROS)
  list (APPEND DEPENDENCIES_PARSERS "ROS")
endif()
if(CH_ENABLE_MODULE_FSI)
  list (APPEND DEPENDENCIES_PARSERS "FSISPH")
endif()
if(CH_ENABLE_MODULE_VEHICLE)
  list (APPEND DEPENDENCIES_PARSER "VEHICLE")
endif()

set(DEPENDENCIES_PARSERS ${DEPENDENCIES_PARSERS} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Create the Chrono_parsers library
#-----------------------------------------------------------------------------

add_library(Chrono_parsers ${Chrono_PARSERS_FILES} ${Chrono_PARSERS_YAML_FILES})
add_library(Chrono::parsers ALIAS Chrono_parsers)

set_target_properties(Chrono_parsers PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_parsers PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_parsers PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_parsers PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_parsers PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_parsers PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_PARSERS>)

target_link_libraries(Chrono_parsers PRIVATE Chrono_core)
target_link_libraries(Chrono_parsers PUBLIC yaml-cpp::yaml-cpp)
if(CH_ENABLE_MODULE_PARDISO_MKL)
  target_link_libraries(Chrono_parsers PRIVATE Chrono_pardisomkl)
endif()
if(CH_ENABLE_MODULE_MUMPS)
  target_link_libraries(Chrono_parsers PRIVATE Chrono_mumps)
endif()

if(CH_ENABLE_MODULE_FSI)
  target_link_libraries(Chrono_parsers PRIVATE Chrono_fsisph)
  if(CH_ENABLE_MODULE_VSG)
    target_link_libraries(Chrono_parsers PRIVATE Chrono_fsisph_vsg)
  endif()
endif()

if(CH_ENABLE_MODULE_VEHICLE)
  target_link_libraries(Chrono_parsers PRIVATE Chrono_vehicle)
endif()

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

    target_sources(Chrono_parsers PRIVATE ChParserURDF.h ChParserURDF.cpp)
    target_link_libraries(Chrono_parsers PUBLIC urdfdom::urdfdom_model)
    target_link_libraries(Chrono_parsers PUBLIC urdfdom::urdfdom_sensor)
    target_link_libraries(Chrono_parsers PUBLIC tinyxml2::tinyxml2)

    source_group("" FILES ChParserURDF.h ChParserURDF.cpp)
else()
    message(STATUS "Package urdfdom NOT found.\n  The Chrono URDF parser will NOT be built.")
endif()

SET(HAVE_URDF "${HAVE_URDF}" PARENT_SCOPE)


# Include ROS support if found --------------------------------------------

if(CH_ENABLE_MODULE_ROS)

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
           target_include_directories(Chrono_parsers PRIVATE "${_ros2_packages_path}/include")
    endforeach()
    target_compile_definitions(Chrono_parsers PUBLIC $<$<COMPILE_LANGUAGE:CXX>:HAVE_ROS>)
    target_link_libraries(Chrono_parsers PRIVATE ament_index_cpp::ament_index_cpp)
    endif()
  else()
    set(HAVE_ROS FALSE)
    message(STATUS "Package ROS2 NOT found.\n  ROS support disabled.")
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

    target_sources(Chrono_parsers PRIVATE ChParserPython.h ChParserPython.cpp)
    target_link_libraries(Chrono_parsers PRIVATE Python3::Python)
    # target_link_libraries(Chrono_parsers PRIVATE Python3::Module)

    source_group("" FILES ChParserPython.h ChParserPython.cpp)
else()
    message(STATUS "Package Python NOT found.\n  The Chrono Python parser will NOT be built.")
endif()

SET(HAVE_PYTHON "${HAVE_PYTHON}" PARENT_SCOPE)

install(TARGETS yaml-cpp Chrono_parsers
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_parsers)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_parsers
        FILES_MATCHING PATTERN "*.h")

# On Windows, extract DLLs from targets and install (always look for Release DLLs)
#if(HAVE_URDF AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
#    get_target_property(URDFDOM_MODEL_DLL urdfdom::urdfdom_model IMPORTED_LOCATION_RELEASE)
#    get_target_property(URDFDOM_SENSOR_DLL urdfdom::urdfdom_sensor IMPORTED_LOCATION_RELEASE)
#    if(EXISTS "${URDFDOM_MODEL_DLL}")
#      install(FILES "${URDFDOM_MODEL_DLL}" DESTINATION bin)
#    endif()
#    if(EXISTS "${URDFDOM_SENSOR_DLL}")
#      install(FILES "${URDFDOM_SENSOR_DLL}" DESTINATION bin)
#    endif()
#
#    get_target_property(TINYXML2_TYPE tinyxml2::tinyxml2 TYPE)
#    if(NOT TINYXML2_TYPE STREQUAL STATIC_LIBRARY)
#      get_target_property(TINYXML2_DLL tinyxml2::tinyxml2 IMPORTED_LOCATION_RELEASE)
#      if(EXISTS "${TINYXML2_DLL}")
#        install(FILES "${TINYXML2_DLL}" DESTINATION bin)
#      endif()
#    endif()
#endif()
