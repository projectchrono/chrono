#=============================================================================
# CMake configuration file for Chrono::PowerElectronics module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_POWERELECTRONICS "Enable the Chrono::PowerElectronics module" OFF)

IF(NOT ENABLE_MODULE_POWERELECTRONICS)
  return()
ENDIF()

message(STATUS "\n==== Chrono::PowerElectronics module ====\n")

set(CXX_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")
set(CH_POWERELECTRONICS_INCLUDES "")
set(CH_POWERELECTRONICS_LIBRARIES "ChronoEngine")
set(CH_POWERELECTRONICS_LINKED_LIBRARIES "")

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

#-----------------------------------------------------------------------------
# PyBind11
#-----------------------------------------------------------------------------
# find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
# include_directories(${Python3_INCLUDE_DIRS})

# set(EXTERNALS "../demos/powerelectronics/externals")

# list(APPEND CH_POWERELECTRONICS_LINKED_LIBRARIES ${Python3_LIBRARIES})
# list(APPEND CH_POWERELECTRONICS_INCLUDES ${Python3_INCLUDE_DIRS})

# #### TODO: MAKE PART OF THIRDPARTY?
# add_subdirectory(
#   /home/bret/chrono-wisc/src/demos/powerelectronics/externals/pybind11
#   ${CMAKE_BINARY_DIR}/pybind11_build
# )
#-----------------------------------------------------------------------------
# NgSpice
#-----------------------------------------------------------------------------
find_library(NGSPICE_LIB ngspice REQUIRED)
list(APPEND CH_POWERELECTRONICS_LINKED_LIBRARIES ${NGSPICE_LIB})

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------


# ------------
# Common Files
# ------------

set(CH_POWERELECTRONICS_BASE_FILES
  ChElectronicsCosimulation.h
	ChElectronicsCosimulation.cpp

  ChElectronicsNetlist.h
	ChElectronicsNetlist.cpp
  
  ChNgSpice.h
	ChNgSpice.cpp

  ChElectronicsCosimResult.h
  
  circuits/ChElectronicCircuit.cpp
  circuits/ChElectronicCircuit.h

  circuits/ChElectronicMotor.h
  circuits/ChElectronicMotor.cpp


)
source_group("base" FILES ${CH_POWERELECTRONICS_BASE_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_powerelectronics library
#-----------------------------------------------------------------------------

add_library(ChronoEngine_powerelectronics #SHARED
	${CH_POWERELECTRONICS_BASE_FILES}
)

# list(APPEND CH_ROS_LIBRARIES ChronoModels_robot)

# list(APPEND CH_POWERELECTRONICS_LIBRARIES pybind11::embed)

# Update compile definitions on optionally found packages

#if (ENABLE_MODULE_VEHICLE)
#	list(APPEND CH_ROS_LIBRARIES ChronoEngine_vehicle)
#  if (ENABLE_MODULE_VEHICLE_MODELS)
#    list(APPEND CH_ROS_LIBRARIES ChronoModels_vehicle)
#  endif()
#endif()

#if (ENABLE_MODULE_SENSOR)
#	list(APPEND CH_ROS_LIBRARIES ChronoEngine_sensor)
#endif()

#if (ENABLE_MODULE_PARSERS AND HAVE_URDF)
# list(APPEND CH_ROS_LIBRARIES ChronoEngine_parsers)
#  target_compile_definitions(ChronoEngine_ros PUBLIC -DCHRONO_PARSERS_URDF)
#endif()

# Set some variables to be visible outside this directory
#set(CH_ROS_INCLUDES   "${CH_ROS_INCLUDES}"   PARENT_SCOPE)
#set(CH_ROS_LIBRARIES  "${CH_ROS_LIBRARIES}"  PARENT_SCOPE)
#set(CH_ROS_HAS_INTERFACES ${CH_ROS_HAS_INTERFACES} PARENT_SCOPE)

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")
endif()

set_target_properties(ChronoEngine_powerelectronics PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")

set_target_properties(ChronoEngine_powerelectronics PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(ChronoEngine_powerelectronics PRIVATE "CH_API_COMPILE_POWERELECTRONICS")

target_include_directories(ChronoEngine_powerelectronics PUBLIC ${CH_POWERELECTRONICS_INCLUDES})

target_link_libraries(ChronoEngine_powerelectronics ${CH_POWERELECTRONICS_LIBRARIES} ${CH_POWERELECTRONICS_LINKED_LIBRARIES})

install(TARGETS ChronoEngine_powerelectronics
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

#-------------------------------------------------------------------------------
# Install Chrono::PowerElectronics headers
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_powerelectronics
        FILES_MATCHING PATTERN "*.h" PATTERN "fbs" EXCLUDE)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CH_CONFIG_INSTALL_PATH "cmake")
else()
    set(CH_CONFIG_INSTALL_PATH "lib/cmake/Chrono")
endif()

# install(FILES ${CMAKE_SOURCE_DIR}/cmake/FindROS2.cmake DESTINATION ${CH_CONFIG_INSTALL_PATH} RENAME ros2-config.cmake)
