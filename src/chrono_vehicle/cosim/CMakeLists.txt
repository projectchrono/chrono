#=============================================================================
# CMake configuration file for Chrono Vehicle cosimulation library
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

# Return now if MPI is not available
if(NOT MPI_FOUND)
	message(WARNING "Chrono::Vehicle cosimulation support requires MPI, but MPI not found")
	message(STATUS "Cosimulation support disabled")
	return()
endif()

# ------------------------------------------------------------------------------

set(CV_COSIM_FILES
    ChVehicleCosimBaseNode.h
    ChVehicleCosimBaseNode.cpp
    ChVehicleCosimWheeledMBSNode.h
    ChVehicleCosimWheeledMBSNode.cpp
    ChVehicleCosimTrackedMBSNode.h
    ChVehicleCosimTrackedMBSNode.cpp
    ChVehicleCosimTireNode.h
    ChVehicleCosimTireNode.cpp
    ChVehicleCosimTerrainNode.h
    ChVehicleCosimTerrainNode.cpp
    ChVehicleCosimOtherNode.h
    ChVehicleCosimDBPRig.h
    ChVehicleCosimDBPRig.cpp
)

set(CV_COSIM_MBS_FILES
    mbs/ChVehicleCosimRigNode.h
    mbs/ChVehicleCosimRigNode.cpp
    mbs/ChVehicleCosimWheeledVehicleNode.h
    mbs/ChVehicleCosimWheeledVehicleNode.cpp
    mbs/ChVehicleCosimTrackedVehicleNode.h
    mbs/ChVehicleCosimTrackedVehicleNode.cpp
    mbs/ChVehicleCosimViperNode.h
    mbs/ChVehicleCosimViperNode.cpp
    mbs/ChVehicleCosimCuriosityNode.h
    mbs/ChVehicleCosimCuriosityNode.cpp
)

set(CV_COSIM_TIRE_FILES
    tire/ChVehicleCosimTireNodeRigid.h
    tire/ChVehicleCosimTireNodeRigid.cpp
    tire/ChVehicleCosimTireNodeFlexible.h
    tire/ChVehicleCosimTireNodeFlexible.cpp
    tire/ChVehicleCosimTireNodeBypass.h
    tire/ChVehicleCosimTireNodeBypass.cpp
)

set(CV_COSIM_TERRAIN_FILES
    terrain/ChVehicleCosimTerrainNodeChrono.h
    terrain/ChVehicleCosimTerrainNodeChrono.cpp
    terrain/ChVehicleCosimTerrainNodeRigid.h
    terrain/ChVehicleCosimTerrainNodeRigid.cpp
    terrain/ChVehicleCosimTerrainNodeSCM.h
    terrain/ChVehicleCosimTerrainNodeSCM.cpp
)

if(CH_ENABLE_MODULE_MULTICORE)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularOMP.h
      terrain/ChVehicleCosimTerrainNodeGranularOMP.cpp)
endif()

if(CH_ENABLE_MODULE_FSI)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularSPH.h
      terrain/ChVehicleCosimTerrainNodeGranularSPH.cpp)
endif()

if(CH_ENABLE_MODULE_GPU)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularGPU.h
      terrain/ChVehicleCosimTerrainNodeGranularGPU.cpp)
endif()

# ------------------------------------------------------------------------------

source_group("" FILES ${CV_COSIM_FILES})
source_group("mbs" FILES ${CV_COSIM_MBS_FILES})
source_group("tire" FILES ${CV_COSIM_TIRE_FILES})
source_group("terrain" FILES ${CV_COSIM_TERRAIN_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_VEHICLECOSIM "")
if(CH_ENABLE_MODULE_VEHICLE)
  list (APPEND DEPENDENCIES_VEHICLECOSIM "VEHICLE")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list (APPEND DEPENDENCIES_VEHICLECOSIM "PARDISOMKL")
endif()
##if(CH_ENABLE_MODULE_MUMPS)
##  list (APPEND DEPENDENCIES_VEHICLECOSIM "MUMPS")
##endif()

if(CH_ENABLE_MODULE_MULTICORE)
  list (APPEND DEPENDENCIES_VEHICLECOSIM "MULTICORE")
endif()
if(CH_ENABLE_MODULE_FSI)
  list (APPEND DEPENDENCIES_VEHICLECOSIM "FSI")
endif()
if(CH_ENABLE_MODULE_GPU)
  list (APPEND DEPENDENCIES_VEHICLECOSIM "GPU")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  list (APPEND DEPENDENCIES_VEHICLECOSIM "POSTPROCESS")
endif()

set(DEPENDENCIES_VEHICLECOSIM ${DEPENDENCIES_VEHICLECOSIM} PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Add the Chrono_vehicle_cosim library
# ------------------------------------------------------------------------------

message(STATUS "Add Chrono_vehicle_cosim library")

add_library(Chrono_vehicle_cosim
    ${CV_COSIM_FILES}
    ${CV_COSIM_MBS_FILES}
    ${CV_COSIM_TIRE_FILES}
    ${CV_COSIM_TERRAIN_FILES}
)
add_library(Chrono::vehicle_cosim ALIAS Chrono_vehicle_cosim)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_vehicle_cosim PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_vehicle_cosim PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_vehicle_cosim PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_vehicle_cosim PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_vehicle_cosim PRIVATE "CH_API_COMPILE_VEHICLE")

target_link_libraries(Chrono_vehicle_cosim PUBLIC Chrono_vehicle)
target_link_libraries(Chrono_vehicle_cosim PUBLIC ChronoModels_robot)
if(CH_ENABLE_MODULE_MULTICORE)
  target_link_libraries(Chrono_vehicle_cosim PUBLIC Chrono_multicore)
endif()
if(CH_ENABLE_MODULE_FSI)
  target_link_libraries(Chrono_vehicle_cosim PUBLIC Chrono_fsisph)
endif()
if(CH_ENABLE_MODULE_GPU)
  target_link_libraries(Chrono_vehicle_cosim PUBLIC Chrono_gpu)
endif()

target_link_libraries(Chrono_vehicle_cosim PUBLIC MPI::MPI_CXX)

if(CH_ENABLE_MODULE_PARDISO_MKL)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_pardisomkl)
endif()
##if(CH_ENABLE_MODULE_MUMPS)
##  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_mumps)
##endif()
if(CH_ENABLE_MODULE_POSTPROCESS)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_postprocess)
endif()
if(CH_ENABLE_MODULE_IRRLICHT)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_irrlicht)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_vehicle_irrlicht)
endif()
if(CH_ENABLE_MODULE_VSG)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_vsg)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_vehicle_vsg)
  if(CH_ENABLE_MODULE_FSI)
    target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_fsisph_vsg)
  endif()
endif()
if(CH_ENABLE_MODULE_OPENGL)
  target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_opengl)
  if(CH_ENABLE_MODULE_FSI)
    target_link_libraries(Chrono_vehicle_cosim PRIVATE Chrono_fsisph_opengl)
  endif()
endif()

install(TARGETS Chrono_vehicle_cosim
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vehicle/cosim)

# -------------------------------------------------------------------------------------------------
# Install files
# -------------------------------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project_vehicle_cosim/ DESTINATION "template_project_vehicle_cosim")
else()
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project_vehicle_cosim/ DESTINATION "share/chrono/template_project_vehicle_cosim")
endif()

# Old way (install headers preserving directory structure)
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
#        DESTINATION include/chrono_vehicle/cosim
#        FILES_MATCHING PATTERN "*.h")
