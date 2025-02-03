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

if(ENABLE_MODULE_MULTICORE)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularOMP.h
      terrain/ChVehicleCosimTerrainNodeGranularOMP.cpp)
endif()

if(ENABLE_MODULE_FSI)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularSPH.h
      terrain/ChVehicleCosimTerrainNodeGranularSPH.cpp)
endif()

if(ENABLE_MODULE_GPU)
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
# Add the ChronoEngine_vehicle_cosim library

message(STATUS "Add ChronoEngine_vehicle_cosim library")

add_library(ChronoEngine_vehicle_cosim
    ${CV_COSIM_FILES}
    ${CV_COSIM_MBS_FILES}
    ${CV_COSIM_TIRE_FILES}
    ${CV_COSIM_TERRAIN_FILES}
)

add_library(ChronoEngine::ChronoEngine_vehicle_cosim ALIAS ChronoEngine_vehicle_cosim)

target_compile_definitions(ChronoEngine_vehicle_cosim PRIVATE "CH_API_COMPILE_VEHICLE")

target_link_libraries(ChronoEngine_vehicle_cosim PUBLIC ChronoEngine_vehicle)
target_link_libraries(ChronoEngine_vehicle_cosim PUBLIC ChronoModels_robot)
if(ENABLE_MODULE_MULTICORE)
  target_link_libraries(ChronoEngine_vehicle_cosim PUBLIC ChronoEngine_multicore)
endif()
if(ENABLE_MODULE_FSI)
  target_link_libraries(ChronoEngine_vehicle_cosim PUBLIC ChronoEngine_fsisph)
endif()
if(ENABLE_MODULE_GPU)
  target_link_libraries(ChronoEngine_vehicle_cosim PUBLIC ChronoEngine_gpu)
endif()

target_link_libraries(ChronoEngine_vehicle_cosim PUBLIC MPI::MPI_CXX)

if(ENABLE_MODULE_PARDISO_MKL)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_pardisomkl)
endif()
if(ENABLE_MODULE_MUMPS)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_mumps)
endif()
if(ENABLE_MODULE_POSTPROCESS)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_postprocess)
endif()
if(ENABLE_MODULE_IRRLICHT)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_irrlicht)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_vehicle_irrlicht)
endif()
if(ENABLE_MODULE_VSG)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_vsg)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_vehicle_vsg)
  if(ENABLE_MODULE_FSI)
    target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_fsisph_vsg)
  endif()
endif()
if(ENABLE_MODULE_OPENGL)
  target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_opengl)
  if(ENABLE_MODULE_FSI)
    target_link_libraries(ChronoEngine_vehicle_cosim PRIVATE ChronoEngine_fsisph_opengl)
  endif()
endif()


install(TARGETS ChronoEngine_vehicle_cosim
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vehicle/cosim)
