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

# ------------------------------------------------------------------------------

# Set flags, include paths, and libraries

list(APPEND LIBRARIES ChronoEngine_vehicle)
list(APPEND LIBRARIES ChronoModels_robot)
list(APPEND LIBRARIES "${MPI_CXX_LIBRARIES}")
set(LINKER_FLAGS "${CH_LINKERFLAG_LIB} ${MPI_CXX_LINK_FLAGS}")
set(INCLUDES "${CH_INCLUDES};${MPI_CXX_INCLUDE_PATH}")
set(CXX_FLAGS "${CH_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}")

if(ENABLE_MODULE_MULTICORE)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularOMP.h
      terrain/ChVehicleCosimTerrainNodeGranularOMP.cpp)
  source_group("terrain" FILES)
  set(INCLUDES "${INCLUDES};${CH_MULTICORE_INCLUDES}")
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_multicore)
endif()

if(ENABLE_MODULE_FSI)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularSPH.h
      terrain/ChVehicleCosimTerrainNodeGranularSPH.cpp)
  set(INCLUDES "${INCLUDES};${CH_FSI_INCLUDES}")
  list(APPEND LIBRARIES ChronoEngine_fsi)
endif()

if(ENABLE_MODULE_GPU)
  set(CV_COSIM_TERRAIN_FILES ${CV_COSIM_TERRAIN_FILES}
      terrain/ChVehicleCosimTerrainNodeGranularGPU.h
      terrain/ChVehicleCosimTerrainNodeGranularGPU.cpp)
  set(INCLUDES "${INCLUDES};${CH_GPU_INCLUDES}")
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_GPU_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_gpu)
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  set(INCLUDES "${INCLUDES};${CH_MKL_INCLUDES}")
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_pardisomkl)
endif()

if(ENABLE_MODULE_MUMPS)
  set(INCLUDES "${INCLUDES};${CH_MUMPS_INCLUDES}")
  list(APPEND LIBRARIES ChronoEngine_mumps)
endif()

if(ENABLE_MODULE_IRRLICHT)
  set(INCLUDES "${INCLUDES};${CH_IRRLICHT_INCLUDES}")
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_irrlicht)
  list(APPEND LIBRARIES ChronoEngine_vehicle_irrlicht)
endif()

if(ENABLE_MODULE_VSG)
  include_directories(${CH_VSG_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_vsg)
  list(APPEND LIBRARIES ChronoEngine_vehicle_vsg)
endif()

if(ENABLE_MODULE_OPENGL)
  set(INCLUDES "${INCLUDES};${CH_OPENGL_INCLUDES}")
  list(APPEND LIBRARIES ChronoEngine_opengl)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(LINKER_FLAGS  "${LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
endif()

if(ENABLE_MODULE_POSTPROCESS)
    list(APPEND LIBRARIES ChronoEngine_postprocess)
endif()

# ------------------------------------------------------------------------------
# Make some variables visible from parent directory (for use in demo CMakeLists files)

unset(CH_VEHCOSIM_INCLUDES CACHE)
unset(CH_VEHCOSIM_CXX_FLAGS CACHE)
unset(CH_VEHCOSIM_LINKER_FLAGS CACHE)
unset(CH_VEHCOSIM_LIBRARIES CACHE)

set(CH_VEHCOSIM_INCLUDES "${INCLUDES}" CACHE INTERNAL "")
set(CH_VEHCOSIM_CXX_FLAGS "${CXX_FLAGS}" CACHE INTERNAL "")
set(CH_VEHCOSIM_LINKER_FLAGS "${LINKER_FLAGS}" CACHE INTERNAL "")
set(CH_VEHCOSIM_LIBRARIES "${LIBRARIES}" CACHE INTERNAL "")

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

include_directories(${INCLUDES})

set_target_properties(ChronoEngine_vehicle_cosim PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${LINKER_FLAGS}")

target_compile_definitions(ChronoEngine_vehicle_cosim PRIVATE "CH_API_COMPILE_VEHICLE")
target_compile_definitions(ChronoEngine_vehicle_cosim PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_vehicle_cosim ${LIBRARIES})

install(TARGETS ChronoEngine_vehicle_cosim
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
