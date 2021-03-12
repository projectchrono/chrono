#=============================================================================
# CMake configuration file for Chrono Vehicle cosimulation library
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

# Return now if MPI is not available
if(NOT MPI_CXX_FOUND)
	message("Chrono::Vehicle cosimulation support requires MPI")
	message(STATUS "Cosimulation support disabled")
	return()
endif()

# ------------------------------------------------------------------------------

set(CV_COSIM_FILES
    ChVehicleCosimBaseNode.h
    ChVehicleCosimBaseNode.cpp
    ChVehicleCosimRigNode.h
    ChVehicleCosimRigNode.cpp
    ChVehicleCosimTerrainNode.h
    ChVehicleCosimTerrainNode.cpp
    ChVehicleCosimTerrainNodeSCM.h
    ChVehicleCosimTerrainNodeSCM.cpp
    )

# ------------------------------------------------------------------------------

# Note, inherit CXX_FLAGS and LIBRARIES from parent Chrono::Vehicle CMakeLists.
# These already include MPI and (if available) Irrlicht.

set(LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")

if(ENABLE_MODULE_MULTICORE)
  set(CV_COSIM_FILES ${CV_COSIM_FILES}
      ChVehicleCosimTerrainNodeRigid.h
      ChVehicleCosimTerrainNodeRigid.cpp
      ChVehicleCosimTerrainNodeGranularOMP.h
      ChVehicleCosimTerrainNodeGranularOMP.cpp)
  include_directories(${CH_MULTICORE_INCLUDES})
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_multicore)
endif()

if(ENABLE_MODULE_FSI)
  set(CV_COSIM_FILES ${CV_COSIM_FILES}
      ChVehicleCosimTerrainNodeGranularSPH.h
      ChVehicleCosimTerrainNodeGranularSPH.cpp)
  include_directories(${CH_FSI_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_fsi)
endif()

if(ENABLE_MODULE_GPU)
  set(CV_COSIM_FILES ${CV_COSIM_FILES}
      ChVehicleCosimTerrainNodeGranularGPU.h
      ChVehicleCosimTerrainNodeGranularGPU.cpp)
  include_directories(${CH_GPU_INCLUDES})
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_GPU_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_gpu)
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_pardisomkl)
endif()

#if(ENABLE_MODULE_MUMPS)
#  include_directories(${CH_MUMPS_INCLUDES})
#  list(APPEND LIBRARIES ChronoEngine_mumps)
#endif()

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHTINC})
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_irrlicht)
endif()

if(ENABLE_MODULE_OPENGL)
  include_directories(${CH_OPENGL_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_opengl)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(LINKER_FLAGS  "${LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
endif()

if(ENABLE_MODULE_DISTRIBUTED)
  include_directories(${CH_DISTRIBUTED_INCLUDES})
  list(APPEND LIBRARIES ChronoEngine_distributed)
  set(CXX_FLAGS "${CXX_FLAGS} ${CH_DISTRIBUTED_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_DISTRIBUTED_LINK_FLAGS}")
endif()

# ------------------------------------------------------------------------------

# Add the ChronoEngine_vehicle_cosim library

source_group("" FILES ${CV_COSIM_FILES})

message(STATUS "Add ChronoEngine_vehicle_cosim library")

#message("VEH_COSIM:  CXX FLAGS: ${CXX_FLAGS}")
#message("VEH_COSIM:  LNK FLAGS: ${LINKER_FLAGS}")
#message("VEH_COSIM:  LIBRARIES: ${LIBRARIES}")

add_library(ChronoEngine_vehicle_cosim SHARED
    ${CV_COSIM_FILES}
)

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
