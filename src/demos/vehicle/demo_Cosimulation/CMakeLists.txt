# Co-simulation tire test rig
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

# ------------------------------------------------------------------------------

include_directories(${CH_INCLUDES})
include_directories(${MPI_CXX_INCLUDE_PATH})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE} ${MPI_CXX_LINK_FLAGS}")

set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_vehicle_cosim
    ChronoModels_vehicle
)

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

if(ENABLE_MODULE_MULTICORE)
  include_directories(${CH_MULTICORE_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MULTICORE_CXX_FLAGS}")
  set(LIBRARIES ${LIBRARIES} ChronoEngine_multicore)
endif()

if(ENABLE_MODULE_DISTRIBUTED)
  include_directories(${CH_DISTRIBUTED_INCLUDES})
  set(LIBRARIES ${LIBRARIES} ChronoEngine_distributed)
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_DISTRIBUTED_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_DISTRIBUTED_LINK_FLAGS}")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  set(LIBRARIES ${LIBRARIES} ChronoEngine_pardisomkl)
endif()

#if(ENABLE_MODULE_MUMPS)
#  include_directories(${CH_MUMPS_INCLUDES})
#  set(LIBRARIES ${LIBRARIES} ChronoEngine_mumps)
#endif()

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHTINC})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBRARIES ChronoEngine_irrlicht)
endif()

if(ENABLE_MODULE_OPENGL)
  include_directories(${CH_OPENGL_INCLUDES})
  set(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
     set(LINKER_FLAGS  "${LINKER_FLAGS} -framework IOKit -framework Cocoa -framework OpenGL")
  endif()
endif()

# ------------------------------------------------------------------------------

#message("VEH_COSIM DEMO:  CXX FLAGS: ${COMPILER_FLAGS}")
#message("VEH_COSIM DEMO:  LNK FLAGS: ${LINKER_FLAGS}")
#message("VEH_COSIM DEMO:  LIBRARIES: ${LIBRARIES}")

set(PROGRAMS 
    demo_VEH_Cosim_WheelRig
    demo_VEH_Cosim_WheelRig_CustomTerrain
    demo_VEH_Cosim_WheeledVehicle
    )

FOREACH(PROGRAM ${PROGRAMS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  source_group("" FILES ${PROGRAM}.cpp)

  set_target_properties(${PROGRAM} PROPERTIES 
      COMPILE_FLAGS "${COMPILER_FLAGS}"
      LINK_FLAGS "${LINKER_FLAGS}")

  target_link_libraries(${PROGRAM} ${LIBRARIES})
ENDFOREACH()
