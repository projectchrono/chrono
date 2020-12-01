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

if(ENABLE_MODULE_PARALLEL)
  include_directories(${CH_PARALLEL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_PARALLEL_CXX_FLAGS}")
  set(LIBRARIES ${LIBRARIES} ChronoEngine_parallel)
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

if(ENABLE_MODULE_DISTRIBUTED)
  include_directories(${CH_DISTRIBUTED_INCLUDES})
  set(LIBRARIES ${LIBRARIES} ChronoEngine_distributed)
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_DISTRIBUTED_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_DISTRIBUTED_LINK_FLAGS}")
endif()

# ------------------------------------------------------------------------------

#message("VEH_COSIM DEMO:  CXX FLAGS: ${COMPILER_FLAGS}")
#message("VEH_COSIM DEMO:  LNK FLAGS: ${LINKER_FLAGS}")
#message("VEH_COSIM DEMO:  LIBRARIES: ${LIBRARIES}")

set(PROGRAM demo_VEH_Cosim_WheelRig)
source_group("" FILES ${PROGRAM}.cpp)

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${PROGRAM}.cpp)

set_target_properties(${PROGRAM} PROPERTIES 
    COMPILE_FLAGS "${COMPILER_FLAGS}"
    LINK_FLAGS "${LINKER_FLAGS}")

target_link_libraries(${PROGRAM} ${LIBRARIES})
add_dependencies(${PROGRAM} ${LIBRARIES})
