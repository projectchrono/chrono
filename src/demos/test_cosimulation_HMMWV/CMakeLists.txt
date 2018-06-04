# Co-simulation full vehicle (HMMWV)
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT ENABLE_MODULE_FEA OR NOT ENABLE_MODULE_MKL)
  return()
endif()

if(NOT ENABLE_MODULE_GRANULAR)
  return()
endif()

# ------------------------------------------------------------------------------

include_directories(${CH_MKL_INCLUDES})
include_directories(${CH_GRANULAR_INCLUDES})
include_directories(${MPI_CXX_INCLUDE_PATH})

set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${CH_GRANULAR_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")

set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle
    ChronoEngine_fea
    ChronoEngine_mkl
    ChronoEngine_granular
)

if(ENABLE_MODULE_GRANULAR)
    include_directories(${CH_GRANULAR_INCLUDES})
    set(LIBRARIES ${LIBRARIES} ChronoEngine_granular)
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_GRANULAR_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_GRANULAR_LINK_FLAGS}")
endif()

# ------------------------------------------------------------------------------

message(STATUS "Test programs for CO-SIMULATION...")

set(COMMON_FILES
    BaseNode.h
    BaseNode.cpp
    VehicleNode.h
    VehicleNode.cpp
    TireNode.h
    TireNode.cpp
    )

if(ENABLE_MODULE_GRANULAR)
    set(PROGRAM test_cosimulation_HMMWV_GRAN)
    set(TEST_FILES "${COMMON_FILES}" TerrainNodeGran.h TerrainNodeGran.cpp)
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} ${PROGRAM}.cpp ${TEST_FILES})
    source_group("" FILES ${PROGRAM}.cpp ${TEST_FILES})
    set_target_properties(${PROGRAM} PROPERTIES
                          COMPILE_FLAGS "${COMPILER_FLAGS}"
                          LINK_FLAGS "${LINKER_FLAGS}")
    target_link_libraries(${PROGRAM} ${LIBRARIES})
endif()
