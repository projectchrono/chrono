# Test programs for RoboSimian simulation
# ------------------------------------------------------------------------------
# MESSAGE(WARNING "HERE!!!!")
if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT ENABLE_MODULE_IRRLICHT)
  return()
endif()

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_IRRLICHTINC})
include_directories(${CH_GRANULAR_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS
  demo_GRAN_TriMeshNoFric_SMC_robosimian
  test_RS_robosimian_granular
)

set(MODEL_FILES
    robosimian.h
    robosimian.cpp
    # granular.h
    # granular.cpp
)

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_irrlicht
    ChronoEngine_granular
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

message(STATUS "Test programs for GRANULAR RoboSimian...")

foreach(PROGRAM ${PROGRAMS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp" ${MODEL_FILES})
    source_group(""  FILES "${PROGRAM}.cpp" ${MODEL_FILES})

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS} ${CH_GRANULAR_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
		ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})
                          

endforeach(PROGRAM)

