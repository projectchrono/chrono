# Test programs for HMMWV simulation
# ------------------------------------------------------------------------------
if(NOT ENABLE_MODULE_VEHICLE)
  return()
endif()

if(NOT ENABLE_MODULE_GRANULAR)
  return()
endif()

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

include_directories(${CH_GRANULAR_INCLUDES})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

# ------------------------------------------------------------------------------

set(PROGRAMS
  test_GRAN_HMMWV
)

set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle
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
        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS} ${CH_GRANULAR_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )
    target_link_libraries(${PROGRAM} ${LIBRARIES})
	add_dependencies(${PROGRAM} ${LIBRARIES})
endforeach(PROGRAM)