if(NOT ENABLE_MODULE_VEHICLE)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(TESTS
    btest_VEH_hmmwvDLC
    btest_VEH_m113Acc
    )

# ------------------------------------------------------------------------------

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHTINC})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

# ------------------------------------------------------------------------------

message(STATUS "Benchmark test programs for VEHICLE module...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )
    target_link_libraries(${PROGRAM} ${LIBS} benchmark_main)
endforeach(PROGRAM)
