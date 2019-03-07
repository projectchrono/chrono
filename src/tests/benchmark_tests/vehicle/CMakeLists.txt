# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_IRRLICHT)
    return()
endif()

if(NOT ENABLE_MODULE_VEHICLE)
    return()
endif()
    
# ------------------------------------------------------------------------------

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

include_directories(${CH_IRRLICHTINC})
set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
list(APPEND LIBS "ChronoEngine_irrlicht")

list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

#--------------------------------------------------------------

include_directories(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googletest/googletest/include)
list(APPEND LIBS gtest_main)

include_directories(${PROJECT_SOURCE_DIR}/src/chrono_thirdparty/googlebenchmark/include)
list(APPEND LIBS benchmark_main)

# ------------------------------------------------------------------------------

set(TESTS
    btest_VEH_m113Acc
)

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER tests
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}"
    )
    target_link_libraries(${PROGRAM} ${LIBS})
endforeach(PROGRAM)
