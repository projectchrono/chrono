#--------------------------------------------------------------
# Files for comparing SynChrono scaling performance to baseline Chrono::Vehicle performance
#
# Requires the SynChrono and Chrono::Vehicle modules 
#--------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE OR NOT ENABLE_MODULE_SYNCHRONO)
  return()
endif()  

# ------------------------------------------------------------------------------

set(TESTS
    btest_SCM_SYNscaling
    btest_SCM_VEHscaling
    )

# ------------------------------------------------------------------------------

set(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoEngine_synchrono
    )

if (ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    set(LIBRARIES ${LIBRARIES} ChronoEngine_irrlicht ChronoEngine_vehicle_irrlicht)
endif()

include_directories(${SYN_INCLUDES})

# ------------------------------------------------------------------------------

message(STATUS "Benchmark test programs for SCM scaling...")

foreach(PROGRAM ${TESTS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${CXX_FLAGS} ${SYN_CXX_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBRARIES})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)
