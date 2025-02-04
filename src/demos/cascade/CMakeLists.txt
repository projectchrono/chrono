#=============================================================================
# CMake configuration file for various Chrono::Cascade demos.
# These example programs require run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List of all Cascade demos

set(DEMOS_CAS
	demo_CAS_converter
	demo_CAS_stepfile
	demo_CAS_robot
	demo_CAS_profile
)

#--------------------------------------------------------------
# Additional paths and settings

list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_cascade")

if(ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    list(APPEND LIBS "ChronoEngine_vsg")
endif()

#--------------------------------------------------------------

message(STATUS "Add demo programs for CASCADE module")

foreach(PROGRAM ${DEMOS_CAS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    if(WIN32)
      target_compile_definitions(ChronoEngine_cascade PRIVATE WNT)
    endif()
    if(UNIX)
      target_compile_definitions(ChronoEngine_cascade PRIVATE HAVE_IOSTREAM)
      target_compile_definitions(ChronoEngine_cascade PRIVATE HAVE_LIMITS_H)
    endif()
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()    

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO}) 
endforeach()


