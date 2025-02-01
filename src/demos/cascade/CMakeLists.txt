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

include_directories(${OpenCASCADE_INCLUDE_DIR})

list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_cascade")

if(ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    list(APPEND LIBS "ChronoEngine_vsg")
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")	
    add_definitions( "/DWNT" )
endif()	

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_definitions(-DHAVE_IOSTREAM)
    add_definitions(-DHAVE_LIMITS_H)
endif()

#--------------------------------------------------------------

message(STATUS "Add demo programs for CASCADE module")

foreach(PROGRAM ${DEMOS_CAS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()    
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO}) 
endforeach()


