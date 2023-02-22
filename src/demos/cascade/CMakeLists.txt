#=============================================================================
# CMake configuration file for various Chrono::Cascade demos.
# These example programs require run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# Additional paths and settings

include_directories(${OpenCASCADE_INCLUDE_DIR})
set(COMPILER_FLAGS "${CH_CXX_FLAGS} ${OpenCASCADE_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE} ${OpenCASCADE_LINKER_FLAGS}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_cascade")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHTINC})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    set(CMAKE_CXX_STANDARD 17)

    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")	
    add_definitions( "/DWNT" )
endif()	

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_definitions(-DHAVE_IOSTREAM)
    add_definitions(-DHAVE_LIMITS_H)
endif()

#--------------------------------------------------------------
# List of all Cascade demos

set(DEMOS_CAS
	demo_CAS_converter
	demo_CAS_stepfile
	demo_CAS_robot
	demo_CAS_profile
)

message(STATUS "Demo programs for OpenCASCADE module...")

#--------------------------------------------------------------
# Add demo executables

foreach(PROGRAM ${DEMOS_CAS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()    
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO}) 
endforeach()


