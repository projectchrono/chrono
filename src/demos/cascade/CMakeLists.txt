#--------------------------------------------------------------
# Additional paths and settings

include_directories(${OpenCASCADE_INCLUDE_DIR})
#link_directories(${OpenCASCADE_LIBRARY_DIR})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")	
    add_definitions( "/DWNT" )
endif()	

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_definitions(-DHAVE_IOSTREAM)
    add_definitions(-DHAVE_LIMITS_H)
endif()

#--------------------------------------------------------------
# List of all Cascade demos which also require Irrlicht support

set(DEMOS_CAS
	demo_CAS_converter
	demo_CAS_stepfile
	demo_CAS_robot
	demo_CAS_profile
)

message(STATUS "Demo programs for OpenCASCADE module...")

#--------------------------------------------------------------
# Add demo executables that use Irrlicht

if(ENABLE_MODULE_IRRLICHT)

   include_directories(${CH_IRRLICHTINC})

   foreach(PROGRAM ${DEMOS_CAS})
        message(STATUS "...add ${PROGRAM}")

        add_executable(${PROGRAM} "${PROGRAM}.cpp")
        source_group(""  FILES "${PROGRAM}.cpp")

        set_target_properties(${PROGRAM} PROPERTIES 
            FOLDER demos
            COMPILE_FLAGS "${CH_CXX_FLAGS} ${OpenCASCADE_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_EXE} ${OpenCASCADE_LINKER_FLAGS}")
        set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
        target_link_libraries(${PROGRAM}
            ChronoEngine
            ChronoEngine_cascade
            ChronoEngine_irrlicht)

        install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO}) 
    endforeach()

endif()
