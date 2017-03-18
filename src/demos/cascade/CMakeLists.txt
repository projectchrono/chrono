#--------------------------------------------------------------
# Additional include paths and settings

INCLUDE_DIRECTORIES("${CH_CASCADEINC}")

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")	
    ADD_DEFINITIONS( "/DWNT" )
ENDIF()	

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
ENDIF()

#--------------------------------------------------------------
# List of all Cascade demos which also require Irrlicht support

set(DEMOS_CAS
	demo_CAS_converter
	demo_CAS_stepfile
	demo_CAS_robot
)

message(STATUS "Demo programs for CASCADE module...")

#--------------------------------------------------------------
# Add demo executables that use Irrlicht

if(ENABLE_MODULE_IRRLICHT)

   INCLUDE_DIRECTORIES("${CH_IRRLICHTINC}")

   FOREACH(PROGRAM ${DEMOS_CAS})
        MESSAGE(STATUS "...add ${PROGRAM}")

        ADD_EXECUTABLE(${PROGRAM} "${PROGRAM}.cpp")
        SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")
        SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
            FOLDER demos
            COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_EXE}")
        TARGET_LINK_LIBRARIES(${PROGRAM}
            ${CH_IRRLICHTLIB}
            ${CH_CASCADELIBS}
            ChronoEngine
            ChronoEngine_cascade
            ChronoEngine_irrlicht)
        ADD_DEPENDENCIES(${PROGRAM}
            ChronoEngine
            ChronoEngine_cascade
            ChronoEngine_irrlicht)
        INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO}) 
    ENDFOREACH()

endif()

