#=============================================================================
# CMake configuration file for the SUSPENSION_TEST_RIG demo.
# These example programs require Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMOS
    demo_VEH_SuspensionTestRigPlatform
    demo_VEH_SuspensionTestRigPushrod
)

SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
)

#--------------------------------------------------------------
# Add executable

FOREACH(DEMO ${DEMOS})

   MESSAGE(STATUS "...add ${DEMO}")

   ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
   SOURCE_GROUP("" FILES ${DEMO}.cpp)
   SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                         COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                         LINK_FLAGS "${LINKERFLAG_EXE}")
   TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES})
   INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
