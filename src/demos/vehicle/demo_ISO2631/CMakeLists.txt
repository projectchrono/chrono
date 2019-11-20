#=============================================================================
# CMake configuration file for the ISO2631 demos.
# These example programs work with and without Irrlicht run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# List all ISO2631 demos

SET(DEMOS
    demo_VEH_Ride
    demo_VEH_Shock
)

#--------------------------------------------------------------
# Additional include directories

IF(ENABLE_MODULE_IRRLICHT)
    INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle)

IF(ENABLE_MODULE_IRRLICHT)
    SET(LIBRARIES_IRR ChronoEngine_irrlicht)
ELSE()
    SET(LIBRARIES_IRR "")
ENDIF()

#--------------------------------------------------------------
# Create the executables

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

FOREACH(DEMO ${DEMOS})

    MESSAGE(STATUS "...add ${DEMO}")

    ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
    SOURCE_GROUP("" FILES ${DEMO}.cpp)
    SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                          COMPILE_FLAGS "${MY_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES} ${LIBRARIES_IRR})
    INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
