#==============================================================================
# CMake configuration file for the RCCar demo.
# This example program requires Irrlicht run-time visualization.
#==============================================================================

#------------------------------------------------------------------------------
# Check dependencies.

IF(NOT ENABLE_MODULE_IRRLICHT)
  RETURN()
ENDIF()

#------------------------------------------------------------------------------
# List all model files

SET(DEMOS
    demo_VEH_RCCar
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executables

FOREACH(DEMO ${DEMOS})

    MESSAGE(STATUS "...add ${DEMO}")

    ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp)
    SOURCE_GROUP("" FILES ${DEMO}.cpp)
    SET_TARGET_PROPERTIES(${DEMO} PROPERTIES
      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
      LINK_FLAGS "${CH_LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO}
      ChronoEngine
      ChronoEngine_irrlicht
      ChronoEngine_vehicle
      ChronoEngine_vehicle_irrlicht
      ChronoModels_vehicle)
INSTALL(TARGETS ${DEMO} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()
