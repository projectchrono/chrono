#=============================================================================
# CMake configuration file for the SUSPENSION_TEST demo.
# These example programs work only with Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

MESSAGE(STATUS "...add SUSPENSION_TEST demos")

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMOS
    demo_SuspensionTest
    demo_SuspensionTestRig
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_irrlicht
    ChronoEngine_vehicle
    ${CH_IRRLICHTLIB}
)

#--------------------------------------------------------------
# Add executables

FOREACH(PROGRAM ${DEMOS})
  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
                        COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
  TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
  INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH()

