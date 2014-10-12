# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(ENABLE_JOINTS_TESTS "Enable Chrono joint validation tests" OFF)

IF(NOT ENABLE_JOINTS_TESTS)
    RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding joint validation tests...")

SET(TEST_PROGRAMS
    test_revolute
    test_spherical
    test_universal
    test_hooke
    test_revsph
)


SET(LIBRARIES 
    ${CHRONOENGINE_LIBRARIES}
    Chrono_Utils
    Chrono_Subsys
)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()


# Add executables
FOREACH(PROGRAM ${TEST_PROGRAMS})
  MESSAGE(STATUS "... ${PROGRAM}")
  
  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    FOLDER tests
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

  TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})

  INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH()

