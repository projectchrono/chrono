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
    test_linactuator
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


IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(WORK_DIR ${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>)
ELSE()
  SET(WORK_DIR ${PROJECT_BINARY_DIR}/bin)
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

  # Note: this is not intended to work on Windows!
  ADD_TEST(NAME ${PROGRAM}
           WORKING_DIRECTORY ${WORK_DIR}
           COMMAND ${WORK_DIR}/${PROGRAM}
           )

ENDFOREACH()

