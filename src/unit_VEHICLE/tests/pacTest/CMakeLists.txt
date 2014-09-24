# ----------------------
# Configuration options
# ----------------------
INCLUDE(CMakeDependentOption)

OPTION(PACTEST_ENABLE "enable test of single ChPacejkaTire" OFF)

IF(NOT PACTEST_ENABLE)
  RETURN()
ENDIF()

MESSAGE(STATUS "Adding ChPacejkaTire single tire tests...")

SET(LIBRARIES 
    ${CHRONOENGINE_LIBRARIES}
    Chrono_Utils
    Chrono_Subsys
)

IF(ENABLE_IRRLICHT AND ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  SET(CH_BUILDFLAGS "${CH_BUILDFLAGS} /wd4275")
ENDIF()

SET(TEST_PACTIRE_FILES
  test_pacTire.cpp
  )

SOURCE_GROUP("" FILES ${TEST_PACTIRE_FILES} )

# create a test of the pac tire for kinematically driven cases
# e.g., validation
ADD_EXECUTABLE(test_pacTire ${TEST_PACTIRE_FILES})
SET_TARGET_PROPERTIES(test_pacTire PROPERTIES
                      FOLDER tests
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_EXE}")

TARGET_LINK_LIBRARIES(test_pacTire ${LIBRARIES})
INSTALL(TARGETS test_pacTire DESTINATION bin)
