#=============================================================================
# CMake configuration file for Chrono_Utils
#
# Currently this is invoked from the main CMakeLists for chrono-parallel
#=============================================================================


#
# LIST THE FILES in the ChronoEngine_Utils LIBRARY
#

SET(ChronoEngine_Utils_SOURCES
    ChUtilsCreators.cpp
    ChUtilsGenerators.cpp
    ChUtilsInputOutput.cpp
    )

SET(ChronoEngine_Utils_HEADERS
    ChUtilsCommon.h
    ChUtilsCreators.h
    ChUtilsGenerators.h
    ChUtilsSamplers.h
    ChUtilsInputOutput.h
    )

#
# INCLUDE Chrono and Chrono_Parallel HEADERS
#

INCLUDE_DIRECTORIES(
    ${CHRONO_INC}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    )

#
# ADD THE ChronoEngine_Utils LIBRARY
#

ADD_LIBRARY(ChronoEngine_Utils SHARED
    ${ChronoEngine_Utils_HEADERS}
    ${ChronoEngine_Utils_SOURCES})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")

    SET(CH_BUILDFLAGS "-DNOMINMAX")
ENDIF()

IF(MSVC)
    # Avoid deprecation warnings
    ADD_DEFINITIONS( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    ADD_DEFINITIONS( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
    
    # Set the output directory for DLLs and EXEs
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
ENDIF()


SET_TARGET_PROPERTIES(ChronoEngine_Utils PROPERTIES
  COMPILE_FLAGS "${CH_BUILDFLAGS}"
  LINK_FLAGS "${CH_LINKERFLAG_GPU}"
  COMPILE_DEFINITIONS "CH_API_COMPILE_UTILS")

TARGET_LINK_LIBRARIES(ChronoEngine_Utils ${CHRONO_LIB} ChronoEngine_Parallel)

INSTALL(TARGETS ChronoEngine_Utils
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )


IF(ENABLE_OPENGL)
    ADD_SUBDIRECTORY(opengl)
ENDIF()

