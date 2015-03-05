OPTION(ENABLE_DEMOS_VEHICLE "Build the demos using ChronoVehicle" OFF)

IF(NOT ENABLE_DEMOS_VEHICLE)
    RETURN()
ENDIF()


#--------------------------------------------------------------
# Find the ChronoVehicle library

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
FIND_PACKAGE(ChronoVehicle)

# Stop now if we could not find the ChronoVehicle SDK
IF(NOT EXISTS "${CH_VEHICLE_SDKDIR}/subsys/ChVehicle.h")
  MESSAGE(FATAL_ERROR "Cannot find the ChronoVehicle SDK. Make sure CH_VEHICLE_SDKDIR is set correctly.")
  RETURN()
ENDIF()

# Check debug/release ChronoVehicle libraries. Stop now if neither could be found.
IF(CHRONOVEHICLE_LIBRARY_DEBUG)
  MESSAGE(STATUS "ChronoVehicle library (debug) found")
ENDIF()
IF(CHRONOVEHICLE_LIBRARY_RELEASE)
  MESSAGE(STATUS "ChronoVehicle library (release) found")
ENDIF()

IF(NOT CHRONOVEHICLE_LIBRARY_DEBUG AND NOT CHRONOVEHICLE_LIBRARY_RELEASE)
  MESSAGE(FATAL_ERROR "Cannot find the ChronoVehicle library. Check CH_VEHICLE_LIBDIR_DEBUG and CH_VEHICLE_LIBDIR_RELEASE.")
  RETURN()
ENDIF()

MESSAGE(STATUS "ChronoVehicle includes:  ${CHRONOVEHICLE_INCLUDES}")
MESSAGE(STATUS "ChronoVehicle libraries: ${CHRONOVEHICLE_LIBRARIES}")

#--------------------------------------------------------------
# Generate the configuration file 

SET(CHRONOVEHICLE_DATA_DIR "${CH_VEHICLE_SDKDIR}/data/")

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
  ${CMAKE_BINARY_DIR}/demos_vehicle/config.h
  )

MESSAGE(STATUS "PRJ bin dir: ${PROJECT_BINARY_DIR}")
MESSAGE(STATUS "CMK bin dir: ${CMAKE_BINARY_DIR}")


#--------------------------------------------------------------
# Set the include paths and libraries

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    "${CMAKE_BINARY_DIR}/demos_vehicle"
    ${CHRONOVEHICLE_INCLUDES}
)

SET(LIBRARIES 
	${LIBRARIES}
    ${CHRONOVEHICLE_LIBRARIES}
)

IF (ENABLE_OPENGL)
    INCLUDE_DIRECTORIES(
        ${GLM_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIR}
    )

    SET(LIBRARIES ${LIBRARIES} ChronoEngine_OpenGL)
ENDIF()

# ------------------------------------------------------------------------------
# Automatically copy DLLs 
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # HACK?
    IF(CH_VEHICLE_LIBDIR_DEBUG)
        SET(CH_VEHICLE_BINDIR "${CH_VEHICLE_LIBDIR_DEBUG}/../../bin")
    ENDIF()
    IF(CH_VEHICLE_LIBDIR_RELEASE)
        SET(CH_VEHICLE_BINDIR "${CH_VEHICLE_LIBDIR_RELEASE}/../../bin")
    ENDIF()

    # Create custom target for copying DLLs; add it to the default build target
    ADD_CUSTOM_TARGET(COPY_VEHICLE_DLLS ALL)

    # Create custom command, invoked post-build to copy DLLs to the appropriate
    # directory (depending on the configuration selected at build time in VS)
    ADD_CUSTOM_COMMAND(
        TARGET COPY_VEHICLE_DLLS POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CH_VEHICLE_BINDIR}/$<CONFIGURATION>/ChronoVehicle.dll"
            "${CMAKE_BINARY_DIR}/bin/$<CONFIGURATION>"
    )
ENDIF()

#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------
# List of all executables

SET(DEMOS
    hmmwv
)

MESSAGE(STATUS "Vehicle demo programs...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${CH_BUILDFLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH(PROGRAM)

