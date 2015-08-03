#=============================================================================
# CMake configuration file for Chrono-Vehicle
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_UNIT_VEHICLE "Enable the Chrono Vehicle module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_UNIT_VEHICLE)
  RETURN()
ENDIF()

# If Irrlicht support was enabled, provide option to add Irrklang support.
cmake_dependent_option(ENABLE_IRRKLANG "Enable Irrklang library for sound" OFF
                       "ENABLE_UNIT_IRRLICHT" OFF)



# ----------------------------------------------------------------------------
# Find IrrKlang library
# ----------------------------------------------------------------------------
IF(ENABLE_IRRKLANG)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      SET(CH_IRRKLANG_SDKDIR ""  CACHE PATH   "Where is your IrrKlang SDK installed?")
	  SET (CH_IRRKLANGLIB   "" CACHE FILEPATH "Where is your Irrklank library?")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      FIND_PATH(CH_IRRKLANG_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrklang" "/usr/local/include/irrklang")
      FIND_LIBRARY(CH_IRRKLANGLIB NAMES Irrklang PATHS "/usr/local/lib" ${CH_IRRKLANG_SDKDIR}/lib/Linux)
  ENDIF()

  IF(EXISTS "${CH_IRRKLANG_SDKDIR}/include")
      SET(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}/include")
  ELSE()
      SET(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}")
  ENDIF()

  INCLUDE_DIRECTORIES(${CH_IRRKLANGINC})

  MARK_AS_ADVANCED(CLEAR CH_IRRKLANG_SDKDIR)
  MARK_AS_ADVANCED(CLEAR CH_IRRKLANGLIB)

ELSE()

  MARK_AS_ADVANCED(FORCE CH_IRRKLANG_SDKDIR)
  MARK_AS_ADVANCED(FORCE CH_IRRKLANGLIB)

ENDIF()

# ----------------------------------------------------------------------------
# Add paths to the top of the source directory and the binary directory
# ----------------------------------------------------------------------------

SET(CH_VEHICLE_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
)

INCLUDE_DIRECTORIES(${CH_VEHICLE_INCLUDES})


# ----------------------------------------------------------------------------
# Create configuration file
# ----------------------------------------------------------------------------

# Prepare replacement variables

IF(ENABLE_IRRLICHT)
  SET(IRRLICHT_ENABLED "1")
ELSE()
  SET(IRRLICHT_ENABLED "0")
ENDIF()

IF(ENABLE_IRRKLANG)
  SET(IRRKLANG_ENABLED "1")
ELSE()
  SET(IRRKLANG_ENABLED "0")
ENDIF()

SET(CHRONO_DATA_DIR "${CH_CHRONO_SDKDIR}/demos/data/")

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/cmake/ChConfigVehicle.in
  ${PROJECT_BINARY_DIR}/chrono_vehicle/ChConfigVehicle.h
  )

# ----------------------------------------------------------------------------
# Add subdirectories
# ----------------------------------------------------------------------------

ADD_SUBDIRECTORY(chrono_vehicle)
ADD_SUBDIRECTORY(chrono_utils)
#ADD_SUBDIRECTORY(tests)
