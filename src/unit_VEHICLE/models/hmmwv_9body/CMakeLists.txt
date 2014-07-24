# HMMWV vehicle model, 9 bodies, simplified suspension dynamic setup
# Works with and without Irrlicht run-time visualization
# will always be able to output PovRay data files for post-process animations

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CmakeDependentOption)

OPTION(HMMWV_9BODY_ENABLE "Build the HMMWV vehicle model" OFF)

# If building this model and if Irrlicht support is enabled in Chrono,
# give the option of using Irrlicht for the HMMWV_9BODY model.
CMAKE_DEPENDENT_OPTION(
	HMMWV_9BODY_USE_IRRLICHT "Use Irrlicht for the HMMWV_9BODY model" OFF
	"HMMWV_9BODY_ENABLE;ENABLE_IRRLICHT" OFF
	)

# Give option to use terrain
CMAKE_DEPENDENT_OPTION(
	HMMWV_9BODY_USE_TERRAIN "Use terrain" OFF
	"HMMWV_9BODY_ENABLE" OFF
	)

IF(NOT HMMWV_9BODY_ENABLE)
	RETURN()
ENDIF()

# ----------------------

MESSAGE(STATUS "Adding HMMWV_9BODY...")

SET(demo_HMMWV_HEADERS
		HMMWV_9body.h
		HMMWVTerrain.h
		SoilbinWheel.h
		DoubleAarm.h
		ChVehicleDriver.h
		)

SET(demo_HMMWV_SRCS
	demo_HMMWV_9body.cpp
	HMMWV_9body.cpp
	DoubleAarm.cpp
)

SET(HMMWV_LIBS 
	${CHRONO_LIB}
	Chrono_Utils
	)

SET(HMMWV_DEPENDENCIES
	${CHRONO_LIB}
	Chrono_Utils
	)


IF(HMMWV_9BODY_USE_IRRLICHT)
	# allow user-control via a GUI thru the event receiver
	# No testing mechanism; rather, a class that controls the terrain features
	SET(demo_HMMWV_HEADERS ${demo_HMMWV_HEADERS}
			HMMWVEventReceiver.h
		)

	# include the irrlicht library as a dependency
	SET(HMMWV_LIBS ${HMMWV_LIBS} ${CH_IRRLICHTLIB})
	SET(HMMWV_DEPENDENCIES ${HMMWV_DEPENDENCIES})
ENDIF()

# Create the executable
ADD_EXECUTABLE(demo_HMMWV_9body ${demo_HMMWV_SRCS} ${demo_HMMWV_HEADERS})
SET_TARGET_PROPERTIES(demo_HMMWV_9body PROPERTIES LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_HMMWV_9body ${HMMWV_LIBS})
ADD_DEPENDENCIES(demo_HMMWV_9body ${HMMWV_DEPENDENCIES})
INSTALL(TARGETS demo_HMMWV_9body DESTINATION bin)

message("libraries: ${HMMWV_LIBS}")
message("dependencies: ${HMMWV_DEPENDENCIES}")

# ----------------------
# Create config file
# ----------------------

# Prepare replacement variables

IF(HMMWV_9BODY_USE_IRRLICHT)
	SET(DEFINE_USE_IRRLICHT "#define USE_IRRLICHT")
ELSE()
	SET(DEFINE_USE_IRRLICHT "")
ENDIF()

IF(HMMWV_9BODY_USE_TERRAIN)
	SET(DEFINE_USE_TERRAIN "#define USE_TERRAIN")
ELSE()
	SET(DEFINE_USE_TERRAIN "")
ENDIF()

# Generate the configuration header file using substitution variables.
# Place the header file in the library output directory and make sure it can
# be found at compile time.

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/HMMWV_9body_config.in
  ${PROJECT_BINARY_DIR}/HMMWV_9body_config.h
  )

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
