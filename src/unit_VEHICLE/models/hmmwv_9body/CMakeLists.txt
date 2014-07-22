# HMMWV vehicle model, 9 bodies, simplified suspension dynamic setup
# Works with and without Irrlicht run-time visualization
# will always be able to output PovRay data files for post-process animations
cmake_minimum_required(VERSION 2.8)

OPTION(ENABLE_HMMWV_9BODY    "Build the HMMWV vehicle model"     OFF)
MESSAGE("Building the 9-body HMMWV? : ${ENABLE_HMMWV_9BODY}")

# ----------------------
# Configuration options
# ----------------------
INCLUDE(CmakeDependentOption)
OPTION(USE_IRRLICHT "Build demos with Irrlicht support" ON)
MESSAGE("Using Irrlicht? : ${USE_IRRLICHT}")

# ----------------------

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



IF(USE_IRRLICHT)
	# allow user-control via a GUI thru the event receiver
	# No testing mechanism; rather, a class that controls the terrain features
	SET(demo_HMMWV_HEADERS ${demo_HMMWV_HEADERS}
			HMMWVEventReceiver.h
		)

	# include the irrlicht library as a dependency
	SET(HMMWV_LIBS ${HMMWV_LIBS} ${CH_IRRLICHTLIB})
	SET(HMMWV_DEPENDENCIES ${HMMWV_DEPENDENCIES} Irrlicht)
ENDIF()

IF(ENABLE_HMMWV_9BODY)
	ADD_EXECUTABLE(demo_HMMWV_9body ${demo_HMMWV_SRCS} ${demo_HMMWV_HEADERS})
	SET_TARGET_PROPERTIES(demo_HMMWV_9body PROPERTIES LINK_FLAGS "${LINKERFLAG_EXE}")
	TARGET_LINK_LIBRARIES(demo_HMMWV_9body
		${HMMWV_LIBS}
		)
	ADD_DEPENDENCIES(demo_HMMWV_9body ${HMMWV_DEPENDENCIES})
	INSTALL(TARGETS demo_HMMWV_9body DESTINATION bin)
ENDIF()
