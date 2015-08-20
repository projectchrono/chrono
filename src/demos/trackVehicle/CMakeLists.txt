#=============================================================================
# CMake configuration file for Tracked Vehicle modeling library
#=============================================================================
SET(ENABLE_TRACK_VEHICLE
  FALSE  CACHE  BOOL
  "Turn this ON to generate the Tracked Vehicle modeling unit."
)

IF(NOT ENABLE_TRACK_VEHICLE)
  RETURN()
ELSE()
  MESSAGE(STATUS "...enabling Tracked Vehicle library and models. Also creates Chrono_Utils library")
ENDIF()

# require irrlicht
IF(NOT ENABLE_UNIT_IRRLICHT)
  MESSAGE(WARNING "need irrlicht for GUI input")
  RETURN()
ENDIF()


cmake_minimum_required(VERSION 2.8)

SET(CT_BASE_FILES
	subsys/ChApiSubsys.h
	subsys/base/ChTrackVehicle.h
	subsys/base/ChTrackVehicle.cpp
	subsys/base/ChDriverTrack.h
	subsys/base/ChDriverTrack.cpp
)

SET(CT_VEHICLE_FILES
	subsys/trackVehicle/TrackVehicle.h
	subsys/trackVehicle/TrackVehicle.cpp
	subsys/trackVehicle/TrackVehicleM113.h
	subsys/trackVehicle/TrackVehicleM113.cpp
	subsys/trackVehicle/DriveChain.h
	subsys/trackVehicle/DriveChain.cpp
)

SET(CT_SUSPENSION_FILES
	subsys/suspension/TorsionArmSuspension.h
	subsys/suspension/TorsionArmSuspension.cpp
)

SET(CT_IDLER_FILES
	subsys/idler/IdlerSimple.h
	subsys/idler/IdlerSimple.cpp
	subsys/idler/SupportRoller.h
	subsys/idler/SupportRoller.cpp
)

SET(CT_DRIVER_FILES
	subsys/driver/ChIrrGuiTrack.h
	subsys/driver/ChIrrGuiTrack.cpp
	subsys/driver/Track_FuncDriver.h
)

SET(CT_DRIVEGEAR_FILES
	subsys/driveGear/DriveGear.h
	subsys/driveGear/DriveGear.cpp
	subsys/driveGear/DriveGearMotion.h
	subsys/driveGear/DriveGearMotion.cpp
)

SET(CT_CHAIN_FILES
	subsys/trackChain/TrackChain.h
	subsys/trackChain/TrackChain.cpp
)

SET(CT_TRACKSYSTEM_FILES
	subsys/trackSystem/TrackSystem.h
	subsys/trackSystem/TrackSystem.cpp
	subsys/trackSystem/TrackSystemM113.h
	subsys/trackSystem/TrackSystemM113.cpp
)

SET(CH_POWERTRAIN_FILES
	subsys/powertrain/TrackPowertrain.h
	subsys/powertrain/TrackPowertrain.cpp
)

SET(CH_DRIVELINE_FILES
	subsys/driveline/TrackDriveline.h
	subsys/driveline/TrackDriveline.cpp
)

SET(CH_COLLISION_FILES
	subsys/collision/TrackCollisionCallback.h
	subsys/collision/CollisionReporters.h)

SOURCE_GROUP("base" FILES ${CT_BASE_FILES})
SOURCE_GROUP("vehicle" FILES ${CT_VEHICLE_FILES})
SOURCE_GROUP("suspension" FILES ${CT_SUSPENSION_FILES})
SOURCE_GROUP("idler" FILES ${CT_IDLER_FILES})
SOURCE_GROUP("driver" FILES ${CT_DRIVER_FILES})
SOURCE_GROUP("driveGear" FILES ${CT_DRIVEGEAR_FILES})
SOURCE_GROUP("chain" FILES ${CT_CHAIN_FILES})
SOURCE_GROUP("trackSystem" FILES ${CT_TRACKSYSTEM_FILES})
SOURCE_GROUP("powertrain" FILES ${CH_POWERTRAIN_FILES})
# SOURCE_GROUP("driveline" FILES ${CH_DRIVELINE_FILES})
SOURCE_GROUP("collision" FILES ${CH_COLLISION_FILES})

# ------------------------------------------------------------------------------
# Add the ChronoTrackVehicle library
ADD_LIBRARY(TrackVehicle SHARED 
			${CT_BASE_FILES}
			${CT_VEHICLE_FILES}
			${CT_SUSPENSION_FILES}
			${CT_IDLER_FILES}
			${CT_DRIVER_FILES}
			${CT_DRIVEGEAR_FILES}
			${CT_CHAIN_FILES}
			${CT_TRACKSYSTEM_FILES}
			${CH_POWERTRAIN_FILES}
			# ${CH_DRIVELINE_FILES}
			${CH_COLLISION_FILES}
			)
			
SET_TARGET_PROPERTIES(TrackVehicle PROPERTIES
	COMPILE_FLAGS "${CH_CXX_FLAGS}"
	LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
	COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_TRACKS")

# two extra target libraries if using Irrlicht, plus include dirs
IF(ENABLE_UNIT_IRRLICHT)	
	SET(CT_TARGET_LIBS
		Chrono_Utils
		ChronoEngine
		ChronoEngine_IRRLICHT
		${CH_IRRLICHTLIB}
	)
	INCLUDE_DIRECTORIES(${CH_IRRLICHTINC} )
ELSE()
	SET(CT_TARGET_LIBS
		Chrono_Utils
		ChronoEngine
	)
ENDIF()

	
TARGET_LINK_LIBRARIES(TrackVehicle ${CT_TARGET_LIBS})


	
# ADD_DEPENDENCIES (TrackVehicle ChronoEngine Chrono_Utils ChronoEngine_IRRLICHT)  #  not automatic
		
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/demos/trackVehicle/)

INSTALL(TARGETS TrackVehicle
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

# INSTALL(FILES ${} DESTINATION include/demos/trackVehicle)
	

# ------------------------------------------------------------------------------
# Copy the track_data folder to the bin directory
IF(MSVC)
  FILE( COPY ${CMAKE_SOURCE_DIR}/demos/trackVehicle/track_data DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSEIF(XCODE_VERSION)
  FILE( COPY ${CMAKE_SOURCE_DIR}/demos/trackVehicle/track_data DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSE()
  FILE( COPY ${CMAKE_SOURCE_DIR}/demos/trackVehicle/track_data DESTINATION ${CMAKE_BINARY_DIR}/data/)
ENDIF()

# add demo models here
# ---------------------------------------------
# DEMO 1: track vehicle model created by Justin, M113 vehicle from Wallin, plus a powertrain
# ---------------------------------------------
# DEMO 2: test the kinematics of the new constraint
# ---------------------------------------------
# DEMO 3: DriveChain test
# ---------------------------------------------
# DEMO 4: track vehicle model created by Justin, M113 vehicle from Wallin, driven by a motion on the gear sprockets.
SET(TRACK_DEMOS
demo_trackVehicle
test_idlerJoint
test_driveChain
test_M113)


FOREACH(PROGRAM ${TRACK_DEMOS})

	ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
	SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")
	SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
		FOLDER demos
		COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
		LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)
	TARGET_LINK_LIBRARIES(${PROGRAM}
		${CT_TARGET_LIBS}
		TrackVehicle
		)
	ADD_DEPENDENCIES(${PROGRAM} ${CT_TARGET_LIBS} TrackVehicle)
	INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH()

