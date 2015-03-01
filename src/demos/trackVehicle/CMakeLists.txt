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

ADD_SUBDIRECTORY(utils)

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
	subsys/trackVehicle/DriveChain.h
	subsys/trackVehicle/DriveChain.cpp
	subsys/trackVehicle/LoopChain.h
	subsys/trackVehicle/LoopChain.cpp
	subsys/trackVehicle/TrackSoilBin.h
	subsys/trackVehicle/TrackSoilBin.cpp
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
	subsys/driver/Track_FuncDriver.cpp
)

SET(CT_DRIVEGEAR_FILES
	subsys/driveGear/DriveGear.h
	subsys/driveGear/DriveGear.cpp
)

SET(CT_CHAIN_FILES
	subsys/trackChain/TrackChain.h
	subsys/trackChain/TrackChain.cpp
)

SET(CT_TRACKSYSTEM_FILES
	subsys/trackSystem/TrackSystem.h
	subsys/trackSystem/TrackSystem.cpp
)

SET(CH_POWERTRAIN_FILES
	subsys/powertrain/TrackPowertrain.h
	subsys/powertrain/TrackPowertrain.cpp
)

SET(CH_DRIVELINE_FILES
	subsys/driveline/TrackDriveline.h
	subsys/driveline/TrackDriveline.cpp
)


SOURCE_GROUP("base" FILES ${CT_BASE_FILES})
SOURCE_GROUP("vehicle" FILES ${CT_VEHICLE_FILES})
SOURCE_GROUP("suspension" FILES ${CT_SUSPENSION_FILES})
SOURCE_GROUP("idler" FILES ${CT_IDLER_FILES})
SOURCE_GROUP("driver" FILES ${CT_DRIVER_FILES})
SOURCE_GROUP("driveGear" FILES ${CT_DRIVEGEAR_FILES})
SOURCE_GROUP("chain" FILES ${CT_CHAIN_FILES})
SOURCE_GROUP("trackSystem" FILES ${CT_TRACKSYSTEM_FILES})
SOURCE_GROUP("powertrain" FILES ${CH_POWERTRAIN_FILES})
SOURCE_GROUP("driveline" FILES ${CH_DRIVELINE_FILES})

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
			${CH_DRIVELINE_FILES}
			)
			
SET_TARGET_PROPERTIES(TrackVehicle PROPERTIES
	COMPILE_FLAGS "${CH_BUILDFLAGS}"
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
# DEMO 1: M113 model created by Justin, from report & data from Shabana and Wallin
ADD_EXECUTABLE(demo_track_M113  demo_track_M113.cpp)

SOURCE_GROUP(""  FILES  demo_track_M113.cpp)

SET_TARGET_PROPERTIES(demo_track_M113  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(demo_track_M113 ${CT_TARGET_LIBS} TrackVehicle)

INSTALL(TARGETS demo_track_M113 DESTINATION bin)

# ---------------------------------------------
# DEMO 2: test the kinematics of the new constraint
ADD_EXECUTABLE(test_idlerJoint  test_idlerJoint.cpp)

SOURCE_GROUP(""  FILES  test_idlerJoint.cpp)

SET_TARGET_PROPERTIES(test_idlerJoint  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(test_idlerJoint ${CT_TARGET_LIBS} TrackVehicle)

INSTALL(TARGETS test_idlerJoint DESTINATION bin)

# ---------------------------------------------
# DEMO 3: DriveChain test
ADD_EXECUTABLE(test_driveChain  test_driveChain.cpp)

SOURCE_GROUP(""  FILES  test_driveChain.cpp)

SET_TARGET_PROPERTIES(test_driveChain  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(test_driveChain ${CT_TARGET_LIBS} TrackVehicle)

INSTALL(TARGETS test_driveChain DESTINATION bin)