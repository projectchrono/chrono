#=============================================================================
# CMake configuration file for Tracked Vehicle modeling library
#=============================================================================
SET(ENABLE_TRACK_VEHICLE
  FALSE  CACHE  BOOL
  "Turn this ON to generate the Tracked Vehicle modeling unit."
)

# require irrlicht
IF(NOT ENABLE_UNIT_IRRLICHT)
  MESSAGE(WARNING "need irrlicht for GUI input")
  RETURN()
ENDIF()

IF(NOT ENABLE_TRACK_VEHICLE)
  RETURN()
ELSE()
  MESSAGE(STATUS "...enabling Tracked Vehicle library and models. Also creates Chrono_Utils library")
ENDIF()


ADD_SUBDIRECTORY(utils)

cmake_minimum_required(VERSION 2.8)

SET(ChronoEngine_unit_TrackVehicle_SOURCES
	subsys/driveGear/DriveGear.cpp
	subsys/driveline/TrackDriveline.cpp
	subsys/driveline/TrackDriveline_1WD.cpp
	subsys/driver/ChDriverTrack.cpp
	subsys/driver/ChIrrGuiTrack.cpp
	subsys/driver/Track_FuncDriver.cpp
	subsys/idler/IdlerSimple.cpp
	subsys/powertrain/TrackPowerTrain.cpp
	subsys/suspension/TorsionArmSuspension.cpp
	subsys/trackChain/TrackChain.cpp
	subsys/trackSystem/TrackSystem.cpp
	subsys/trackVehicle/TrackVehicle.cpp
	subsys/trackVehicle/DriveChain.cpp
)

SET(ChronoEngine_unit_TrackVehicle_HEADERS
	subsys/ChApiSubsys.h
	subsys/driveGear/DriveGear.h
	subsys/driveline/TrackDriveline.h
	subsys/driveline/TrackDriveline_1WD.h
	subsys/driver/ChDriverTrack.h
	subsys/driver/ChIrrGuiTrack.h
	subsys/driver/Track_FuncDriver.h
	subsys/idler/IdlerSimple.h
	subsys/powertrain/TrackPowerTrain.h
	subsys/suspension/TorsionArmSuspension.h
	subsys/trackChain/TrackChain.h
	subsys/trackSystem/TrackSystem.h
	subsys/trackVehicle/TrackVehicle.h
	subsys/trackVehicle/DriveChain.h
)


SOURCE_GROUP(trackVehicle FILES
	${ChronoEngine_unit_TrackVehicle_SOURCES}
	${ChronoEngine_unit_TrackVehicle_HEADERS})
	
	
# build the library, with custom properties
ADD_LIBRARY(TrackVehicle SHARED 
			${ChronoEngine_unit_TrackVehicle_SOURCES}
			${ChronoEngine_unit_TrackVehicle_HEADERS})

SET_TARGET_PROPERTIES(TrackVehicle PROPERTIES
						COMPILE_FLAGS "${CH_BUILDFLAGS}"
						LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
						COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_TRACKS")
                          
TARGET_LINK_LIBRARIES(TrackVehicle 
	Chrono_Utils
	ChronoEngine
	ChronoEngine_IRRLICHT
    ${CH_IRRLICHTLIB}
)
	
ADD_DEPENDENCIES (TrackVehicle ChronoEngine Chrono_Utils ChronoEngine_IRRLICHT)  #  not automatic
		
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/demos/trackVehicle/
					${CH_IRRLICHTINC} )
	
INSTALL(TARGETS TrackVehicle
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_unit_TrackVehicle_HEADERS} DESTINATION include/demos/trackVehicle)
	
	
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

TARGET_LINK_LIBRARIES(demo_track_M113
	ChronoEngine
	ChronoEngine_IRRLICHT
	TrackVehicle
	Chrono_Utils
    ${CH_IRRLICHTLIB}
)

# ADD_DEPENDENCIES (demo_track_M113 ChronoEngine ChronoEngine_IRRLICHT TrackVehicle)

INSTALL(TARGETS demo_track_M113 DESTINATION bin)
MESSAGE("Added DEMO 1: M113 track vehicle, user driven with GUI")

# ---------------------------------------------
# DEMO 2: test the kinematics of the new constraint
ADD_EXECUTABLE(test_idlerJoint  test_idlerJoint.cpp)

SOURCE_GROUP(""  FILES  test_idlerJoint.cpp)

SET_TARGET_PROPERTIES(test_idlerJoint  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(test_idlerJoint
	ChronoEngine
	ChronoEngine_IRRLICHT
	TrackVehicle
	Chrono_Utils
    ${CH_IRRLICHTLIB}
)

# ADD_DEPENDENCIES (test_idlerJoint ChronoEngine ChronoEngine_IRRLICHT TrackVehicle)

INSTALL(TARGETS test_idlerJoint DESTINATION bin)
MESSAGE("Added DEMO 2: test idlerJoint motion against conventional 2 body/2 constraint system")

# ---------------------------------------------
# DEMO 3: DriveChain test
ADD_EXECUTABLE(test_driveChain  test_driveChain.cpp)

SOURCE_GROUP(""  FILES  test_driveChain.cpp)

SET_TARGET_PROPERTIES(test_driveChain  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(test_driveChain
	ChronoEngine
	ChronoEngine_IRRLICHT
	TrackVehicle
	Chrono_Utils
    ${CH_IRRLICHTLIB}
)

# ADD_DEPENDENCIES (test_driveChain ChronoEngine ChronoEngine_IRRLICHT TrackVehicle)

INSTALL(TARGETS test_driveChain DESTINATION bin)
MESSAGE("Added DEMO 3: test a driveChain system. Power to a gear is transferred to an idler wheel via rigid-body chain")