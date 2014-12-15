#=============================================================================
# CMake configuration file for Tracked Vehicle modeling library
#=============================================================================
SET(ENABLE_TRACKED_VEHICLE
  FALSE  CACHE  BOOL
  "Turn this ON to generate the Tracked Vehicle modeling unit."
)

# require irrlicht
IF(NOT ENABLE_UNIT_IRRLICHT)
  MESSAGE(STATUS "need irrlicht for GUI input")
  RETURN()
ENDIF()

IF(NOT ENABLE_TRACKED_VEHICLE)
  RETURN()
ELSE()
  MESSAGE(STATUS "...enabling Tracked Vehicle library and models")
ENDIF()


cmake_minimum_required(VERSION 2.8)

SET(ChronoEngine_unit_TrackedVehicle_SOURCES
	subsys/driveGear/DriveGear.cpp
	subsys/driver/ChDriverTrack.cpp
	subsys/idler/IdlerSimple.cpp
	subsys/suspension/TorsionArmSuspension.cpp
	subsys/trackChain/TrackChain.cpp
	subsys/trackSystem/TrackSystem.cpp
	subsys/trackVehicle/TrackVehicle.cpp
)

SET(ChronoEngine_unit_TrackedVehicle_HEADERS
	subsys/ChApiSubsys.h
	subsys/driveGear/DriveGear.h
	subsys/driver/ChDriverTrack.h
	subsys/idler/IdlerSimple.h
	subsys/suspension/TorsionArmSuspension.h
	subsys/trackChain/TrackChain.h
	subsys/trackSystem/TrackSystem.h
	subsys/trackVehicle/TrackVehicle.h
)

SOURCE_GROUP(trackedVehicle FILES
	${ChronoEngine_unit_TrackedVehicle_SOURCES}
	${ChronoEngine_unit_TrackedVehicle_HEADERS})
	
	
# build the library, with custom properties
ADD_LIBRARY(TrackedVehicle SHARED 
			${ChronoEngine_unit_TrackedVehicle_SOURCES}
			${ChronoEngine_unit_TrackedVehicle_HEADERS})

SET_TARGET_PROPERTIES(TrackedVehicle PROPERTIES
                           COMPILE_FLAGS "${CH_BUILDFLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_TRACKS")
                          
TARGET_LINK_LIBRARIES(TrackedVehicle 
	ChronoEngine
)
	
ADD_DEPENDENCIES (TrackedVehicle ChronoEngine)  #  not automatic
		
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/demos/trackedVehicle 
					${CH_IRRLICHTINC} )
	
INSTALL(TARGETS TrackedVehicle
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_unit_TrackedVehicle_HEADERS} DESTINATION include/demos/trackedVehicle)
	
	
# ------------------------------------------------------------------------------
# Copy the track_data folder to the bin directory
IF(MSVC)
  FILE( COPY ${CMAKE_SOURCE_DIR}/demos/trackedVehicle/track_data DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSEIF(XCODE_VERSION)
  FILE( COPY ${CMAKE_SOURCE_DIR}/demos/trackedVehicle/track_data DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSE()
  FILE( COPY ${CMAKE_SOURCE_DIR}/demos/trackedVehicle/track_data DESTINATION ${CMAKE_BINARY_DIR}/data/)
ENDIF()

# add demo models here
ADD_EXECUTABLE(demo_tracked_M113  demo_tracked_M113.cpp)

SOURCE_GROUP(""  FILES  demo_tracked_M113.cpp)

SET_TARGET_PROPERTIES(demo_tracked_M113  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS_IRR}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
)

TARGET_LINK_LIBRARIES(demo_tracked_M113
	ChronoEngine
	ChronoEngine_IRRLICHT
	TrackedVehicle
    ${CH_IRRLICHTLIB}
)

# ADD_DEPENDENCIES (demo_tracked_M113 ChronoEngine ChronoEngine_IRRLICHT TrackedVehicle)

INSTALL(TARGETS demo_tracked_M113 DESTINATION bin)
