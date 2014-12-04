#=============================================================================
# CMake configuration file for Tracked Vehicle modeling library
#=============================================================================

SET(ENABLE_UNIT_TRACKED
  FALSE  CACHE  BOOL
  "Turn this ON to generate the Tracked Vehicle modeling unit."
)

IF(NOT ENABLE_UNIT_TRACKED)
  RETURN()
ELSE()
  MESSAGE(STATUS "...enabling Tracked Vehicle Unit")
ENDIF()

cmake_minimum_required(VERSION 2.8)

SET(ChronoEngine_unit_TrackedVehicle_SOURCES
	subsys/driveGear/DriveGear.cpp
	subsys/driver/ChDataDriver.cpp
	subsys/idler/IdlerSimple.cpp
	subsys/suspension/TorsionArmSuspension.cpp
	subsys/trackChain/TrackChain.cpp
	subsys/trackSystem/TrackSystem.cpp
	subsys/trackVehicle/TrackVehicle.cpp
)

SET(ChronoEngine_unit_TrackedVehicle_HEADERS
	subsys/ChApiSubsys.h
	subsys/driveGear/DriveGear.h
	subsys/driver/ChDataDriver.h
	subsys/idler/IdlerSimple.h
	subsys/suspension/TorsionArmSuspension.h
	subsys/trackChain/TrackChain.h
	subsys/trackSystem/TrackSystem.h
	subsys/trackVehicle/TrackVehicle.h
)

SOURCE_GROUP(unit_TrackedVehicle FILES
	${ChronoEngine_unit_TrackedVehicle_SOURCES}
	${ChronoEngine_unit_TrackedVehicle_HEADERS})
	
	
# build the library, with custom properties
ADD_LIBRARY(ChronoEngine_Tracks SHARED 
			${ChronoEngine_unit_TrackedVehicle_SOURCES}
			${ChronoEngine_unit_TrackedVehicle_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_Tracks PROPERTIES
                           COMPILE_FLAGS "${CH_BUILDFLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                          COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_TRACKS")
                          
TARGET_LINK_LIBRARIES(ChronoEngine_Tracks 
	ChronoEngine
)
	
ADD_DEPENDENCIES (ChronoEngine_Tracks ChronoEngine)  #  not automatic
	
	
# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
#SET (foo      		"${foo}" 			 PARENT_SCOPE )
	
INSTALL(TARGETS ChronoEngine_Tracks
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)

INSTALL(FILES ${ChronoEngine_unit_TrackedVehicle_HEADERS} DESTINATION include/chrono/unit_TrackedVehicle)
	
	
	
	
# ------------------------------------------------------------------------------
# Copy the track_data folder to the bin directory
IF(MSVC)
  FILE( COPY ${CMAKE_SOURCE_DIR}/tracked_vehicles/track_data DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSEIF(XCODE_VERSION)
  FILE( COPY ${CMAKE_SOURCE_DIR}/tracked_vehicles/track_data DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
ELSE()
  FILE( COPY ${CMAKE_SOURCE_DIR}/tracked_vehicles/track_data DESTINATION ${CMAKE_BINARY_DIR}/data/)
ENDIF()

