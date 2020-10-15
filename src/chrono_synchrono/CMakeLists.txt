#=============================================================================
# CMake configuration file for SynChrono module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_SYNCHRONO "Enable the SynChrono module" OFF)

IF(NOT ENABLE_MODULE_SYNCHRONO)
  return()
ENDIF()

message(STATUS "==== SynChrono module ====")

set(SYN_INCLUDES ${SYN_INCLUDES})

set(SYN_CXX_FLAGS "${CH_CXX_FLAGS}")
set(SYN_LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")
set(SYN_LIBRARIES "")
set(SYN_LIB_NAMES "ChronoEngine")

#-----------------------------------------------------------------------------
# Find FlatBuffers
#-----------------------------------------------------------------------------

message(STATUS "Searching for FlatBuffers...")

set(Flatbuffers_DIR "../chrono_thirdparty/flatbuffers/CMake" CACHE STRING "")

find_package(Flatbuffers REQUIRED)

find_path(Flatbuffers_INCLUDE_DIR flatbuffers/flatbuffers.h)

message(STATUS "FlatBuffer libraries: ${FLATBUFFER_LIBRARY}")
message(STATUS "FlatBuffer includes: ${Flatbuffers_INCLUDE_DIR}")

set(SYN_INCLUDES
    ${SYN_INCLUDES}
    ${Flatbuffers_INCLUDE_DIR}
)

#-----------------------------------------------------------------------------
# Find MPI (Default transport method)
#-----------------------------------------------------------------------------

find_package(MPI REQUIRED)
message(STATUS "MPI libraries: ${MPI_LIBRARIES}")

set(SYN_INCLUDES
    ${SYN_INCLUDES}
    ${MPI_C_HEADER_DIR}
    ${MPI_CXX_HEADER_DIR}
)

set(SYN_LIBRARIES
	${SYN_LIBRARIES}
	${MPI_LIBRARIES}
)

#-----------------------------------------------------------------------------
# Find DDS (Optional)
#-----------------------------------------------------------------------------
option(USE_FAST_DDS "Enable the FastDDS interface for the SynChrono Module" OFF)

if (USE_FAST_DDS)
	set(FAST_DDS_INSTALL_DIR "" CACHE PATH "Path to FastDDS install dir")
	set(fastcdr_DIR "${FAST_DDS_INSTALL_DIR}/lib/cmake/fastcdr" CACHE FILEPATH "Path to the FastCDR cmake config file" FORCE)
	set(fastrtps_DIR "${FAST_DDS_INSTALL_DIR}/share/fastrtps/cmake" CACHE FILEPATH "Path to the FastDDS cmake config file" FORCE)
	set(foonathan_memory_DIR "${FAST_DDS_INSTALL_DIR}/lib/foonathan_memory/cmake" CACHE FILEPATH "Path to the foonathan memory cmake config file" FORCE)

	find_package(fastcdr REQUIRED)
	find_package(fastrtps REQUIRED)

	set(SYN_OTHER_LIBRARIES
		fastrtps
		fastcdr
	)

endif()

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Generate the configuration header file using substitution variables.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SynConfig.h.in
${PROJECT_BINARY_DIR}/chrono_synchrono/SynConfig.h)

install(FILES "${PROJECT_BINARY_DIR}/chrono_synchrono/SynConfig.h"
DESTINATION include/chrono_synchrono)

# ------------
# Common Files
# ------------

set(SYN_BASE_FILES
	SynApi.h
)
source_group("base" FILES ${SYN_BASE_FILES})

set(SYN_AGENT_FILES
	agent/SynAgent.h
	agent/SynAgent.cpp

	agent/SynAgentFactory.h
	agent/SynAgentFactory.cpp

	agent/SynVehicleAgent.h
	agent/SynVehicleAgent.cpp
	agent/SynWheeledVehicleAgent.h
	agent/SynWheeledVehicleAgent.cpp
	agent/SynTrackedVehicleAgent.h
	agent/SynTrackedVehicleAgent.cpp
    
    agent/SynEnvironmentAgent.h
    agent/SynEnvironmentAgent.cpp
)
if(USE_FAST_DDS)
	list(APPEND SYN_AGENT_FILES
		agent/SynDDSAgent.h
		agent/SynDDSAgent.cpp
	)
endif()
source_group("agent" FILES ${SYN_AGENT_FILES})

set(SYN_BRAIN_FILES
	brain/SynBrain.h
	brain/SynVehicleBrain.h
	brain/SynVehicleBrain.cpp
	
	brain/SynACCBrain.h
	brain/SynACCBrain.cpp
	brain/SynQueryEnvBrain.h
	brain/SynQueryEnvBrain.cpp
	brain/SynVehicleBrain.h
	brain/SynVehicleBrain.cpp
	brain/SynCollisionBrain.h
    brain/SynCollisionBrain.cpp
    brain/SynEnvironmentBrain.h

    brain/driver/SynMultipathDriver.cpp
    brain/driver/SynMultipathDriver.h
)
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	# _InteractiveDriver includes a joystick file that only works on Linux
	list(APPEND SYN_BRAIN_FILES
		brain/driver/SynInteractiveDriver.h
		brain/driver/SynInteractiveDriver.cpp
	)
endif()
source_group("brain" FILES ${SYN_BRAIN_FILES})

set(SYN_CLI_FILES
	cli/SynCLI.h
	cli/SynCLI.cpp
)
source_group("cli" FILES ${SYN_CLI_FILES})

set(SYN_COMMUNICATION_FILES
	communication/SynCommunicationManager.h
	communication/SynCommunicationManager.cpp

	communication/mpi/SynMPIManager.h
	communication/mpi/SynMPIManager.cpp
)
if(USE_FAST_DDS)
	list(APPEND SYN_COMMUNICATION_FILES
		communication/dds/SynDDSManager.h
		communication/dds/SynDDSManager.cpp

		communication/dds/SynDDSEntity.h
		communication/dds/SynDDSEntity.cpp
		communication/dds/SynDDSListener.h
		communication/dds/SynDDSListener.cpp
		communication/dds/SynDDSPublisher.h
		communication/dds/SynDDSPublisher.cpp
		communication/dds/SynDDSSubscriber.h
		communication/dds/SynDDSSubscriber.cpp
		communication/dds/SynDDSParticipant.h
		communication/dds/SynDDSParticipant.cpp

		communication/dds/idl/SynDDSMessage.h
		communication/dds/idl/SynDDSMessage.cxx
		communication/dds/idl/SynDDSMessagePubSubTypes.h
		communication/dds/idl/SynDDSMessagePubSubTypes.cxx
	)
endif()
source_group("communication" FILES ${SYN_COMMUNICATION_FILES})

set(SYN_FLATBUFFER_FILES
	flatbuffer/SynFlatBuffersManager.h
	flatbuffer/SynFlatBuffersManager.cpp
	
	flatbuffer/message/SynFlatBuffers_generated.h

	flatbuffer/message/SynMessage.h
	flatbuffer/message/SynMessage.cpp

	flatbuffer/message/SynAgentMessage.h
	flatbuffer/message/SynAgentMessage.cpp
	flatbuffer/message/SynWheeledVehicleMessage.h
	flatbuffer/message/SynWheeledVehicleMessage.cpp
	flatbuffer/message/SynTrackedVehicleMessage.h
	flatbuffer/message/SynTrackedVehicleMessage.cpp
	
	flatbuffer/message/SynSensorMessage.h
	flatbuffer/message/SynSensorMessage.cpp
	flatbuffer/message/SynSCMMessage.h
	flatbuffer/message/SynSCMMessage.cpp
    
    flatbuffer/message/SynApproachMessage.h
    flatbuffer/message/SynApproachMessage.cpp
    flatbuffer/message/SynEnvironmentMessage.h
    flatbuffer/message/SynEnvironmentMessage.cpp
	flatbuffer/message/SynMAPMessage.h
	flatbuffer/message/SynMAPMessage.cpp
    flatbuffer/message/SynSPATMessage.h
	flatbuffer/message/SynSPATMessage.cpp

	flatbuffer/message/SynMessageUtils.h
	flatbuffer/message/SynMessageUtils.cpp
	flatbuffer/message/SynMessageFactory.h
	flatbuffer/message/SynMessageFactory.cpp
)
source_group("flatbuffer" FILES ${SYN_FLATBUFFER_FILES})

set(SYN_TERRAIN_FILES
	terrain/SynTerrain.h
	terrain/SynTerrain.cpp

	terrain/SynTerrainFactory.h
	terrain/SynTerrainFactory.cpp

	terrain/SynRigidTerrain.h
	terrain/SynRigidTerrain.cpp
	terrain/SynSCMTerrain.h
	terrain/SynSCMTerrain.cpp
)
source_group("terrain" FILES ${SYN_TERRAIN_FILES})

set(SYN_UTILS_FILES
	utils/SynUtilsJSON.h
	utils/SynUtilsJSON.cpp
	utils/SynDataLoader.h
    utils/SynDataLoader.cpp
    utils/SynGPSTools.h
    utils/SynGPSTools.cpp
)
source_group("utils" FILES ${SYN_UTILS_FILES})

set(SYN_VEHICLE_FILES
	vehicle/SynVehicle.h
	vehicle/SynVehicle.cpp
	vehicle/SynWheeledVehicle.h
	vehicle/SynWheeledVehicle.cpp
	vehicle/SynTrackedVehicle.h
	vehicle/SynTrackedVehicle.cpp
)
source_group("vehicle" FILES ${SYN_VEHICLE_FILES})

set(SYN_VISUALIZATION_FILES
	visualization/SynVisualization.h
	visualization/SynVisualization.cpp
	visualization/SynVisualizationManager.h
	visualization/SynVisualizationManager.cpp

)
if(ENABLE_MODULE_SENSOR)
	list(APPEND SYN_VISUALIZATION_FILES
		visualization/SynSensorVisualization.h
		visualization/SynSensorVisualization.cpp
	)
endif()
if(ENABLE_MODULE_IRRLICHT)
	list(APPEND SYN_VISUALIZATION_FILES
		visualization/SynIrrVehicleVisualization.h
		visualization/SynIrrVehicleVisualization.cpp
	)
endif()
source_group("visualization" FILES ${SYN_VISUALIZATION_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_synchrono library
#-----------------------------------------------------------------------------

if (ENABLE_MODULE_SENSOR)
  # list(APPEND SYN_LIBRARIES ChronoEngine_sensor)
  list(APPEND SYN_LIB_NAMES ChronoEngine_sensor)
endif()

if (ENABLE_MODULE_MKL)
  #list(APPEND SYN_LIBRARIES ChronoEngine_mkl)
  list(APPEND SYN_LIB_NAMES ChronoEngine_mkl)
  list(APPEND SYN_INCLUDES  ${CH_MKL_INCLUDES})
  list(APPEND SYN_CXX_FLAGS ${CH_MKL_CXX_FLAGS})
  list(APPEND SYN_LINKER_FLAGS ${CH_MKL_LINK_FLAGS})
endif()

if (ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_VEHICLE)
  # list(APPEND SYN_LIBRARIES ChronoEngine_vehicle_irrlicht)
  list(APPEND SYN_LIB_NAMES ChronoEngine_vehicle_irrlicht)
endif()

if (ENABLE_MODULE_IRRLICHT)
  # list(APPEND SYN_LIBRARIES ChronoEngine_irrlicht)
  list(APPEND SYN_LIB_NAMES ChronoEngine_irrlicht)

  list(APPEND SYN_INCLUDES  ${CH_IRRLICHTINC})
  list(APPEND SYN_CXX_FLAGS ${CH_IRRLICHT_CXX_FLAGS})
endif()

if (ENABLE_MODULE_VEHICLE)
  # list(APPEND SYN_LIBRARIES ChronoEngine_vehicle)
  # list(APPEND SYN_LIBRARIES ChronoModels_vehicle)
  list(APPEND SYN_LIB_NAMES ChronoEngine_vehicle)
  list(APPEND SYN_LIB_NAMES ChronoModels_vehicle)
endif()

# Set some variables to be visible outside this directory
SET(SYN_INCLUDES     "${SYN_INCLUDES}"  PARENT_SCOPE)
SET(SYN_LIBRARIES    "${SYN_LIBRARIES}" PARENT_SCOPE)
SET(SYN_CXX_FLAGS    "${SYN_CXX_FLAGS}" PARENT_SCOPE)

message(STATUS "SYN_LIBRARIES: ${SYN_LIBRARIES}")
message(STATUS "SYN_INCLUDES: ${SYN_INCLUDES}")

include_directories(${SYN_INCLUDES})

add_library(ChronoEngine_synchrono SHARED
	${SYN_BASE_FILES}
	${SYN_AGENT_FILES}
	${SYN_BRAIN_FILES}
	${SYN_CLI_FILES}
	${SYN_COMMUNICATION_FILES}
	${SYN_FLATBUFFER_FILES}
	${SYN_TERRAIN_FILES}
	${SYN_UTILS_FILES}
	${SYN_VEHICLE_FILES}
	${SYN_VISUALIZATION_FILES}
)

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")

    #FIGURE OUT WHERE ALL THE LIBRARY DLLS FROM WINDOWS ARE
    #SET(CH_IRRLICHT_DLL "${IRRLICHT_ROOT}/bin/Win32-VisualStudio/Irrlicht.dll")
endif()

set_target_properties(ChronoEngine_synchrono PROPERTIES
                      COMPILE_FLAGS "${SYN_CXX_FLAGS}"
					  LINK_FLAGS "${SYN_LINKER_FLAGS}")

target_compile_definitions(ChronoEngine_synchrono PRIVATE "SYN_API_COMPILE")

target_include_directories(ChronoEngine_synchrono PUBLIC ${SYN_INCLUDES})

target_link_libraries(ChronoEngine_synchrono ${SYN_LIB_NAMES} ${SYN_LIBRARIES} ${SYN_OTHER_LIBRARIES})

install(TARGETS ChronoEngine_synchrono
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

#-------------------------------------------------------------------------------
# Install SynChrono headers
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_synchrono
        FILES_MATCHING PATTERN "*.h")