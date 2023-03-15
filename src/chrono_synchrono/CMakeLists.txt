#=============================================================================
# CMake configuration file for SynChrono module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_SYNCHRONO "Enable the SynChrono module" OFF)

IF(NOT ENABLE_MODULE_SYNCHRONO)
  return()
ENDIF()

message(STATUS "\n==== SynChrono module ====\n")

# Return now if MPI is not available
if(NOT MPI_FOUND)
	message("Chrono::SynChrono requires MPI, but MPI not found. Disabling Chrono::SynChrono")
	set(ENABLE_MODULE_SYNCHRONO OFF CACHE BOOL "Enable the SynChrono module" FORCE)
	return()
endif()

# Return now if Chrono::Vehicle is not enabled
if(NOT ENABLE_MODULE_VEHICLE)
    message("Chrono::SynChrono depends on Chrono::Vehicle which is disabled. Disabling Chrono::SynChrono")
    set(ENABLE_MODULE_SYNCHRONO OFF CACHE BOOL "Enable the SynChrono module" FORCE)
    return()
endif()

set(SYN_CXX_FLAGS "${CH_CXX_FLAGS}")
set(SYN_LINKER_FLAGS "${CH_LINKERFLAG_SHARED}")
set(SYN_LIBRARIES "")
set(SYN_LIB_NAMES "ChronoEngine")

#-----------------------------------------------------------------------------
# Initialize exported variables
#-----------------------------------------------------------------------------

set(SYN_INCLUDES
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
	set(fastrtps_INSTALL_DIR "" CACHE PATH "Path to FastDDS install dir")

	if (fastrtps_INSTALL_DIR STREQUAL "")
	  set(FASTDDS_FOUND FALSE)
	else()
	  find_package(fastrtps REQUIRED PATHS "${fastrtps_INSTALL_DIR}/cmake" "${fastrtps}/share/fastrtps/cmake")
	  
	  message(STATUS "FastCDR found?        ${fastcdr_FOUND}")
	  message(STATUS "FastCDR include dir:  ${fastcdr_INCLUDE_DIR}")
      message(STATUS "FastCDR lib dir:      ${fastcdr_LIB_DIR}")
	  
	  message(STATUS "FastRTPS found?       ${fastrtps_FOUND}")
	  message(STATUS "FastRTPS include dir: ${fastrtps_INCLUDE_DIR}")
      message(STATUS "FastRTPS lib dir:     ${fastrtps_LIB_DIR}")
	  
	  message(STATUS "FooNathan found?      ${foonathan_memory_FOUND}")

	  if (fastcdr_FOUND AND fastrtps_FOUND AND foonathan_memory_FOUND)
	    list(APPEND SYN_LIBRARIES fastrtps fastcdr)
	    set(SYN_INCLUDES ${SYN_INCLUDES} ${fastrtps_INCLUDE_DIR} ${fastcdr_INCLUDE_DIR})
		set(FASTDDS_FOUND TRUE)
        set(fastrtps_DIR "${fastrtps_DIR}" PARENT_SCOPE)
	  else()
	    set(FASTDDS_FOUND FALSE)
      endif()
	endif()
endif()

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Prepare replacement variables
if(FASTDDS_FOUND)
	set(CHRONO_FASTDDS "#define CHRONO_FASTDDS")
else()
	set(CHRONO_FASTDDS "#undef CHRONO_FASTDDS")
endif()

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

	SynChronoManager.h
	SynChronoManager.cpp
)
source_group("base" FILES ${SYN_BASE_FILES})

set(SYN_AGENT_FILES
	agent/SynAgent.h
	agent/SynAgent.cpp

	agent/SynAgentFactory.h
	agent/SynAgentFactory.cpp

	agent/SynWheeledVehicleAgent.h
	agent/SynWheeledVehicleAgent.cpp
	agent/SynTrackedVehicleAgent.h
	agent/SynTrackedVehicleAgent.cpp
	
	agent/SynCopterAgent.h
	agent/SynCopterAgent.cpp

	agent/SynSCMTerrainAgent.h
	agent/SynSCMTerrainAgent.cpp
    
    agent/SynEnvironmentAgent.h
    agent/SynEnvironmentAgent.cpp
)
source_group("agent" FILES ${SYN_AGENT_FILES})

set(SYN_CONTROLLER_FILES
	controller/SynControllerFunctions.h
	controller/SynControllerFunctions.cpp
	
    controller/driver/SynMultiPathDriver.h
    controller/driver/SynMultiPathDriver.cpp
)
source_group("controller" FILES ${SYN_CONTROLLER_FILES})

set(SYN_COMMUNICATION_FILES
	communication/SynCommunicator.h
	communication/SynCommunicator.cpp
    
    communication/mpi/SynMPICommunicator.h
    communication/mpi/SynMPICommunicator.cpp
)
if(FASTDDS_FOUND)
	list(APPEND SYN_COMMUNICATION_FILES
		communication/dds/SynDDSCommunicator.h
		communication/dds/SynDDSCommunicator.cpp
		
		communication/dds/SynDDSPublisher.h
		communication/dds/SynDDSPublisher.cpp
		communication/dds/SynDDSSubscriber.h
		communication/dds/SynDDSSubscriber.cpp
		communication/dds/SynDDSTopic.h
		communication/dds/SynDDSTopic.cpp
		communication/dds/SynDDSListener.h
		communication/dds/SynDDSListener.cpp

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
    flatbuffer/message/SynSimulationMessage.h
    flatbuffer/message/SynSimulationMessage.cpp

	flatbuffer/message/SynWheeledVehicleMessage.h
	flatbuffer/message/SynWheeledVehicleMessage.cpp
	flatbuffer/message/SynTrackedVehicleMessage.h
	flatbuffer/message/SynTrackedVehicleMessage.cpp

	flatbuffer/message/SynCopterMessage.h
	flatbuffer/message/SynCopterMessage.cpp
	
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

set(SYN_UTILS_FILES
	utils/SynDataLoader.h
	utils/SynDataLoader.cpp
    utils/SynGPSTools.h
    utils/SynGPSTools.cpp
    utils/SynLog.h
    utils/SynLog.cpp
)
source_group("utils" FILES ${SYN_UTILS_FILES})

#-----------------------------------------------------------------------------
# Create the ChronoEngine_synchrono library
#-----------------------------------------------------------------------------

if (ENABLE_MODULE_SENSOR)
  list(APPEND SYN_LIB_NAMES ChronoEngine_sensor)
endif()
  
if (ENABLE_MODULE_IRRLICHT)
  list(APPEND SYN_LIB_NAMES ChronoEngine_irrlicht)
  list(APPEND SYN_LIB_NAMES ChronoEngine_vehicle_irrlicht)

  set(SYN_INCLUDES  ${SYN_INCLUDES} ${CH_IRRLICHT_INCLUDES})
  set(SYN_CXX_FLAGS "${SYN_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
endif()

# Chrono::Vehicle is required
list(APPEND SYN_LIB_NAMES ChronoEngine_vehicle)
list(APPEND SYN_LIB_NAMES ChronoModels_vehicle)

# Set some variables to be visible outside this directory
set(FASTDDS_FOUND "${FASTDDS_FOUND}" PARENT_SCOPE)
set(SYN_INCLUDES  "${SYN_INCLUDES}"  PARENT_SCOPE)
set(SYN_LIBRARIES "${SYN_LIBRARIES}" PARENT_SCOPE)
set(SYN_CXX_FLAGS "${SYN_CXX_FLAGS}" PARENT_SCOPE)

include_directories("${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/flatbuffers/include")
include_directories(${SYN_INCLUDES})

add_library(ChronoEngine_synchrono SHARED
	${SYN_BASE_FILES}
	${SYN_CONTROLLER_FILES}
    ${SYN_AGENT_FILES}
    ${SYN_COMMUNICATION_FILES}
    ${SYN_FLATBUFFER_FILES}
    ${SYN_UTILS_FILES}
)

# windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4661 /wd4005")
endif()

set_target_properties(ChronoEngine_synchrono PROPERTIES
                      COMPILE_FLAGS "${SYN_CXX_FLAGS}"
                      LINK_FLAGS "${SYN_LINKER_FLAGS}")

target_compile_definitions(ChronoEngine_synchrono PRIVATE "SYN_API_COMPILE")

target_include_directories(ChronoEngine_synchrono PUBLIC "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/flatbuffers/include")
target_include_directories(ChronoEngine_synchrono PUBLIC ${SYN_INCLUDES})

target_link_libraries(ChronoEngine_synchrono ${SYN_LIB_NAMES} ${SYN_LIBRARIES})

install(TARGETS ChronoEngine_synchrono
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

#-------------------------------------------------------------------------------
# Install SynChrono headers
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_synchrono
        FILES_MATCHING PATTERN "*.h" PATTERN "fbs" EXCLUDE)

# Install required chrono_thirdparty headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/flatbuffers/include
        DESTINATION include/chrono_thirdparty/flatbuffers
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
