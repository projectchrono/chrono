#=============================================================================
# CMake configuration file for SynChrono module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(CH_ENABLE_MODULE_SYNCHRONO "Enable the SynChrono module" OFF)

IF(NOT CH_ENABLE_MODULE_SYNCHRONO)
  return()
ENDIF()

message(STATUS "\n==== SynChrono module ====\n")

# Return now if MPI is not available
if(NOT MPI_CXX_FOUND)
	message("Chrono::SynChrono requires MPI, but MPI not found. Disabling Chrono::SynChrono")
	set(CH_ENABLE_MODULE_SYNCHRONO OFF CACHE BOOL "Enable the SynChrono module" FORCE)
	return()
endif()

# Return now if Chrono::Vehicle is not enabled
if(NOT CH_ENABLE_MODULE_VEHICLE)
    message("Chrono::SynChrono depends on Chrono::Vehicle which is disabled. Disabling Chrono::SynChrono")
    set(CH_ENABLE_MODULE_SYNCHRONO OFF CACHE BOOL "Enable the SynChrono module" FORCE)
    return()
endif()

# Return now if the Chrono vehicle models are not enabled
if(NOT CH_ENABLE_MODULE_VEHICLE_MODELS)
    message("Chrono::SynChrono depends on the vehicle models library which is disabled. Disabling Chrono::SynChrono")
    set(CH_ENABLE_MODULE_SYNCHRONO OFF CACHE BOOL "Enable the SynChrono module" FORCE)
    return()
endif()

# ----------------------------------------------------------------------------
# Create base library
# ----------------------------------------------------------------------------

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
	utils/SynDataPath.h
	utils/SynDataPath.cpp
    utils/SynGPSTools.h
    utils/SynGPSTools.cpp
    utils/SynLog.h
    utils/SynLog.cpp
)
source_group("utils" FILES ${SYN_UTILS_FILES})

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_SYNCHRONO "")
if(CH_ENABLE_MODULE_VEHICLE)
  list (APPEND DEPENDENCIES_SYNCHRONO "VEHICLE")
endif()
if(CH_ENABLE_MODULE_SENSOR)
  list (APPEND DEPENDENCIES_SYNCHRONO "SENSOR")
endif()
if(CH_ENABLE_MODULE_SENSOR)
  list (APPEND DEPENDENCIES_SYNCHRONO "IRRLICHT")
endif()

set(DEPENDENCIES_SYNCHRONO ${DEPENDENCIES_SYNCHRONO} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Generate and install configuration file
#-----------------------------------------------------------------------------

set(SYN_CONFIG_FILE ${PROJECT_BINARY_DIR}/chrono_synchrono/SynConfig.h)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SynConfig.h.in ${SYN_CONFIG_FILE} @ONLY)

source_group("" FILES ${SYN_CONFIG_FILE})

install(FILES ${SYN_CONFIG_FILE} DESTINATION include/chrono_synchrono)

#-----------------------------------------------------------------------------
# Add the Chrono_synchrono library
#-----------------------------------------------------------------------------

add_library(Chrono_synchrono
    ${SYN_BASE_FILES}
    ${SYN_CONTROLLER_FILES}
    ${SYN_AGENT_FILES}
    ${SYN_COMMUNICATION_FILES}
    ${SYN_FLATBUFFER_FILES}
    ${SYN_UTILS_FILES}
	${SYN_CONFIG_FILE}
)
add_library(Chrono::synchrono ALIAS Chrono_synchrono)

set_target_properties(Chrono_synchrono PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_synchrono PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_synchrono PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_synchrono PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_synchrono PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

# Windows builds should disable warning 4661 and 4005
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_options(Chrono_synchrono PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4661>)
    target_compile_options(Chrono_synchrono PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4005>)
endif()

target_compile_definitions(Chrono_synchrono PRIVATE $<$<COMPILE_LANGUAGE:CXX>:SYN_API_COMPILE>)

target_include_directories(Chrono_synchrono PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/flatbuffers/include>
  $<INSTALL_INTERFACE:include/chrono_thirdparty/flatbuffers/include>
)

# Add dependencies
if (CH_ENABLE_MODULE_SENSOR)
  target_link_libraries(Chrono_synchrono PUBLIC Chrono_sensor) # needs PUBLIC
  find_package(OptiX REQUIRED)
  target_link_libraries(Chrono_synchrono INTERFACE OptiX::OptiX) # needs INTERFACE
endif()
  
if (CH_ENABLE_MODULE_IRRLICHT)
  target_link_libraries(Chrono_synchrono PRIVATE Chrono_irrlicht)
  target_link_libraries(Chrono_synchrono PUBLIC Chrono_vehicle_irrlicht) # needs PUBLIC because of IrrKlang
endif()

target_link_libraries(Chrono_synchrono PRIVATE Chrono_core)
target_link_libraries(Chrono_synchrono PUBLIC Chrono_vehicle) # needs PUBLIC
target_link_libraries(Chrono_synchrono PRIVATE ChronoModels_vehicle)
target_link_libraries(Chrono_synchrono PRIVATE ChronoModels_robot)
target_link_libraries(Chrono_synchrono PUBLIC MPI::MPI_CXX) # needs PUBLIC

#-----------------------------------------------------------------------------
# Find DDS (Optional)
#-----------------------------------------------------------------------------

message(STATUS "Synchrono configured with MPI backend")

option(CH_USE_SYNCHRONO_FASTDDS "Enable the FastDDS interface for the SynChrono Module" OFF)

set(FASTDDS_FOUND FALSE)

if (CH_USE_SYNCHRONO_FASTDDS)
	set(FastDDS_ROOT "" CACHE PATH "Path to FastDDS install directory")

	if (FastDDS_ROOT STREQUAL "")
	  set(FASTDDS_FOUND FALSE)
	else()
	  message(STATUS "Find FastRTPS")
	  find_package(fastrtps REQUIRED PATHS "${FastDDS_ROOT}/cmake" "${fastrtps}/share/fastrtps/cmake")
	  
	  message(STATUS "  FastCDR found?        ${fastcdr_FOUND}")
	  message(STATUS "  FastCDR include dir:  ${fastcdr_INCLUDE_DIR}")
      message(STATUS "  FastCDR lib dir:      ${fastcdr_LIB_DIR}")
	  message(STATUS "  FastRTPS found?       ${fastrtps_FOUND}")
	  message(STATUS "  FastRTPS include dir: ${fastrtps_INCLUDE_DIR}")
      message(STATUS "  FastRTPS lib dir:     ${fastrtps_LIB_DIR}")
	  message(STATUS "  FooNathan found?      ${foonathan_memory_FOUND}")

	  if (fastcdr_FOUND AND fastrtps_FOUND AND foonathan_memory_FOUND)
		target_link_libraries(Chrono_synchrono PUBLIC fastrtps fastcdr)
	    ##set(SYN_INCLUDES ${SYN_INCLUDES} ${fastrtps_INCLUDE_DIR} ${fastcdr_INCLUDE_DIR})
		set(FASTDDS_FOUND TRUE)
        set(fastrtps_DIR "${fastrtps_DIR}" PARENT_SCOPE)
	  else()
	    set(FASTDDS_FOUND FALSE)
      endif()
	endif()

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
	source_group("communication" FILES ${SYN_COMMUNICATION_FILES})
	target_sources(Chrono_synchrono PRIVATE ${SYN_COMMUNICATION_FILES})
	set(CHRONO_SYNCHRONO_USE_FASTDDS "#define CHRONO_SYNCHRONO_USE_FASTDDS")

    message(STATUS "Synchrono configured with FastDDS backend")

else()
	set(CHRONO_SYNCHRONO_USE_FASTDDS "#undef CHRONO_SYNCHRONO_USE_FASTDDS")
	message(STATUS "Synchrono configured without FastDDS backend")
endif()

# Set some variables to be visible outside this directory
set(FASTDDS_FOUND "${FASTDDS_FOUND}" PARENT_SCOPE)


install(TARGETS Chrono_synchrono
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
		INCLUDES DESTINATION include/chrono_synchrono)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_synchrono
        FILES_MATCHING PATTERN "*.h" PATTERN "fbs" EXCLUDE)

# Install 3rd party headers       
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/flatbuffers/include
        DESTINATION include/chrono_thirdparty/flatbuffers
        FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh" PATTERN "*.hpp" PATTERN "*.inl")
