set(COMPONENT_NAME "FMU2cs_WheeledVehiclePtrain")
set(COMPONENT_SOURCES FMU_WheeledVehiclePtrain.h FMU_WheeledVehiclePtrain.cpp)
set(COMPONENT_RESOURCES_DIR ${CMAKE_SOURCE_DIR}/data/vehicle/fmi/wheeled_vehicle_ptrain)
if (CH_ENABLE_MODULE_IRRLICHT)
  set(COMPONENT_CHRONO_DEPENDENCIES Chrono_core Chrono_vehicle Chrono_irrlicht  Chrono_vehicle_irrlicht)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(COMPONENT_DLL_DEPENDENCIES "${CH_IRRLICHT_DLL}")
  else()
    set(COMPONENT_DLL_DEPENDENCIES "")
  endif()
else()
  set(COMPONENT_CHRONO_DEPENDENCIES Chrono_core Chrono_vehicle)
endif()

set(FMU_CS ON)

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries
list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_vehicle)

include_directories(${FMU_FORGE_DIR})

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
  list(APPEND LIBS Chrono_vehicle_irrlicht)
endif()

#==============================================================

message(STATUS "...add ${COMPONENT_NAME}")
message(STATUS "   FetchContent(fmu-forge)")

set(FMU_MODEL_IDENTIFIER ${COMPONENT_NAME})
set(USE_CUSTOM_TYPESPLATFORM OFF)
set(FMU_MAIN_DIRECTORY ${VEHICLE_FMU_MAIN_DIR})
set(FMU_RESOURCES_DIRECTORY ${COMPONENT_RESOURCES_DIR})
set(FMU_DEPENDENCIES ${COMPONENT_DLL_DEPENDENCIES})
set(FMU_MSG_PREFIX "   - ")

FetchContent_Declare(
    ${FMU_MODEL_IDENTIFIER}
    SOURCE_DIR ${FMU_FORGE_DIR}/fmi2
)

FetchContent_MakeAvailable(${FMU_MODEL_IDENTIFIER})

string(TOUPPER ${FMU_MODEL_IDENTIFIER} FMU_MODEL_IDENTIFIER_UPPERCASE)
MARK_AS_ADVANCED(FETCHCONTENT_BASE_DIR)
MARK_AS_ADVANCED(FETCHCONTENT_FULLY_DISCONNECTED)
MARK_AS_ADVANCED(FETCHCONTENT_QUIET)
MARK_AS_ADVANCED(FETCHCONTENT_SOURCE_DIR_${FMU_MODEL_IDENTIFIER_UPPERCASE})
MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED)
MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED_${FMU_MODEL_IDENTIFIER_UPPERCASE})

#--------------------------------------------------------------

add_dependencies(${FMU_MODEL_IDENTIFIER} ${COMPONENT_CHRONO_DEPENDENCIES})

# add FMU source files to target
target_sources(${FMU_MODEL_IDENTIFIER} PRIVATE ${COMPONENT_SOURCES})

target_link_libraries(${FMU_MODEL_IDENTIFIER} ${LIBS})

if(MSVC)
  set_target_properties(${FMU_MODEL_IDENTIFIER} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()