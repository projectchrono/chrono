#=============================================================================
# CMake configuration file for the Chrono peridynamics module
#=============================================================================

option(CH_ENABLE_MODULE_PERIDYNAMICS "Enable the Chrono Peridynamics module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_PERIDYNAMICS)
	return()
endif()

message(STATUS "\n==== Chrono Peridynamics module ====\n")

#-----------------------------------------------------------------------------

set(Chrono_PERIDYNAMICS_SOURCES 
    ChNodePeri.cpp
    ChMatterPeriSprings.cpp
    ChMatterPeriBB.cpp
	ChMatterPeriBBimplicit.cpp
	ChMatterPeriLinearElastic.cpp
    ChMatterPeriLiquid.cpp
    ChPeridynamics.cpp
)

set(Chrono_PERIDYNAMICS_HEADERS
    ChApiPeridynamics.h 
    ChNodePeri.h
    ChMatterPeridynamics.h
    ChMatterPeriSprings.h
    ChMatterPeriBB.h
	ChMatterPeriBBimplicit.h
	ChMatterPeriLinearElastic.h
    ChMatterPeriLiquid.h
    ChPeridynamics.h
)

source_group("" FILES ${Chrono_PERIDYNAMICS_SOURCES} ${Chrono_PERIDYNAMICS_HEADERS})

#-----------------------------------------------------------------------------	
# Create the Chrono_peridynamics library
#-----------------------------------------------------------------------------

add_library(Chrono_peridynamics 
			${Chrono_PERIDYNAMICS_SOURCES}
			${Chrono_PERIDYNAMICS_HEADERS})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_peridynamics PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_peridynamics PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_peridynamics PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_peridynamics PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()
                          
target_compile_definitions(Chrono_peridynamics PRIVATE "CH_API_COMPILE_PERIDYNAMICS")
target_compile_definitions(Chrono_peridynamics PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(Chrono_peridynamics PUBLIC Chrono_core)
	
add_dependencies(Chrono_peridynamics Chrono_core)
	
install(TARGETS Chrono_peridynamics
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_peridynamics)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(FILES ${Chrono_PardisoMKL_HEADERS}
        DESTINATION include/chrono_peridynamics)


