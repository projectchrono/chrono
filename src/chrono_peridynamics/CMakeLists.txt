#=============================================================================
# CMake configuration file for the Chrono peridynamics module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_PERIDYNAMICS "Enable the Chrono Peridynamics module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_PERIDYNAMICS)
	RETURN()
endif()

message(STATUS "\n==== Chrono Peridynamics module ====\n")

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE PERIDYNAMICS LIBRARY

set(Chrono_PERIDYNAMICS_SOURCES 
    ChNodePeri.cpp
    ChMatterPeridynamics.cpp
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

source_group("" FILES 
            ${Chrono_PERIDYNAMICS_SOURCES} 
            ${Chrono_PERIDYNAMICS_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

# The PERIDYNAMICS library is added to the project,
# and some custom properties of this target are set.

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

#set_target_properties(Chrono_peridynamics PROPERTIES
#                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
#                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")
                          
target_compile_definitions(Chrono_peridynamics PRIVATE "CH_API_COMPILE_PERIDYNAMICS")
target_compile_definitions(Chrono_peridynamics PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(Chrono_peridynamics PUBLIC  
					  Chrono_core)
	
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

# Old way
install(FILES ${Chrono_PardisoMKL_HEADERS}
        DESTINATION include/chrono_peridynamics)


