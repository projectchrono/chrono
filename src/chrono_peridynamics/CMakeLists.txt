#=============================================================================
# CMake configuration file for the Chrono peridynamics module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_PERIDYNAMICS "Enable the Chrono Peridynamics module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_PERIDYNAMICS)
	RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Peridynamics module ====\n")

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE PERIDYNAMICS LIBRARY

SET(ChronoEngine_PERIDYNAMICS_SOURCES 
    ChNodePeri.cpp
    ChMatterPeridynamics.cpp
    ChMatterPeriSprings.cpp
    ChMatterPeriBulkElastic.cpp
    ChProximityContainerPeridynamics.cpp
)

SET(ChronoEngine_PERIDYNAMICS_HEADERS
    ChApiPeridynamics.h 
    ChNodePeri.h
    ChMatterPeridynamics.h
    ChMatterPeriSprings.h
    ChMatterPeriBulkElastic.h
    ChProximityContainerPeridynamics.h
)

SOURCE_GROUP("" FILES 
            ${ChronoEngine_PERIDYNAMICS_SOURCES} 
            ${ChronoEngine_PERIDYNAMICS_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

# The PERIDYNAMICS library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(ChronoEngine_peridynamics 
			${ChronoEngine_PERIDYNAMICS_SOURCES}
			${ChronoEngine_PERIDYNAMICS_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_peridynamics PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")
                          
target_compile_definitions(ChronoEngine_peridynamics PRIVATE "CH_API_COMPILE_PERIDYNAMICS")
target_compile_definitions(ChronoEngine_peridynamics PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_peridynamics 
	                  ChronoEngine)
	
ADD_DEPENDENCIES(ChronoEngine_peridynamics ChronoEngine)
	
INSTALL(TARGETS ChronoEngine_peridynamics
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)		

INSTALL(FILES ${ChronoEngine_PERIDYNAMICS_HEADERS}
        DESTINATION include/chrono_peridynamics)


