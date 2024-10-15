#=============================================================================
# CMake configuration file for the Chrono domain decomposition module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MULTIDOMAIN "Enable the Chrono MultiDomain module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_MULTIDOMAIN)
	RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono MultiDomain module ====\n")

# Return now if MPI is not available

if(NOT MPI_FOUND)
	message("Chrono::Multidomain requires MPI, but MPI not found. Disabling Chrono::Multidomain")
	set(ENABLE_MODULE_MULTIDOMAIN OFF CACHE BOOL "Enable the Multidomain module" FORCE)
	return()
endif()

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE MULTIDOMAIN LIBRARY



SET(ChronoEngine_MULTIDOMAIN_SOURCES 
    ChDomainManager.cpp
    ChDomainManagerSharedmemory.cpp
    ChDomainManagerMPI.cpp
    ChSystemDescriptorMultidomain.cpp
    ChSolverPSORmultidomain.cpp
    #ChSystemMultidomain.cpp
    ChMpi.cpp
)

SET(ChronoEngine_MULTIDOMAIN_HEADERS
    ChApiMultiDomain.h
    ChDomainManager.h
    ChDomainManagerSharedmemory.h
    ChDomainManagerMPI.h
    ChSystemDescriptorMultidomain.h
    ChSolverPSORmultidomain.h
    #ChSystemMultidomain.h
    ChMpi.h
)

SOURCE_GROUP("" FILES 
            ${ChronoEngine_MULTIDOMAIN_SOURCES} 
            ${ChronoEngine_MULTIDOMAIN_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

# The MULTIDOMAIN library is added to the project,
# and some custom properties of this target are set.

include_directories(${MPI_CXX_HEADER_DIR})

ADD_LIBRARY(ChronoEngine_multidomain 
			${ChronoEngine_MULTIDOMAIN_SOURCES}
			${ChronoEngine_MULTIDOMAIN_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_multidomain PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")
                          
target_compile_definitions(ChronoEngine_multidomain PRIVATE "CH_API_COMPILE_MULTIDOMAIN")
target_compile_definitions(ChronoEngine_multidomain PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_multidomain 
	                  ChronoEngine
					  ${MPI_LIBRARIES})
	
ADD_DEPENDENCIES(ChronoEngine_multidomain ChronoEngine)
	
INSTALL(TARGETS ChronoEngine_multidomain
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)		

INSTALL(FILES ${ChronoEngine_MULTIDOMAIN_HEADERS}
        DESTINATION include/chrono_multidomain)


