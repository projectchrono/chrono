#=============================================================================
# CMake configuration file for the Chrono domain decomposition module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_MULTIDOMAIN "Enable the Chrono MultiDomain module" OFF)

# Return now if this module is not enabled
IF(NOT CH_ENABLE_MODULE_MULTIDOMAIN)
	return()
ENDIF()

message(STATUS "\n==== Chrono MultiDomain module ====\n")

# Return now if MPI is not available

if(NOT MPI_FOUND)
	message("Chrono::Multidomain requires MPI, but MPI not found. Disabling Chrono::Multidomain")
	set(CH_ENABLE_MODULE_MULTIDOMAIN OFF CACHE BOOL "Enable the Multidomain module" FORCE)
	return()
endif()

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE MULTIDOMAIN LIBRARY



set(Chrono_MULTIDOMAIN_SOURCES 
    ChDomain.cpp
    ChDomainBuilder.cpp
    ChDomainManager.cpp
    ChDomainManagerSharedmemory.cpp
    ChDomainManagerMPI.cpp
    ChSystemDescriptorMultidomain.cpp
    ChSolverPSORmultidomain.cpp
    ChSolverLumpedMultidomain.cpp
    ChSolverLumpedPSORMultidomain.cpp
    ChSolverBBmultidomain.cpp
    ChMpi.cpp
)

set(Chrono_MULTIDOMAIN_HEADERS
    ChApiMultiDomain.h
    ChDomain.h
    ChDomainBuilder.h
    ChDomainManager.h
    ChDomainManagerSharedmemory.h
    ChDomainManagerMPI.h
    ChSystemDescriptorMultidomain.h
    ChSolverPSORmultidomain.h
    ChSolverLumpedMultidomain.h
    ChSolverLumpedPSORMultidomain.h
    ChSolverBBmultidomain.h
    ChMpi.h
)

source_group("" FILES 
            ${Chrono_MULTIDOMAIN_SOURCES} 
            ${Chrono_MULTIDOMAIN_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

# The MULTIDOMAIN library is added to the project,
# and some custom properties of this target are set.

include_directories(${MPI_CXX_HEADER_DIR})

add_library(Chrono_multidomain 
			${Chrono_MULTIDOMAIN_SOURCES}
			${Chrono_MULTIDOMAIN_HEADERS})

set_target_properties(Chrono_multidomain PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")
                          
target_compile_definitions(Chrono_multidomain PRIVATE "CH_API_COMPILE_MULTIDOMAIN")
target_compile_definitions(Chrono_multidomain PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(Chrono_multidomain PRIVATE Chrono_core)
target_link_libraries(Chrono_multidomain PUBLIC ${MPI_LIBRARIES})

install(TARGETS Chrono_multidomain
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/Chrono_multidomain)
	
		

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_POSTPROCESS_HEADERS}
        DESTINATION include/Chrono_multidomain)


