#=============================================================================
# CMake configuration file for the Chrono modal module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MODAL "Enable the Chrono Modal module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_MODAL)
	MARK_AS_ADVANCED(FORCE SPECTRA_INCLUDE_DIR)
	RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Modal module ====\n")

MARK_AS_ADVANCED(CLEAR SPECTRA_INCLUDE_DIR)

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE MODAL LIBRARY

SET(ChronoEngine_MODAL_SOURCES 
    ChModalAssembly.cpp
    ChModalDamping.cpp
    ChEigenvalueSolver.cpp
    ChKrylovSchurEig.cpp
)

SET(ChronoEngine_MODAL_HEADERS
    ChApiModal.h 
    ChModalAssembly.h
    ChModalDamping.h
    ChEigenvalueSolver.h
    ChKrylovSchurEig.h
)

SOURCE_GROUP("" FILES 
            ${ChronoEngine_MODAL_SOURCES} 
            ${ChronoEngine_MODAL_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

SET(SPECTRA_INCLUDE_DIR "" CACHE PATH "Path to the SPECTRA include dir, ex. C:/foo/Spectra/includes.")

INCLUDE_DIRECTORIES(${SPECTRA_INCLUDE_DIR})
set(CH_MODAL_INCLUDES "${SPECTRA_INCLUDE_DIR}" PARENT_SCOPE)

# The MODAL library is added to the project,
# and some custom properties of this target are set.

# The generated .cxx wrapper can be so huge that the /bigobj flag is required in VC++
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
endif()

ADD_LIBRARY(ChronoEngine_modal 
			${ChronoEngine_MODAL_SOURCES}
			${ChronoEngine_MODAL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_modal PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")
                          
target_compile_definitions(ChronoEngine_modal PRIVATE "CH_API_COMPILE_MODAL")
target_compile_definitions(ChronoEngine_modal PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoEngine_modal 
	                  ChronoEngine)
	
ADD_DEPENDENCIES(ChronoEngine_modal ChronoEngine)
	
INSTALL(TARGETS ChronoEngine_modal
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)		

INSTALL(FILES ${ChronoEngine_MODAL_HEADERS}
        DESTINATION include/chrono_modal)


