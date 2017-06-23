#=============================================================================
# CMake configuration file for the Chrono MKL module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_MKL "Enable the Chrono MKL module" OFF)

IF(NOT ENABLE_MODULE_MKL)
    MARK_AS_ADVANCED(FORCE MKL_ROOT)
    MARK_AS_ADVANCED(FORCE MATH_LIBRARY)
    MARK_AS_ADVANCED(FORCE IOMP5_LIBRARY)
    RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono MKL module ====")

MARK_AS_ADVANCED(CLEAR MKL_ROOT)
MARK_AS_ADVANCED(CLEAR MATH_LIBRARY)
MARK_AS_ADVANCED(CLEAR IOMP5_LIBRARY)

# ------------------------------------------------------------------------------
# Dependencies for MKL module
# ------------------------------------------------------------------------------

message(STATUS "Find MKL libraries")

#set(MKL_USE_STATIC_LIBS ON)
#set(MKL_FIND_DEBUG ON)
find_package(MKL)

message(STATUS "   MKL include dirs:   ${MKL_INCLUDE_DIRS}")
message(STATUS "   MKL libraries:      ${MKL_LIBRARIES}")
message(STATUS "   IOMP5 library:      ${IOMP5_LIBRARY}")
message(STATUS "   MATH library:       ${MATH_LIBRARY}")
message(STATUS "   MKL library dirs:   ${MKL_LIBRARY_DIRS}")

if(NOT MKL_LIBRARIES)
  set(MKL_ROOT "" CACHE PATH "Location of MKL installation")
endif()

# ------------------------------------------------------------------------------
# Set up linking line for Intel OpenMP library
# ------------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
	SET(CH_MKL_LINK_FLAGS	"-Wl,--as-needed")
endif ()

SET(CH_MKL_LINK_FLAGS "${CH_MKL_LINK_FLAGS}" PARENT_SCOPE)
	
# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the MKL module
# ------------------------------------------------------------------------------

SET(CH_MKL_INCLUDES ${MKL_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})
SET(CH_MKL_INCLUDES "${CH_MKL_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List all files in the Chrono mkl module
# ------------------------------------------------------------------------------

SET(ChronoEngine_MKL_HEADERS
  ChApiMkl.h
  ChSolverMKL.h
  ChMklEngine.h
)

SET(ChronoEngine_MKL_SOURCES
  ChMklEngine.cpp
)

SOURCE_GROUP("" FILES ${ChronoEngine_MKL_HEADERS} ${ChronoEngine_MKL_SOURCES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_mkl library
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_mkl SHARED
            ${ChronoEngine_MKL_SOURCES}
            ${ChronoEngine_MKL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_mkl PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_MKL_LINK_FLAGS}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_MKL")

TARGET_LINK_LIBRARIES(ChronoEngine_mkl
                      ChronoEngine
                      ${MKL_LIBRARIES}
					  ${IOMP5_LIBRARY}
					  )

INSTALL(TARGETS ChronoEngine_mkl
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

INSTALL(FILES ${ChronoEngine_MKL_HEADERS} 
        DESTINATION include/chrono_mkl)
