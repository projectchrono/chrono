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
    MARK_AS_ADVANCED(FORCE EIGEN3_INCLUDE_DIRS)
    RETURN()
ENDIF()

MESSAGE(STATUS "==== Chrono MKL module ====")

MARK_AS_ADVANCED(CLEAR MATH_LIBRARY)
MARK_AS_ADVANCED(CLEAR IOMP5_LIBRARY)
MARK_AS_ADVANCED(CLEAR EIGEN3_INCLUDE_DIRS)

# ------------------------------------------------------------------------------
# Dependencies for MKL module
# ------------------------------------------------------------------------------

#set(MKL_USE_STATIC_LIBS ON)
#set(MKL_FIND_DEBUG ON)
find_package(MKL)

if(NOT MKL_FOUND)
  set(MKL_ROOT "" CACHE PATH "Location of MKL installation")
endif()

# SET(EIGEN3_INCLUDE_DIRS "" CACHE PATH "Location of Eigen3 headers (should contain ./Engine)")

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the MKL module
# ------------------------------------------------------------------------------

SET(CH_MKL_INCLUDES
    ${MKL_INCLUDE_DIRS}
    # ${EIGEN3_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(${CH_MKL_INCLUDES})

SET(CH_MKL_INCLUDES "${CH_MKL_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List all files in the Chrono mkl module
# ------------------------------------------------------------------------------

SET(ChronoEngine_MKL_HEADERS
  ChApiMkl.h
  ChLcpMklSolver.h
  ChMklEngine.h
  ChApiCSR3.h
  ChCSR3Matrix.h
)

SET(ChronoEngine_MKL_SOURCES
  ChLcpMklSolver.cpp
  ChMklEngine.cpp
  ChCSR3Matrix.cpp
)

SOURCE_GROUP("" FILES ${ChronoEngine_MKL_HEADERS} ${ChronoEngine_MKL_SOURCES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_mkl library
# ------------------------------------------------------------------------------

ADD_LIBRARY(ChronoEngine_mkl SHARED
            ${ChronoEngine_CSR3_HEADERS}
            ${ChronoEngine_CSR3_SOURCES}
            ${ChronoEngine_MKL_SOURCES}
            ${ChronoEngine_MKL_HEADERS})

SET_TARGET_PROPERTIES(ChronoEngine_mkl PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_MKL")

TARGET_LINK_LIBRARIES(ChronoEngine_mkl
                      ChronoEngine
                      ${MKL_LIBRARIES})
# Note: Eigen doesn't have library files

INSTALL(TARGETS ChronoEngine_mkl
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_MKL_HEADERS} 
        DESTINATION include/chrono_mkl)
