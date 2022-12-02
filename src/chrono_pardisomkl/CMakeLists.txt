#=============================================================================
# CMake configuration file for the Chrono PardisoMKL module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_PARDISO_MKL "Enable the Chrono PardisoMKL module" OFF)

if(NOT ENABLE_MODULE_PARDISO_MKL)
    mark_as_advanced(FORCE MKL_ROOT)
    mark_as_advanced(FORCE MATH_LIBRARY)
    mark_as_advanced(FORCE IOMP5_LIBRARY)
    return()
endif()

message(STATUS "\n==== Chrono PardisoMKL module ====\n")

mark_as_advanced(CLEAR MKL_ROOT)
mark_as_advanced(CLEAR MATH_LIBRARY)
mark_as_advanced(CLEAR IOMP5_LIBRARY)

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

# Make required libraries visible from outside current directory
set(CH_MKL_LIBRARIES ${MKL_LIBRARIES} ${IOMP5_LIBRARY})
set(CH_MKL_LIBRARIES "${CH_MKL_LIBRARIES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Set up linking line for Intel OpenMP library
# ------------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CH_MKL_LINK_FLAGS	"-Wl,--as-needed")
  endif()
endif ()

set(CH_MKL_LINK_FLAGS "${CH_MKL_LINK_FLAGS}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the MKL module
# ------------------------------------------------------------------------------

set(CH_MKL_INCLUDES ${MKL_INCLUDE_DIRS})

include_directories(${CH_MKL_INCLUDES})
set(CH_MKL_INCLUDES "${CH_MKL_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List all files in the Chrono mkl module
# ------------------------------------------------------------------------------

set(ChronoEngine_PardisoMKL_HEADERS
  ChApiPardisoMKL.h
  ChSolverPardisoMKL.h
)

set(ChronoEngine_PardisoMKL_SOURCES
  ChSolverPardisoMKL.cpp
)

source_group("" FILES ${ChronoEngine_PardisoMKL_HEADERS} ${ChronoEngine_PardisoMKL_SOURCES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_pardisomkl library
# ------------------------------------------------------------------------------

add_library(ChronoEngine_pardisomkl SHARED
            ${ChronoEngine_PardisoMKL_SOURCES}
            ${ChronoEngine_PardisoMKL_HEADERS})

set_target_properties(ChronoEngine_pardisomkl PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_MKL_LINK_FLAGS}")

target_compile_definitions(ChronoEngine_pardisomkl PRIVATE "CH_API_COMPILE_PARDISOMKL")
target_compile_definitions(ChronoEngine_pardisomkl PRIVATE "EIGEN_USE_MKL_ALL")
target_compile_definitions(ChronoEngine_pardisomkl PRIVATE "MKL_LP64")
target_compile_definitions(ChronoEngine_pardisomkl PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_pardisomkl
                      ChronoEngine
                      ${MKL_LIBRARIES}
                      ${IOMP5_LIBRARY}
                      )

install(TARGETS ChronoEngine_pardisomkl
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_PardisoMKL_HEADERS}
        DESTINATION include/chrono_pardisomkl)
