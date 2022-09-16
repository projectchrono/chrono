#=============================================================================
# CMake configuration file for the Chrono PardisoProject module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_PARDISO_PROJECT "Enable the Chrono PardisoProject module" OFF)

if(NOT ENABLE_MODULE_PARDISO_PROJECT)
    return()
endif()

message(STATUS "\n==== Chrono PardisoProject module ====\n")

# ------------------------------------------------------------------------------
# Dependencies for Chrono PardisoProject module
# ------------------------------------------------------------------------------
# ------------------ PardisoProject

SET(PARDISOPROJECT_LIBRARIES "C:/workspace/libraries/pardisoproject/libpardiso600-WIN-X86-64.lib" CACHE FILEPATH "Location of Pardiso Project import (.lib) library")

if(NOT EXISTS "${PARDISOPROJECT_LIBRARIES}")
  MESSAGE(SEND_ERROR "The Pardiso Project library set as \"${PARDISOPROJECT_LIBRARIES}\" does not exist.")
  RETURN()
else()
  MESSAGE(STATUS "  PardisoProject library found: ${PARDISOPROJECT_LIBRARIES}")
endif()

set(PARDISOPROJECT_LIBRARIES "${PARDISOPROJECT_LIBRARIES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Set up linking line for Intel OpenMP library
# ------------------------------------------------------------------------------

# if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#   if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#     set(CH_PARDISOPROJECT_LINK_FLAGS	"-Wl,--as-needed")
#   endif()
# endif ()

set(CH_PARDISOPROJECT_LINK_FLAGS "${CH_PARDISOPROJECT_LINK_FLAGS}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# Collect all additional include directories necessary for the PardisoProject module
# ------------------------------------------------------------------------------

set(CH_PARDISOPROJECT_INCLUDES ${PARDISOPROJECT_INCLUDE_DIRS})

include_directories(${CH_PARDISOPROJECT_INCLUDES})
set(CH_PARDISOPROJECT_INCLUDES "${CH_PARDISOPROJECT_INCLUDES}" PARENT_SCOPE)

# ------------------------------------------------------------------------------
# List all files in the Chrono PardisoProject module
# ------------------------------------------------------------------------------

set(ChronoEngine_PARDISOPROJECT_HEADERS
  ChApiPardisoProject.h
  ChSolverPardisoProject.h
  ChPardisoProjectEngine.h
)

set(ChronoEngine_PARDISOPROJECT_SOURCES
  ChSolverPardisoProject.cpp
  ChPardisoProjectEngine.cpp
)

source_group("" FILES ${ChronoEngine_PARDISOPROJECT_HEADERS} ${ChronoEngine_PARDISOPROJECT_SOURCES})

# ------------------------------------------------------------------------------
# Add the ChronoEngine_pardisoproject library
# ------------------------------------------------------------------------------

add_library(ChronoEngine_pardisoproject SHARED
            ${ChronoEngine_PARDISOPROJECT_SOURCES}
            ${ChronoEngine_PARDISOPROJECT_HEADERS})

set_target_properties(ChronoEngine_pardisoproject PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED} ${CH_PARDISOPROJECT_LINK_FLAGS}")

target_compile_definitions(ChronoEngine_pardisoproject PRIVATE "CH_API_COMPILE_PARDISOPROJECT")
target_compile_definitions(ChronoEngine_pardisoproject PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_pardisoproject
                      ChronoEngine
                      ${PARDISOPROJECT_LIBRARIES}
                      )


install(TARGETS ChronoEngine_pardisoproject
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_PARDISOPROJECT_HEADERS}
        DESTINATION include/chrono_pardisoproject)
