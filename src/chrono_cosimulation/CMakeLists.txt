#=============================================================================
# CMake configuration file for chrono cosimulation module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_COSIMULATION "Enable the Chrono Cosimulation module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_COSIMULATION)
  return()
endif()

MESSAGE(STATUS "\n==== Chrono Cosimulation module ====\n")

set(ChronoEngine_COSIMULATION_SOURCES
    ChCosimulation.cpp
)

set(ChronoEngine_COSIMULATION_HEADERS
    ChApiCosimulation.h
    ChCosimulation.h
)

source_group("" FILES
             ${ChronoEngine_COSIMULATION_SOURCES}
             ${ChronoEngine_COSIMULATION_HEADERS})

#-----------------------------------------------------------------------------	

# Add the COSIMULATION library

add_library(ChronoEngine_cosimulation SHARED
            ${ChronoEngine_COSIMULATION_SOURCES}
            ${ChronoEngine_COSIMULATION_HEADERS})

set_target_properties(ChronoEngine_cosimulation PROPERTIES 
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_cosimulation PRIVATE "CH_API_COMPILE_COSIMULATION")
target_compile_definitions(ChronoEngine_cosimulation PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_cosimulation ChronoEngine)

add_dependencies(ChronoEngine_cosimulation ChronoEngine)

install(TARGETS ChronoEngine_cosimulation
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_COSIMULATION_HEADERS}
        DESTINATION include/chrono_cosimulation)
