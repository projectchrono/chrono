#=============================================================================
# CMake configuration file for the Chrono::Parsers module
#=============================================================================

option(ENABLE_MODULE_PARSERS "Enable the Chrono Parsers module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_PARSERS)
    return()
endif()

MESSAGE(STATUS "\n==== Chrono Parsers module ====\n")

#-----------------------------------------------------------------------------

set(ChronoEngine_PARSERS_SOURCES 
    ChParserAdams.cpp
    ChAdamsTokenizer.yy.cpp
    ChParserOpenSim.cpp
)

set(ChronoEngine_PARSERS_HEADERS
    ChApiParsers.h
    ChParserAdams.h 
    ChParserOpenSim.h
)

source_group("" FILES 
            ${ChronoEngine_PARSERS_SOURCES} 
            ${ChronoEngine_PARSERS_HEADERS})

#-----------------------------------------------------------------------------	

include_directories(${SPECTRA_INCLUDE_DIR})

add_library(ChronoEngine_parsers SHARED ${ChronoEngine_PARSERS_SOURCES} ${ChronoEngine_PARSERS_HEADERS})

set_target_properties(ChronoEngine_parsers PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_parsers PRIVATE "CH_API_COMPILE_PARSERS")
target_compile_definitions(ChronoEngine_parsers PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_parsers ChronoEngine)

install(TARGETS ChronoEngine_parsers
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)		

install(FILES ${ChronoEngine_PARSERS_HEADERS}
        DESTINATION include/chrono_parsers)
