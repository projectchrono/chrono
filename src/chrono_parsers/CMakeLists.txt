#=============================================================================
# CMake configuration file for the Chrono::Parsers module
#=============================================================================

option(ENABLE_MODULE_PARSERS "Enable the Chrono Parsers module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_PARSERS)
    mark_as_advanced(FORCE urdfdom_DIR)
    mark_as_advanced(FORCE urdfdom_headers_DIR)
    mark_as_advanced(FORCE console_bridge_DIR)
    mark_as_advanced(FORCE tinyxml2_DIR)
    return()
endif()

MESSAGE(STATUS "\n==== Chrono Parsers module ====\n")

mark_as_advanced(CLEAR urdfdom_DIR)
mark_as_advanced(CLEAR urdfdom_headers_DIR)
mark_as_advanced(CLEAR console_bridge_DIR)
mark_as_advanced(CLEAR tinyxml2_DIR)

#-----------------------------------------------------------------------------

set(HAVE_URDF FALSE)

find_package(urdfdom QUIET)
if(urdfdom_FOUND)
    find_package(urdfdom_headers REQUIRED)
    find_package(console_bridge REQUIRED)
    #find_package(tinyxml2 REQUIRED)

    #if(urdfdom_headers_FOUND AND console_bridge_FOUND AND tinyxml2_FOUND)
    if(urdfdom_headers_FOUND AND console_bridge_FOUND)
        set(HAVE_URDF TRUE)
        message(STATUS "Found URDFCOM and dependencies.")
        message(STATUS "URDFDOM include dirs: ${urdfdom_INCLUDE_DIRS} ")
    else()
        message(STATUS "A dependency of the package urdfdom NOT found. The Chrono URDF parser will NOT be built.")
    endif()
else()
    message(STATUS "Package urdfdom NOT found. The Chrono URDF parser will NOT be built.")
endif()

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

if(HAVE_URDF)
    set(ChronoEngine_PARSERS_SOURCES ${ChronoEngine_PARSERS_SOURCES} ChParserURDF.cpp)
    set(ChronoEngine_PARSERS_HEADERS ${ChronoEngine_PARSERS_HEADERS} ChParserURDF.h)
endif()

source_group("" FILES ${ChronoEngine_PARSERS_SOURCES} ${ChronoEngine_PARSERS_HEADERS})

#-----------------------------------------------------------------------------

set(CH_PARSERS_INCLUDES "")
if(HAVE_URDF)
    include_directories(${urdfdom_INCLUDE_DIRS})
    set(CH_PARSERS_INCLUDES "${CH_PARSERS_INCLUDES} ${urdfdom_INCLUDE_DIRS}")
endif()
set(HAVE_URDF "${HAVE_URDF}" PARENT_SCOPE)
set(CH_PARSERS_INCLUDES "${CH_PARSERS_INCLUDES}" PARENT_SCOPE)

add_library(ChronoEngine_parsers SHARED ${ChronoEngine_PARSERS_SOURCES} ${ChronoEngine_PARSERS_HEADERS})

set_target_properties(ChronoEngine_parsers PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_parsers PRIVATE "CH_API_COMPILE_PARSERS")
target_compile_definitions(ChronoEngine_parsers PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_parsers ChronoEngine)
if(HAVE_URDF)
    target_link_libraries(ChronoEngine_parsers urdfdom::urdfdom_model)
    target_link_libraries(ChronoEngine_parsers urdfdom::urdfdom_world)
    target_link_libraries(ChronoEngine_parsers urdfdom::urdfdom_sensor)
endif()

install(TARGETS ChronoEngine_parsers
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_PARSERS_HEADERS}
        DESTINATION include/chrono_parsers)
