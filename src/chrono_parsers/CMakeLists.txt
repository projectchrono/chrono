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
    find_package(tinyxml2 REQUIRED)

    if(urdfdom_headers_FOUND AND console_bridge_FOUND AND tinyxml2_FOUND)
        set(HAVE_URDF TRUE)
        message(STATUS "Found URDFDOM and dependencies.")
        message(STATUS "  URDFDOM include dirs: ${urdfdom_INCLUDE_DIRS} ")
    else()
        message(STATUS "A dependency of the package urdfdom NOT found. The Chrono URDF parser will NOT be built.")
    endif()
else()
    message(STATUS "Package urdfdom NOT found. The Chrono URDF parser will NOT be built.")
endif()

#-----------------------------------------------------------------------------

set(HAVE_PYTHON FALSE)

set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonInterp QUIET)
find_package(PythonLibs QUIET)

if(PythonInterp_FOUND AND PythonLibs_FOUND)
    set(HAVE_PYTHON TRUE)
    message(STATUS "Found Python and dependencies.")
    get_filename_component(CH_PYTHONDIR "${PYTHON_EXECUTABLE}" PATH)
    set(CH_PYTHONINC "${PYTHON_INCLUDE_DIR}")
    set(CH_PYTHONLIB "${PYTHON_LIBRARIES}")
    set(CH_PYTHONDIR  "${CH_PYTHONDIR}"  PARENT_SCOPE)
    set(CH_PYTHONINC  "${CH_PYTHONINC}"  PARENT_SCOPE)
    set(CH_PYTHONLIB  "${CH_PYTHONLIB}"  PARENT_SCOPE)
    message(STATUS "  Python directory:   ${CH_PYTHONDIR}")
    message(STATUS "  Python include dir: ${CH_PYTHONINC}")
    message(STATUS "  Python libraries:   ${PYTHON_LIBRARIES}")
else()
    message(STATUS "Package Python NOT found. The Chrono Python parser will NOT be built.")
endif()

#-----------------------------------------------------------------------------

set(ChronoEngine_PARSERS_SOURCES 
    ChParserAdams.cpp
    ChParserAdamsTokenizer.yy.cpp
    ChParserOpenSim.cpp
    ChRobotActuation.cpp
)

set(ChronoEngine_PARSERS_HEADERS
    ChApiParsers.h
    ChParserAdams.h 
    ChParserOpenSim.h
    ChRobotActuation.h
)

if(HAVE_URDF)
    set(ChronoEngine_PARSERS_SOURCES ${ChronoEngine_PARSERS_SOURCES} ChParserURDF.cpp)
    set(ChronoEngine_PARSERS_HEADERS ${ChronoEngine_PARSERS_HEADERS} ChParserURDF.h)
endif()

if(HAVE_PYTHON)
   set(ChronoEngine_PARSERS_SOURCES ${ChronoEngine_PARSERS_SOURCES} ChParserPython.cpp)
   set(ChronoEngine_PARSERS_HEADERS ${ChronoEngine_PARSERS_HEADERS} ChParserPython.h)
endif()

source_group("" FILES ${ChronoEngine_PARSERS_SOURCES} ${ChronoEngine_PARSERS_HEADERS})

#-----------------------------------------------------------------------------

SET(HAVE_PYTHON "${HAVE_PYTHON}" PARENT_SCOPE)
SET(HAVE_URDF "${HAVE_URDF}" PARENT_SCOPE)

#-----------------------------------------------------------------------------

set(CH_PARSERS_INCLUDES "")
if(HAVE_URDF)
    include_directories(${urdfdom_INCLUDE_DIRS})
    set(CH_PARSERS_INCLUDES "${CH_PARSERS_INCLUDES};${urdfdom_INCLUDE_DIRS}")
endif()
if(HAVE_PYTHON)
    include_directories(${CH_PYTHONINC})
    set(CH_PARSERS_INCLUDES "${CH_PARSERS_INCLUDES};${CH_PYTHONINC}")
endif()
set(HAVE_URDF "${HAVE_URDF}" PARENT_SCOPE)
set(HAVE_PYTHON "${HAVE_PYTHON}" PARENT_SCOPE)
set(CH_PARSERS_INCLUDES "${CH_PARSERS_INCLUDES}" PARENT_SCOPE)

add_library(ChronoEngine_parsers ${ChronoEngine_PARSERS_SOURCES} ${ChronoEngine_PARSERS_HEADERS})

set_target_properties(ChronoEngine_parsers PROPERTIES
                      COMPILE_FLAGS "${CH_CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_LIB}")

target_compile_definitions(ChronoEngine_parsers PRIVATE "CH_API_COMPILE_PARSERS")
target_compile_definitions(ChronoEngine_parsers PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_parsers ChronoEngine)
if(HAVE_URDF)
    target_link_libraries(ChronoEngine_parsers urdfdom::urdfdom_model)
    target_link_libraries(ChronoEngine_parsers urdfdom::urdfdom_world)
    target_link_libraries(ChronoEngine_parsers urdfdom::urdfdom_sensor)
    target_link_libraries(ChronoEngine_parsers tinyxml2::tinyxml2)
endif()
if(HAVE_PYTHON)
    target_link_libraries(ChronoEngine_parsers ${CH_PYTHONLIB})
endif()

install(TARGETS ChronoEngine_parsers
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(FILES ${ChronoEngine_PARSERS_HEADERS}
        DESTINATION include/chrono_parsers)
