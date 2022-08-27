#=============================================================================
# CMake configuration file for the Chrono Python Parser module
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_PYTHON "Enable the Chrono Python module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_PYTHON)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  return()
endif()

message(STATUS "\n==== Chrono Pyparser module ====\n")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

# Python package name
set(CHPY_PACKAGENAME pychrono)

if(DEFINED CH_INSTALL_PYTHON_PACKAGE)
  message(STATUS "Override the CH_INSTALL_PYTHON with CH_INSTALL_PYTHON_PACKAGE")
  set (CH_INSTALL_PYTHON "${CH_INSTALL_PYTHON_PACKAGE}")
endif()

#-----------------------------------------------------------------------------
# Enable debugging CMake output
#-----------------------------------------------------------------------------

set(DBG_SCRIPT false)

#-----------------------------------------------------------------------------
# Find PYTHON
#-----------------------------------------------------------------------------

message(STATUS "...find Python")

set(Python_ADDITIONAL_VERSIONS 3.4)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs ${PYTHON_VERSION_STRING} EXACT)

get_filename_component(CH_PYTHONDIR "${PYTHON_EXECUTABLE}" PATH)
set(CH_PYTHONINC "${PYTHON_INCLUDE_DIR}")
set(CH_PYTHONLIB "${PYTHON_LIBRARIES}")

if (DBG_SCRIPT)
  message("CH_PYTHONDIR:   ${CH_PYTHONDIR}")
  message("CH_PYTHONINC:   ${CH_PYTHONINC}")
  message("CH_PYTHONLIB:   ${CH_PYTHONLIB}")
endif()

# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick
set(CH_PYTHONDIR  "${CH_PYTHONDIR}"  PARENT_SCOPE)
set(CH_PYTHONINC  "${CH_PYTHONINC}"  PARENT_SCOPE)
set(CH_PYTHONLIB  "${CH_PYTHONLIB}"  PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Create the ChronoEngine_pyparser library
#-----------------------------------------------------------------------------

message(STATUS "...add Chrono PYPARSER library")

# List the files in the Python parser library to be used via C++
set(ChronoEngine_PYPARSER_SOURCES
    ChPython.cpp
    )
set(ChronoEngine_PYPARSER_HEADERS
    ChApiPyParser.h
    ChPython.h
    ChSwigutils.h
    )

source_group(""  FILES  ${ChronoEngine_PYPARSER_SOURCES} ${ChronoEngine_PYPARSER_HEADERS})

include_directories(${CH_PYTHONINC})

add_library(ChronoEngine_pyparser SHARED
            ${ChronoEngine_PYPARSER_SOURCES}
            ${ChronoEngine_PYPARSER_HEADERS})

set_target_properties(ChronoEngine_pyparser PROPERTIES
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoEngine_pyparser PRIVATE "CH_API_COMPILE_PYPARSER")
target_compile_definitions(ChronoEngine_pyparser PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_pyparser ${CH_PYTHONLIB} ChronoEngine)

ADD_DEPENDENCIES(ChronoEngine_pyparser ChronoEngine)

install(TARGETS ChronoEngine_pyparser
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoEngine_PYPARSER_HEADERS} DESTINATION include/chrono_pyparser)
