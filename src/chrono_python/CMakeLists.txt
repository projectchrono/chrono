#=============================================================================
# CMake configuration file for the Chrono Python module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_PYTHON "Enable the Chrono Python module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_PYTHON)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  return()
endif()

message(STATUS "==== Chrono Python module ====")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

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

# this to fix warnings about the use of LOCATION with CMake version >= 3.0
if(POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
endif()

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
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}" 
                      COMPILE_DEFINITIONS "CH_API_COMPILE_PYPARSER")
                          
target_link_libraries(ChronoEngine_pyparser ${CH_PYTHONLIB} ChronoEngine)

ADD_DEPENDENCIES(ChronoEngine_pyparser ChronoEngine)

install(TARGETS ChronoEngine_pyparser
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

#-----------------------------------------------------------------------------
# Find SWIG
#-----------------------------------------------------------------------------

message(STATUS "...find SWIG")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

# The generated .cxx wrapper can be so huge that the /bigobj flag is required in VC++
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   /bigobj")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")
endif()

# Set location of SWIG-generated files.
# Note that *.py files are also generated in this location.
set(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/swig")
SET(CMAKE_SWIG_FLAGS "-c++")

if(DBG_SCRIPT)
  message("SWIG_USE_FILE:      ${SWIG_USE_FILE}")
  message("CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")
  message("Installation path:  ${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_PYTHON}")
endif()

#-----------------------------------------------------------------------------
# MODULE for the ChronoEngine_python_core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "...add Chrono::Python CORE module")

# Python module name
set(CHPY_CORE ChronoEngine_python_core)

# Interface files
set(CHPY_CORE_MODULE_FILE
    ChModuleCore.i
    )
set(CHPY_CORE_WRAPPER_FILES
    ChException.i
    ChCoordsys.i
    ChVector.i
    ChQuaternion.i
    )

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})

# Create the SWIG module.
SWIG_ADD_MODULE(${CHPY_CORE} python ${CHPY_CORE_MODULE_FILE}) 
SWIG_LINK_LIBRARIES(${CHPY_CORE} ${PYTHON_LIBRARY} ChronoEngine)

# Ensure that the PYD library file is generated in the bin/ directory.
set_target_properties(${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} PROPERTIES 
                      PROJECT_LABEL "${CHPY_CORE}"
                      OUTPUT_NAME   "${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}"
                      LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                      )

ADD_DEPENDENCIES (${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} ChronoEngine)

# Copy PY file next to PYD file 
set(CHPY_CORE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_CORE}.py")
get_target_property(CHPY_CORE_PYD_FILE ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} LOCATION)
get_filename_component(CHPY_CORE_PYD_PATH "${CHPY_CORE_PYD_FILE}" PATH)

add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CORE_PY_FILE} ${CHPY_CORE_PYD_PATH})

# Install module
install(TARGETS ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
        RUNTIME DESTINATION ${CH_INSTALL_PYTHON}
        LIBRARY DESTINATION ${CH_INSTALL_PYTHON}
        ARCHIVE DESTINATION ${CH_INSTALL_PYTHON})
install(FILES ${CHPY_CORE_PY_FILE} DESTINATION ${CH_INSTALL_PYTHON})

if (DBG_SCRIPT)
  message("Module name:     ${CHPY_CORE}")
  message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}")
  message("PY file:         ${CHPY_CORE_PY_FILE}")
  message("PYD file:        ${CHPY_CORE_PYD_FILE}")
  message("PYD path:        ${CHPY_CORE_PYD_PATH}")
endif()

#-----------------------------------------------------------------------------
# MODULE for the ChronoEngine_python_postprocess python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_POSTPROCESS)

  message(STATUS "...add Chrono::Python POSTPROCESS module")
  
  # Python module name
  set(CHPY_POSTPROCESS ChronoEngine_python_postprocess)

  # Interface files
  set(CHPY_POSTPROCESS_MODULE_FILE 
      ChModulePostprocess.i
      )

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  SWIG_ADD_MODULE(${CHPY_POSTPROCESS} python ${CHPY_POSTPROCESS_MODULE_FILE}) 
  SWIG_LINK_LIBRARIES(${CHPY_POSTPROCESS} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_postprocess)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "${CHPY_POSTPROCESS}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} ChronoEngine_postprocess)

  # Copy PY file next to PYD file 
  set(CHPY_POSTPROCESS_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_POSTPROCESS}.py")
  get_target_property(CHPY_POSTPROCESS_PYD_FILE ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_POSTPROCESS_PYD_PATH "${CHPY_POSTPROCESS_PYD_FILE}" PATH)

  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_POSTPROCESS_PY_FILE} ${CHPY_POSTPROCESS_PYD_PATH})

  # Install module
  install(TARGETS ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}
          RUNTIME DESTINATION ${CH_INSTALL_PYTHON}
          LIBRARY DESTINATION ${CH_INSTALL_PYTHON}
          ARCHIVE DESTINATION ${CH_INSTALL_PYTHON})
  install(FILES ${CHPY_POSTPROCESS_PY_FILE} DESTINATION ${CH_INSTALL_PYTHON})

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_POSTPROCESS}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}")
    message("PY file:         ${CHPY_POSTPROCESS_PY_FILE}")
    message("PYD file:        ${CHPY_POSTPROCESS_PYD_FILE}")
    message("PYD path:        ${CHPY_POSTPROCESS_PYD_PATH}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the ChronoEngine_python_fea python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_FEA)

  message(STATUS "...add Chrono::Python FEA module")
  
  # Python module name
  set(CHPY_FEA ChronoEngine_python_fea)

  # Interface files
  set(CHPY_FEA_MODULE_FILE 
      ChModuleFea.i
      )

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  SWIG_ADD_MODULE(${CHPY_FEA} python ${CHPY_FEA_MODULE_FILE}) 
  SWIG_LINK_LIBRARIES(${CHPY_FEA} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_fea)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "${CHPY_FEA}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} ChronoEngine_fea)

  # Copy PY file next to PYD file 
  set(CHPY_FEA_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_FEA}.py")
  get_target_property(CHPY_FEA_PYD_FILE ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_FEA_PYD_PATH "${CHPY_FEA_PYD_FILE}" PATH)

  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_FEA_PY_FILE} ${CHPY_FEA_PYD_PATH})

  # Install module
  install(TARGETS ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}
          RUNTIME DESTINATION ${CH_INSTALL_PYTHON}
          LIBRARY DESTINATION ${CH_INSTALL_PYTHON}
          ARCHIVE DESTINATION ${CH_INSTALL_PYTHON})
  install(FILES ${CHPY_FEA_PY_FILE} DESTINATION ${CH_INSTALL_PYTHON})

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_FEA}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}")
    message("PY file:         ${CHPY_FEA_PY_FILE}")
    message("PYD file:        ${CHPY_FEA_PYD_FILE}")
    message("PYD path:        ${CHPY_FEA_PYD_PATH}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the ChronoEngine_python_irrlicht python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_IRRLICHT)

  message(STATUS "...add Chrono::Python IRRLICHT module")
  
  # Python module name
  set(CHPY_IRRLICHT ChronoEngine_python_irrlicht)

  # Interface files
  set(CHPY_IRRLICHT_MODULE_FILE 
      ChModuleIrrlicht.i
      )

  include_directories(${CH_IRRLICHTINC})

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_IRRLICHT_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_IRRLICHT_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  SWIG_ADD_MODULE(${CHPY_IRRLICHT} python ${CHPY_IRRLICHT_MODULE_FILE}) 
  SWIG_LINK_LIBRARIES(${CHPY_IRRLICHT} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_irrlicht ${CH_IRRLICHTLIB})

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "${CHPY_IRRLICHT}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} ChronoEngine_irrlicht)

  # Copy PY file next to PYD file 
  set(CHPY_IRRLICHT_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_IRRLICHT}.py")
  get_target_property(CHPY_IRRLICHT_PYD_FILE ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_IRRLICHT_PYD_PATH "${CHPY_IRRLICHT_PYD_FILE}" PATH)

  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_IRRLICHT_PY_FILE} ${CHPY_IRRLICHT_PYD_PATH})

  # Install module
  install(TARGETS ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}
          RUNTIME DESTINATION ${CH_INSTALL_PYTHON}
          LIBRARY DESTINATION ${CH_INSTALL_PYTHON}
          ARCHIVE DESTINATION ${CH_INSTALL_PYTHON})
  install(FILES ${CHPY_IRRLICHT_PY_FILE} DESTINATION ${CH_INSTALL_PYTHON})

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_IRRLICHT}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}")
    message("PY file:         ${CHPY_IRRLICHT_PY_FILE}")
    message("PYD file:        ${CHPY_IRRLICHT_PYD_FILE}")
    message("PYD path:        ${CHPY_IRRLICHT_PYD_PATH}")
  endif()

endif()


#-----------------------------------------------------------------------------
# Display information about setting the PYTHONPATH environment variable
#-----------------------------------------------------------------------------

message("")
message("To have access to the Chrono::Python wrapper modules, after building and (optionally) installing,")
message("append one of the following to the PYTHONPATH environment variable:")
message("  For the modules in the BUILD tree:    ${CHPY_CORE_PYD_PATH}")
message("  For the modules in the INSTALL tree:  ${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_PYTHON}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  message("Above, \$(Configuration) represents the current build configuration (Release, Debug, RelWithDebInfo, etc)")
endif()
message("")
