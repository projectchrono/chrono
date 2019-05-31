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

# this to fix warnings about the use of LOCATION with CMake version >= 3.0
if(POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
endif()

# this to fix warnings about the use of SWIG_MODULE_${mymodulename}_REAL_NAME with CMake version >= 3.13
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 OLD)
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
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

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
  message("Installation prefix path:  ${CMAKE_INSTALL_PREFIX}")
endif()

#-----------------------------------------------------------------------------
# MODULE for the core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "...add Chrono::Python CORE module")

# Python module name
set(CHPY_CORE core)

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
set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY ON)
source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})

# Create the SWIG module.
if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
  SWIG_ADD_MODULE(${CHPY_CORE} python ${CHPY_CORE_MODULE_FILE}) 
else()
  SWIG_ADD_LIBRARY(${CHPY_CORE} LANGUAGE python SOURCES ${CHPY_CORE_MODULE_FILE})
endif()
SWIG_LINK_LIBRARIES(${CHPY_CORE} ${PYTHON_LIBRARY} ChronoEngine)

# Ensure that the PYD library file is generated in the bin/ directory.
set_target_properties(${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} PROPERTIES 
                      PROJECT_LABEL "ChronoEngine_python_${CHPY_CORE}"
                      OUTPUT_NAME   "${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}"
                      LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                      )

ADD_DEPENDENCIES (${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} ChronoEngine)

# Copy PY file in a package dir
set(CHPY_CORE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_CORE}.py")
get_target_property(CHPY_CORE_PYD_FILE ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} LOCATION)
get_filename_component(CHPY_CORE_PYD_PATH "${CHPY_CORE_PYD_FILE}" PATH)
add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_CORE_PYD_PATH}/${CHPY_PACKAGENAME})
add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CORE_PY_FILE} ${CHPY_CORE_PYD_PATH}/${CHPY_PACKAGENAME})

# Copy the __init__.py file in the package
add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_SOURCE_DIR}/contrib/packaging-python/__init__.py" ${CHPY_CORE_PYD_PATH}/${CHPY_PACKAGENAME})

# Install .pyd binary module (.so on linux)
install(TARGETS ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
        RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
        LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
        ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
# Install .py module wrapper 
install(FILES "${CHPY_CORE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
# Install __init__.py to signal the directory is a python package
install(FILES "${CMAKE_SOURCE_DIR}/contrib/packaging-python/__init__.py" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
# Install demos
install(DIRECTORY "${CMAKE_SOURCE_DIR}/src/demos/python/" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}/demos")


if (DBG_SCRIPT)
  message("Module name:     ${CHPY_CORE}")
  message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}")
  message("PY file:         ${CHPY_CORE_PY_FILE}")
  message("PYD file:        ${CHPY_CORE_PYD_FILE}")
  message("PYD path:        ${CHPY_CORE_PYD_PATH}")
endif()


#-----------------------------------------------------------------------------
# MODULE for the postprocess python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_POSTPROCESS)

  message(STATUS "...add Chrono::Python POSTPROCESS module")
  
  # Python module name
  set(CHPY_POSTPROCESS postprocess)

  # Interface files
  set(CHPY_POSTPROCESS_MODULE_FILE 
      ChModulePostprocess.i
      )

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_POSTPROCESS} python ${CHPY_POSTPROCESS_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_POSTPROCESS} LANGUAGE python SOURCES ${CHPY_POSTPROCESS_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_POSTPROCESS} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_postprocess)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_POSTPROCESS}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} ChronoEngine_postprocess)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_POSTPROCESS_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_POSTPROCESS}.py")
  get_target_property(CHPY_POSTPROCESS_PYD_FILE ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_POSTPROCESS_PYD_PATH "${CHPY_POSTPROCESS_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_POSTPROCESS_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_POSTPROCESS_PY_FILE} ${CHPY_POSTPROCESS_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
		  LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
  # Install .py module wrapper
  install(FILES "${CHPY_POSTPROCESS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_POSTPROCESS}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME}")
    message("PY file:         ${CHPY_POSTPROCESS_PY_FILE}")
    message("PYD file:        ${CHPY_POSTPROCESS_PYD_FILE}")
    message("PYD path:        ${CHPY_POSTPROCESS_PYD_PATH}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the fea python wrapper.
#-----------------------------------------------------------------------------

  message(STATUS "...add Chrono::Python FEA module")
  
  # Python module name
  set(CHPY_FEA fea)

  # Interface files
  set(CHPY_FEA_MODULE_FILE 
      ChModuleFea.i
      )

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_FEA} python ${CHPY_FEA_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_FEA} LANGUAGE python SOURCES ${CHPY_FEA_MODULE_FILE})
  endif()
SWIG_LINK_LIBRARIES(${CHPY_FEA} ${PYTHON_LIBRARY} ChronoEngine)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_FEA}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_FEA_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_FEA}.py")
  get_target_property(CHPY_FEA_PYD_FILE ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_FEA_PYD_PATH "${CHPY_FEA_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_FEA_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_FEA_PY_FILE} ${CHPY_FEA_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
  # Install .py module wrapper
  install(FILES "${CHPY_FEA_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_FEA}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_FEA}_REAL_NAME}")
    message("PY file:         ${CHPY_FEA_PY_FILE}")
    message("PYD file:        ${CHPY_FEA_PYD_FILE}")
    message("PYD path:        ${CHPY_FEA_PYD_PATH}")
  endif()

#-----------------------------------------------------------------------------
# MODULE for the irrlicht python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_IRRLICHT)

  message(STATUS "...add Chrono::Python IRRLICHT module")
  
  # Python module name
  set(CHPY_IRRLICHT irrlicht)

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
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_IRRLICHT} python ${CHPY_IRRLICHT_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_IRRLICHT} LANGUAGE python SOURCES ${CHPY_IRRLICHT_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_IRRLICHT} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_irrlicht)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_IRRLICHT}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} ChronoEngine_irrlicht)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_IRRLICHT_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_IRRLICHT}.py")
  get_target_property(CHPY_IRRLICHT_PYD_FILE ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_IRRLICHT_PYD_PATH "${CHPY_IRRLICHT_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_IRRLICHT_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_IRRLICHT_PY_FILE} ${CHPY_IRRLICHT_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
  # Install .py module wrapper
  install(FILES "${CHPY_IRRLICHT_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_IRRLICHT}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME}")
    message("PY file:         ${CHPY_IRRLICHT_PY_FILE}")
    message("PYD file:        ${CHPY_IRRLICHT_PYD_FILE}")
    message("PYD path:        ${CHPY_IRRLICHT_PYD_PATH}")
  endif()

endif()


#-----------------------------------------------------------------------------
# MODULE for the mkl python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_MKL)

  message(STATUS "...add python MKL module:  pychrono.mkl")
  
  # Python module name
  set(CHPY_MKL mkl)

  # Interface files
  set(CHPY_MKL_MODULE_FILE 
      ChModuleMkl.i
      )

  include_directories(${CH_MKL_INCLUDES})

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_MKL_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_MKL_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_MKL} python ${CHPY_MKL_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_MKL} LANGUAGE python SOURCES ${CHPY_MKL_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_MKL} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_mkl)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_MKL}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_MKL}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} ChronoEngine_mkl)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_MKL_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_MKL}.py")
  get_target_property(CHPY_MKL_PYD_FILE ${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_MKL_PYD_PATH "${CHPY_MKL_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_MKL}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_MKL_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_MKL}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_MKL_PY_FILE} ${CHPY_MKL_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_MKL}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
		  LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
  # Install .py module wrapper
  install(FILES "${CHPY_MKL_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

endif()


#-----------------------------------------------------------------------------
# MODULE for the cascade python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_CASCADE)

  message(STATUS "...add python CASCADE module:  pychrono.cascade")
  
  # Python module name
  set(CHPY_CASCADE cascade)

  # Interface files
  set(CHPY_CASCADE_MODULE_FILE 
      ChModuleCascade.i
      )

  include_directories(${CASCADE_INCLUDE_DIR})

  set_source_files_properties(${CHPY_CASCADE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_CASCADE} python ${CHPY_CASCADE_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_CASCADE} LANGUAGE python SOURCES ${CHPY_CASCADE_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_CASCADE} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_cascade)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS( "/DWNT" )
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
  ENDIF()

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()
  
  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_CASCADE}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} ChronoEngine_cascade)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_CASCADE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_CASCADE}.py")
  get_target_property(CHPY_CASCADE_PYD_FILE ${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_CASCADE_PYD_PATH "${CHPY_CASCADE_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_CASCADE_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CASCADE_PY_FILE} ${CHPY_CASCADE_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
		  LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
  # Install .py module wrapper
  install(FILES "${CHPY_CASCADE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

endif()


#-----------------------------------------------------------------------------
# MODULE for the vehicle python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_VEHICLE)

  message(STATUS "...add python vehicle module:  pychrono.vehicle")
  
  # Python module name
  set(CHPY_VEHICLE vehicle)

  # Interface files
  set(CHPY_VEHICLE_MODULE_FILE 
      ChModuleVehicle.i
      )

  include_directories(${VEHICLE_INCLUDE_DIR})

  set_source_files_properties(${CHPY_VEHICLE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_VEHICLE} python ${CHPY_VEHICLE_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_VEHICLE} LANGUAGE python SOURCES ${CHPY_VEHICLE_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_VEHICLE} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_vehicle)
  SWIG_LINK_LIBRARIES(${CHPY_VEHICLE} ${PYTHON_LIBRARY} ChronoEngine ChronoModels_vehicle)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS( "/DWNT" )
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
  ENDIF()

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()
  
  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_VEHICLE}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ChronoEngine_vehicle)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ChronoModels_vehicle)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_VEHICLE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_VEHICLE}.py")
  get_target_property(CHPY_VEHICLE_PYD_FILE ${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_VEHICLE_PYD_PATH "${CHPY_VEHICLE_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_VEHICLE_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_VEHICLE_PY_FILE} ${CHPY_VEHICLE_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
		  LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")
  # Install .py module wrapper
  install(FILES "${CHPY_VEHICLE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

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
