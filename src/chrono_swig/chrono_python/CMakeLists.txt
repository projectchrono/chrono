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

message(STATUS "\n==== Chrono Python module ====\n")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

# Python package name
set(CHPY_PACKAGENAME pychrono)


if(DEFINED CH_CONDA_INSTALL)

  message(STATUS "Configure Pychrono for Conda packaging")

  if (NOT DEFINED CH_INSTALL_PYTHON_PACKAGE)
    message(FATAL_ERROR "Installation directory not specified. Set CH_INSTALL_PYTHON_PACKAGE")
  else()
    message(STATUS "Override the CH_INSTALL_PYTHON with CH_INSTALL_PYTHON_PACKAGE")
    set (CH_INSTALL_PYTHON "${CH_INSTALL_PYTHON_PACKAGE}")
  endif()

  if (NOT DEFINED CH_PYCHRONO_DATA_PATH)
    message(FATAL_ERROR "Relative path to Chrono data/ directory not specified. Set CH_PYCHRONO_DATA_PATH")
  endif()

  if (NOT DEFINED CH_PYCHRONO_SHADER_PATH)
    message(FATAL_ERROR "Relative path to Chrono::Sensor shaders/ directory not specified. Set CH_PYCHRONO_SHADER_PATH")
  endif()

endif()

#-----------------------------------------------------------------------------
# CMake policy settings
# TODO: switch to new behavior
#-----------------------------------------------------------------------------

# this to fix warnings about the use of LOCATION with CMake version >= 3.0
if(POLICY CMP0026)
    cmake_policy(SET CMP0026 OLD)
endif()

# this to fix warnings about the use of SWIG_MODULE_${mymodulename}_REAL_NAME with CMake version >= 3.13
if(POLICY CMP0078)
    cmake_policy(SET CMP0078 OLD)
endif()

# UseSWIG honors SWIG_MODULE_NAME via -module flag.
if(POLICY CMP0086)
    cmake_policy(SET CMP0086 OLD)
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

include_directories(${CH_PYTHONINC})

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
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /bigobj")
endif()
 
# Set location of SWIG-generated files.
# Note that *.py files are also generated in this location.
set(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/chrono_python")

# Set SWIG flags.  Disable selected SWIG warnings 
set(CMAKE_SWIG_FLAGS "-c++;-w302,362,389,401,509")

if(DBG_SCRIPT)
  message("SWIG_USE_FILE:      ${SWIG_USE_FILE}")
  message("CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")
  message("Installation path:  ${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_PYTHON}")
  message("Installation prefix path:  ${CMAKE_INSTALL_PREFIX}")
endif()

# Disable additional warnings for generated C++ code
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
elseif(MSVC)
   add_compile_options(/wd4189)  # local variable is initialized but not referenced
   add_compile_options(/wd4275)  # non-DLL-interface class used as base for DLL-interface class
   add_compile_options(/wd4456)  # declaration hides previous local declaration
   add_compile_options(/wd4702)  # unreachable code
   add_compile_options(/wd4706)  # assignment within conditional expression
else()
   add_compile_options(-Wno-unused-variable)
endif()

#-----------------------------------------------------------------------------
# Generate init.py
#-----------------------------------------------------------------------------

# Prepare replacement variables for init.py
set(ADD_CUDA_DLL "")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(ENABLE_MODULE_SENSOR)
    set(ADD_CUDA_DLL "os.add_dll_directory('${CUDA_BINARY_DIR}')")
  endif()
  if(ENABLE_MODULE_CASCADE)
    set(ADD_OCC_DLL "os.add_dll_directory('${OpenCASCADE_BINARY_DIR}')")
  endif()
endif()

# ---------- Pychrono in BUILD tree

# Path to Chrono data/ directory (in SOURCE tree)
set(PYC_DATA_PATH "${PROJECT_SOURCE_DIR}/data/")

# Path to Chrono::Sensor shaders/ directory (in SOURCE tree)
set(PYC_SHADER_PATH "${PROJECT_SOURCE_DIR}/src/chrono_sensor/optix/shaders/")

# Generate the __init__.py file using substitution variables and place it in a temporary place
configure_file(${CMAKE_SOURCE_DIR}/contrib/packaging-python/__init__.py.local.in
               ${PROJECT_BINARY_DIR}/chrono_python/__init__.py)

# ---------- Pychrono in INSTALL tree

if (CH_CONDA_INSTALL)
  # Use specified path to Chrono data/ directory (CH_PYCHRONO_DATA_PATH) *relative* to __init__.py
  set(PYC_DATA_PATH "${CH_PYCHRONO_DATA_PATH}")

  # Use specified path to Chrono::Sensor shaders/ directory (CH_PYCHRONO_SHADER_PATH) *relative* to __init__.py
  set(PYC_SHADER_PATH "${CH_PYCHRONO_SHADER_PATH}") 

  # Generate the __init__.py file using substitution variables and place it in a temporary place 
  configure_file(${CMAKE_SOURCE_DIR}/contrib/packaging-python/__init__.py.conda.in
                 ${PROJECT_BINARY_DIR}/chrono_python/__init__.py.install)
else()
  # Default path to Chrono data/ directory (in INSTALL tree)
  set(PYC_DATA_PATH "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_DATA}/")

  # Default path to Chrono::Sensor shaders/ directory (in INSTALL tree)
  set(PYC_SHADER_PATH "${CMAKE_INSTALL_PREFIX}/include/chrono_sensor/optix/shaders/")

  # Generate the __init__.py file using substitution variables and place it in a temporary place
  configure_file(${CMAKE_SOURCE_DIR}/contrib/packaging-python/__init__.py.local.in
                 ${PROJECT_BINARY_DIR}/chrono_python/__init__.py.install)
endif()

#-----------------------------------------------------------------------------
# Install demos
#-----------------------------------------------------------------------------

### TODO: install demos only for configured modules!!!

install(DIRECTORY "${CMAKE_SOURCE_DIR}/src/demos/python/" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}/demos")

#-----------------------------------------------------------------------------
# MODULE for the core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "...add python CORE module")

# Python module name
set(CHPY_CORE core)

# Interface files
set(CHPY_CORE_MODULE_FILE
    ../interface/core/ChModuleCore.i
    )
set(CHPY_CORE_WRAPPER_FILES
    ../interface/core/ChException.i
    ../interface/core/ChCoordsys.i
    ../interface/core/ChVector.i
    ../interface/core/ChQuaternion.i
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

target_compile_definitions(${SWIG_MODULE_${CHPY_CORE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

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

# Install .pyd binary module (.so on linux)
install(TARGETS ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
        RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
        LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
        ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")

# Install .py module wrapper
install(FILES "${CHPY_CORE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

if (DBG_SCRIPT)
  message("Module name:     ${CHPY_CORE}")
  message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}")
  message("PY file:         ${CHPY_CORE_PY_FILE}")
  message("PYD file:        ${CHPY_CORE_PYD_FILE}")
  message("PYD path:        ${CHPY_CORE_PYD_PATH}")
endif()

#-----------------------------------------------------------------------------
# Relocate init.py
#-----------------------------------------------------------------------------

# ---------- Pychrono in BUILD tree

# Copy the __init__.py (BUILD) file in the package
add_custom_command(TARGET ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_BINARY_DIR}/chrono_python/__init__.py" ${CHPY_CORE_PYD_PATH}/${CHPY_PACKAGENAME})

# ---------- Pychrono in INSTALL tree

# Install __init__.py (INSTALL) to signal the directory is a python package
install(FILES "${PROJECT_BINARY_DIR}/chrono_python/__init__.py.install" 
        DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}" 
        RENAME __init__.py)

#-----------------------------------------------------------------------------
# MODULE for the postprocess python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_POSTPROCESS)

  message(STATUS "...add python POSTPROCESS module")

  # Python module name
  set(CHPY_POSTPROCESS postprocess)

  # Interface files
  set(CHPY_POSTPROCESS_MODULE_FILE
      ../interface/postprocess/ChModulePostprocess.i
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

  target_compile_definitions(${SWIG_MODULE_${CHPY_POSTPROCESS}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

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
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
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

  message(STATUS "...add python FEA module")

  # Python module name
  set(CHPY_FEA fea)

  # Interface files
  set(CHPY_FEA_MODULE_FILE
      ../interface/fea/ChModuleFea.i
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

  target_compile_definitions(${SWIG_MODULE_${CHPY_FEA}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

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
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
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

  message(STATUS "...add python IRRLICHT module")

  # Python module name
  set(CHPY_IRRLICHT irrlicht)

  # Interface files
  set(CHPY_IRRLICHT_MODULE_FILE
      ../interface/irrlicht/ChModuleIrrlicht.i
      )

  include_directories(${CH_IRRLICHT_INCLUDES})

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

  target_compile_definitions(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(${SWIG_MODULE_${CHPY_IRRLICHT}_REAL_NAME} PRIVATE "IGNORE_DEPRECATED_WARNING")

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
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
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
# MODULE for the Pardiso mkl python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_PARDISO_MKL)

  message(STATUS "...add python Pardiso MKL module:  pychrono.pardisomkl")

  # Python module name
  set(CHPY_MKL pardisomkl)

  # Interface files
  set(CHPY_MKL_MODULE_FILE
      ../interface/pardisomkl/ChModulePardisoMkl.i
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
  SWIG_LINK_LIBRARIES(${CHPY_MKL} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_pardisomkl)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} PROPERTIES
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_MKL}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_MKL}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  target_compile_definitions(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_MKL}_REAL_NAME} ChronoEngine_pardisomkl)
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
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_MKL_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_MKL}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_MKL}_REAL_NAME}")
    message("PY file:         ${CHPY_MKL_PY_FILE}")
    message("PYD file:        ${CHPY_MKL_PYD_FILE}")
    message("PYD path:        ${CHPY_MKL_PYD_PATH}")
  endif()

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
      ../interface/cascade/ChModuleCascade.i
      )

  include_directories(${OpenCASCADE_INCLUDE_DIR})

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

  target_compile_definitions(${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

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
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_CASCADE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Cascade includes:  ${OpenCASCADE_INCLUDE_DIR}")
    message("Module name:     ${CHPY_CASCADE}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_CASCADE}_REAL_NAME}")
    message("PY file:         ${CHPY_CASCADE_PY_FILE}")
    message("PYD file:        ${CHPY_CASCADE_PYD_FILE}")
    message("PYD path:        ${CHPY_CASCADE_PYD_PATH}")
  endif()

endif()


#-----------------------------------------------------------------------------
# MODULE for the vehicle python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_VEHICLE AND ENABLE_MODULE_VEHICLE_MODELS)

  message(STATUS "...add python vehicle module:  pychrono.vehicle")

  # Python module name
  set(CHPY_VEHICLE vehicle)

  # Interface files
  set(CHPY_VEHICLE_MODULE_FILE
      ../interface/vehicle/ChModuleVehicle.i
      )

  include_directories(${VEHICLE_INCLUDE_DIR})

  set_source_files_properties(${CHPY_VEHICLE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.

  # Make sure CHRONO_IRRLICHT is defined for the SWIG processor
  if (ENABLE_MODULE_IRRLICHT)
    set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_IRRLICHT")
  endif()

  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_VEHICLE} python ${CHPY_VEHICLE_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_VEHICLE} LANGUAGE python SOURCES ${CHPY_VEHICLE_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_VEHICLE} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_vehicle)
  if(ENABLE_MODULE_IRRLICHT)
    SWIG_LINK_LIBRARIES(${CHPY_VEHICLE} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_vehicle_irrlicht)
  endif()
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

  target_compile_definitions(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ChronoEngine_vehicle)
  if(ENABLE_MODULE_IRRLICHT)
    ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME} ChronoEngine_irrlicht)
  endif()
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
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_VEHICLE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_VEHICLE}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_VEHICLE}_REAL_NAME}")
    message("PY file:         ${CHPY_VEHICLE_PY_FILE}")
    message("PYD file:        ${CHPY_VEHICLE_PYD_FILE}")
    message("PYD path:        ${CHPY_VEHICLE_PYD_PATH}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the sensor python wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_SENSOR)
  message(STATUS "...add python SENSOR module")

  set(NUMPY_INCLUDE_DIR "${NUMPY_INCLUDE_DIR}" CACHE PATH "")

  if(NUMPY_INCLUDE_DIR)
      message(STATUS "   Numpy include directory: ${NUMPY_INCLUDE_DIR}")
  else()
      message("Warning: Numpy include add_subdirectory not set.  The PyChrono sensor module will not be built!")
  endif()
endif()

if(ENABLE_MODULE_SENSOR AND NUMPY_INCLUDE_DIR)

  # Python module name
  set(CHPY_SENSOR sensor)

  # Interface files
  set(CHPY_SENSOR_MODULE_FILE
          ../interface/sensor/ChModuleSensor.i
          )

  include_directories(${NUMPY_INCLUDE_DIR})
  include_directories(${CH_SENSOR_INCLUDES})

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_SENSOR_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_SENSOR_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_SENSOR} python ${CHPY_SENSOR_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_SENSOR} LANGUAGE python SOURCES ${CHPY_SENSOR_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_SENSOR} ${PYTHON_LIBRARY} ChronoEngine ChronoEngine_sensor ${SENSOR_LIBRARIES})

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME} PROPERTIES
          PROJECT_LABEL "ChronoEngine_python_${CHPY_SENSOR}"
          OUTPUT_NAME   "${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME}"
          LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
          )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME} ChronoEngine_sensor)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir
  set(CHPY_SENSOR_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_SENSOR}.py")
  get_target_property(CHPY_SENSOR_PYD_FILE ${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_SENSOR_PYD_PATH "${CHPY_SENSOR_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME}
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_SENSOR_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME}
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_SENSOR_PY_FILE} ${CHPY_SENSOR_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_SENSOR_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_SENSOR}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CHPY_SENSOR}_REAL_NAME}")
    message("PY file:         ${CHPY_SENSOR_PY_FILE}")
    message("PYD file:        ${CHPY_SENSOR_PYD_FILE}")
    message("PYD path:        ${CHPY_SENSOR_PYD_PATH}")
  endif()

endif()


#-----------------------------------------------------------------------------
# Module for the robot models
#-----------------------------------------------------------------------------

  message(STATUS "...add python robot module:  pychrono.robot")

  # Python module name
  set(CHPY_RS robot)

  # Interface files
  set(CHPY_ROBOT_MODULE_FILE 
      ../interface/robot/ChModuleRobot.i
      )

  #include_directories(${VEHICLE_INCLUDE_DIR})

  set_source_files_properties(${CHPY_ROBOT_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
    SWIG_ADD_MODULE(${CHPY_RS} python ${CHPY_ROBOT_MODULE_FILE})
  else()
    SWIG_ADD_LIBRARY(${CHPY_RS} LANGUAGE python SOURCES ${CHPY_ROBOT_MODULE_FILE})
  endif()
  SWIG_LINK_LIBRARIES(${CHPY_RS} ${PYTHON_LIBRARY} ChronoEngine ChronoModels_robot)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    ADD_DEFINITIONS( "/DWNT" )
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_DEFINITIONS(-DHAVE_IOSTREAM)
    ADD_DEFINITIONS(-DHAVE_LIMITS_H)
  ENDIF()

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(${SWIG_MODULE_${CHPY_RS}_REAL_NAME} PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()
  
  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CHPY_RS}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "ChronoEngine_python_${CHPY_RS}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CHPY_RS}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        )

  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_RS}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_RS}_REAL_NAME} ChronoModels_robot)
  ADD_DEPENDENCIES(${SWIG_MODULE_${CHPY_RS}_REAL_NAME} ${SWIG_MODULE_${CHPY_CORE}_REAL_NAME})

  # Copy PY file in a package dir 
  set(CHPY_VEHICLE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_RS}.py")
  get_target_property(CHPY_RS_PYD_FILE ${SWIG_MODULE_${CHPY_RS}_REAL_NAME} LOCATION)
  get_filename_component(CHPY_VEHICLE_PYD_PATH "${CHPY_RS_PYD_FILE}" PATH)
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_RS}_REAL_NAME}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_VEHICLE_PYD_PATH}/${CHPY_PACKAGENAME})
  add_custom_command(TARGET ${SWIG_MODULE_${CHPY_RS}_REAL_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_VEHICLE_PY_FILE} ${CHPY_VEHICLE_PYD_PATH}/${CHPY_PACKAGENAME})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${SWIG_MODULE_${CHPY_RS}_REAL_NAME}
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_VEHICLE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/${CHPY_PACKAGENAME}")

#-----------------------------------------------------------------------------
# Display information about setting the PYTHONPATH environment variable
#-----------------------------------------------------------------------------

message(STATUS "")
message(STATUS "To have access to the Chrono::Python wrapper modules, after building and (optionally) installing,")
message(STATUS "append one of the following to the PYTHONPATH environment variable:")
message(STATUS "  For the modules in the BUILD tree:    ${CHPY_CORE_PYD_PATH}")
message(STATUS "  For the modules in the INSTALL tree:  ${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_PYTHON}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  message(STATUS "Above, \$(Configuration) represents the current build configuration (Release, Debug, RelWithDebInfo, etc)")
endif()
message(STATUS "")
