#=============================================================================
# CMake configuration file for the Chrono Python module
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_PYTHON "Enable the Chrono Python module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_PYTHON)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  return()
endif()

message(STATUS "\n==== Chrono Python module ====\n")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

if(DEFINED CH_CONDA_INSTALL)

  message(STATUS "Configure Pychrono for Conda packaging")

  if (NOT DEFINED CH_INSTALL_PYTHON_PACKAGE)
    message(FATAL_ERROR "Installation directory not specified. Set CH_INSTALL_PYTHON_PACKAGE")
  else()
    message(STATUS "Override the CH_INSTALL_PYTHON with CH_INSTALL_PYTHON_PACKAGE")
    file(TO_CMAKE_PATH "${CH_INSTALL_PYTHON_PACKAGE}" CH_INSTALL_PYTHON)
  endif()

  if (NOT DEFINED CH_PYCHRONO_DATA_PATH)
    message(FATAL_ERROR "Relative path to Chrono data/ directory not specified. Set CH_PYCHRONO_DATA_PATH")
  endif()

  if (CH_ENABLE_MODULE_SENSOR)
    if (NOT DEFINED CH_PYCHRONO_SHADER_PATH)
      message(FATAL_ERROR "Relative path to Chrono::Sensor shaders/ directory not specified. Set CH_PYCHRONO_SHADER_PATH")
    endif()
  endif()

endif()

#-----------------------------------------------------------------------------
# Enable debugging CMake output
#-----------------------------------------------------------------------------

set(DBG_SCRIPT FALSE)

#-----------------------------------------------------------------------------
# Find PYTHON
#-----------------------------------------------------------------------------

message(STATUS "Find Python")
find_package(Python3 REQUIRED 
             COMPONENTS Interpreter Development
             OPTIONAL_COMPONENTS NumPy)

if(Python3_FOUND)
  message(STATUS "  Python version:       ${Python3_VERSION}")
  message(STATUS "  Python NumPy found?   ${Python3_NumPy_FOUND}")
  if(Python3_NumPy_FOUND)
    message(STATUS "  Python NumPy version: ${Python3_NumPy_VERSION}")
  endif()
endif()

#-----------------------------------------------------------------------------
# Find SWIG
#-----------------------------------------------------------------------------

message(STATUS "Find SWIG")
find_package(SWIG QUIET REQUIRED COMPONENTS python)

if(SWIG_FOUND)
  message(STATUS "  SWIG version:      ${SWIG_VERSION}")
  message(STATUS "  SWIG python found? ${SWIG_python_FOUND}")
  message(STATUS "  SWIG executable:   ${SWIG_EXECUTABLE}")
endif()

include(${SWIG_USE_FILE})

# Set location of SWIG-generated files.
# Note that *.py files are also generated in this location.
set(CMAKE_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/chrono_python")

# Set SWIG flags.  Disable selected SWIG warnings 
set(CHRONO_SWIG_FLAGS "-w302,362,389,401,509")

if(DBG_SCRIPT)
  message("SWIG_USE_FILE:      ${SWIG_USE_FILE}")
  message("CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")
  message("Installation path:  ${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_PYTHON}")
  message("Installation prefix path:  ${CMAKE_INSTALL_PREFIX}")
endif()

#-----------------------------------------------------------------------------
# Generate init.py
#-----------------------------------------------------------------------------

# Prepare replacement variables for init.py
set(ADD_CUDA_DLL "")
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(CH_ENABLE_MODULE_SENSOR)
    set(ADD_CUDA_DLL "os.add_dll_directory('${CUDA_BINARY_DIR}')")
  endif()
  if(CH_ENABLE_MODULE_CASCADE)
    set(ADD_OCC_DLL "os.add_dll_directory('${OpenCASCADE_BINARY_DIR}')")
  endif()
endif()

# ---------- Pychrono in BUILD tree

# Path to Chrono data directory and Chrono::Sensor shaders (in BUILD tree)
set(PYC_DATA_PATH "${CH_BUILD_DATA}/")
set(PYC_SHADER_PATH "${CH_BUILD_SENSOR_SHADERS}/")

# Generate the __init__.py file using substitution variables and place it in a temporary place
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.local.in
               ${PROJECT_BINARY_DIR}/chrono_python/__init__.py)

# ---------- Pychrono in INSTALL tree

if (CH_CONDA_INSTALL)
  # Use specified path to Chrono data directory (CH_PYCHRONO_DATA_PATH) and Chrono::Sensor shaders (CH_PYCHRONO_SHADER_PATH)
  # Note: These are *relative* to __init__.py
  set(PYC_DATA_PATH "${CH_PYCHRONO_DATA_PATH}")
  set(PYC_SHADER_PATH "${CH_PYCHRONO_SHADER_PATH}") 

  # Generate the __init__.py file using substitution variables and place it in a temporary place 
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.conda.in
                 ${PROJECT_BINARY_DIR}/chrono_python/__init__.py.install)
else()
  # Path to Chrono data directory and Chrono::Sensor shaders (in local INSTALL tree)
  set(PYC_DATA_PATH "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_DATA}/")
  set(PYC_SHADER_PATH "${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_SENSOR_SHADERS}/")
  
  # Generate the __init__.py file using substitution variables and place it in a temporary place
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.local.in
                 ${PROJECT_BINARY_DIR}/chrono_python/__init__.py.install)
endif()

# Location of Chrono Python modules

if(MULTI_CONFIG)
  set(CHPY_LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}/$<CONFIG>/pychrono")
else()
  set(CHPY_LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}/pychrono")
endif()


#-----------------------------------------------------------------------------
# Install demos
#-----------------------------------------------------------------------------

### TODO: install demos only for configured modules!!!

install(DIRECTORY "${CMAKE_SOURCE_DIR}/src/demos/python/" DESTINATION "${CH_INSTALL_PYTHON}/pychrono/demos")

#-----------------------------------------------------------------------------
# MODULE for the core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "Add python CORE module")

# Python module name
set(CHPY_CORE core)

# Interface files
set(CHPY_CORE_MODULE_FILE ChModuleCore_python.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
if(MSVC)
  set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
endif()

swig_add_library(${CHPY_CORE} LANGUAGE python SOURCES ${CHPY_CORE_MODULE_FILE})
target_link_libraries(${CHPY_CORE} PRIVATE Python3::Python Chrono_core)

# Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
elseif(MSVC)
   target_compile_options(${CHPY_CORE} PRIVATE "/wd4189")  # local variable is initialized but not referenced
   target_compile_options(${CHPY_CORE} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
   target_compile_options(${CHPY_CORE} PRIVATE "/wd4456")  # declaration hides previous local declaration
   target_compile_options(${CHPY_CORE} PRIVATE "/wd4702")  # unreachable code
   target_compile_options(${CHPY_CORE} PRIVATE "/wd4706")  # assignment within conditional expression
else()
   target_compile_options(${CHPY_CORE} PRIVATE "-Wno-unused-variable")
endif()

if(APPLE)
  # See https://github.com/swig/swig/issues/3278
  target_link_options(${CHPY_CORE} PRIVATE -flat_namespace -undefined suppress)
endif()


# Ensure that the PYD library file is generated in the bin/ directory.
set_target_properties(${CHPY_CORE} PROPERTIES
                      PROJECT_LABEL "Chrono_python_core"
                      LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                      SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                      )

add_dependencies(${CHPY_CORE} Chrono_core)

# Copy PY file in a package dir
set(CHPY_CORE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_CORE}.py")
add_custom_command(TARGET ${CHPY_CORE}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CORE_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

# Install .pyd binary module (.so on linux)
install(TARGETS ${CHPY_CORE}
        EXPORT ChronoTargets
        RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
        LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
        ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

# Install .py module wrapper
install(FILES "${CHPY_CORE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

if (DBG_SCRIPT)
  message("Module name:     ${CHPY_CORE}")
  message("PY file:         ${CHPY_CORE_PY_FILE}")
endif()

#-----------------------------------------------------------------------------
# MODULE for the fea python wrapper.
#-----------------------------------------------------------------------------

message(STATUS "Add python FEA module")

# Python module name
set(CHPY_FEA fea)

# Interface files
set(CHPY_FEA_MODULE_FILE ChModuleFea_python.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
if(MSVC)
  set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
endif()

swig_add_library(${CHPY_FEA} LANGUAGE python SOURCES ${CHPY_FEA_MODULE_FILE})
target_link_libraries(${CHPY_FEA} PRIVATE Python3::Python Chrono_core)

# Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
elseif(MSVC)
  target_compile_options(${CHPY_FEA} PRIVATE "/wd4189")  # local variable is initialized but not referenced
  target_compile_options(${CHPY_FEA} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
  target_compile_options(${CHPY_FEA} PRIVATE "/wd4456")  # declaration hides previous local declaration
  target_compile_options(${CHPY_FEA} PRIVATE "/wd4702")  # unreachable code
  target_compile_options(${CHPY_FEA} PRIVATE "/wd4706")  # assignment within conditional expression
else()
  target_compile_options(${CHPY_FEA} PRIVATE "-Wno-unused-variable")
endif()

if(APPLE)
  # See https://github.com/swig/swig/issues/3278
  target_link_options(${CHPY_FEA} PRIVATE -flat_namespace -undefined suppress)
endif()

# Ensure that the PYD library file is generated in the bin/ directory.
set_target_properties(${CHPY_FEA} PROPERTIES
                      PROJECT_LABEL "Chrono_python_fea"
                      LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                      SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                      )

add_dependencies(${CHPY_FEA} Chrono_core)
add_dependencies(${CHPY_FEA} core)

# Copy PY file in a package dir
set(CHPY_FEA_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_FEA}.py")
add_custom_command(TARGET ${CHPY_FEA}
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_FEA_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

# Install .pyd binary module (.so on linux)
install(TARGETS ${CHPY_FEA}
        EXPORT ChronoTargets
        RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
        LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
        ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
# Install .py module wrapper
install(FILES "${CHPY_FEA_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

if (DBG_SCRIPT)
  message("Module name:     ${CHPY_FEA}")
  message("PY file:         ${CHPY_FEA_PY_FILE}")
endif()

#-----------------------------------------------------------------------------
# Relocate init.py
#-----------------------------------------------------------------------------

# ---------- Pychrono in BUILD tree

# Copy the __init__.py (BUILD) file in the package
add_custom_command(TARGET core
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_BINARY_DIR}/chrono_python/__init__.py" ${CHPY_LIBRARY_OUTPUT_DIRECTORY}/__init__.py)

# ---------- Pychrono in INSTALL tree

# Install __init__.py (INSTALL) to signal the directory is a python package
install(FILES "${PROJECT_BINARY_DIR}/chrono_python/__init__.py.install" 
        DESTINATION "${CH_INSTALL_PYTHON}/pychrono" 
        RENAME __init__.py)

#-----------------------------------------------------------------------------
# MODULE for the postprocess python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_POSTPROCESS)

  message(STATUS "Add python POSTPROCESS module")

  # Python module name
  set(CHPY_POSTPROCESS postprocess)

  # Interface files
  set(CHPY_POSTPROCESS_MODULE_FILE ChModulePostprocess_python.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_POSTPROCESS_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  swig_add_library(${CHPY_POSTPROCESS} LANGUAGE python SOURCES ${CHPY_POSTPROCESS_MODULE_FILE})
  target_link_libraries(${CHPY_POSTPROCESS} PRIVATE Python3::Python Chrono_core Chrono_postprocess)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_POSTPROCESS} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_POSTPROCESS} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_POSTPROCESS} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_POSTPROCESS} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_POSTPROCESS} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_POSTPROCESS} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_POSTPROCESS} PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_POSTPROCESS} PROPERTIES
                        PROJECT_LABEL "Chrono_python_postprocess"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(${CHPY_POSTPROCESS} Chrono_core)
  add_dependencies(${CHPY_POSTPROCESS} Chrono_postprocess)
  add_dependencies(${CHPY_POSTPROCESS} core)

  # Copy PY file in a package dir
  set(CHPY_POSTPROCESS_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_POSTPROCESS}.py")
  add_custom_command(TARGET ${CHPY_POSTPROCESS}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_POSTPROCESS_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_POSTPROCESS}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_POSTPROCESS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_POSTPROCESS}")
    message("PY file:         ${CHPY_POSTPROCESS_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the vsg python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)

  message(STATUS "Add python VSG module")

  # Python module name
  set(CHPY_VSG vsg3d)

  # Interface files
  set(CHPY_VSG_MODULE_FILE ChModuleVsg_python.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_VSG_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_VSG_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_VSG_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  swig_add_library(${CHPY_VSG} LANGUAGE python SOURCES ${CHPY_VSG_MODULE_FILE})
  target_link_libraries(${CHPY_VSG} PRIVATE Python3::Python Chrono_core Chrono_vsg)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_VSG} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_VSG} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_VSG} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_VSG} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_VSG} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_VSG} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_VSG} PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_VSG} PROPERTIES
                        PROJECT_LABEL "Chrono_python_vsg"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(${CHPY_VSG} PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(${CHPY_VSG} Chrono_core)
  add_dependencies(${CHPY_VSG} Chrono_vsg)
  add_dependencies(${CHPY_VSG} core)

  # Copy PY file in a package dir
  set(CHPY_VSG_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_VSG}.py")
  add_custom_command(TARGET ${CHPY_VSG}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_VSG_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_VSG}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_VSG_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_VSG}")
    message("PY file:         ${CHPY_VSG_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the fsi python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_FSI_SPH)

  message(STATUS "Add python FSI module")

  # Python module name
  set(CHPY_FSI fsi)

  set(CHPY_FSI_MODULE_FILE ChModuleFsi_python.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_FSI_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_FSI_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_FSI_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()
  
  # Make sure CHRONO_VSG is defined for the SWIG processor
  if (CH_ENABLE_MODULE_VSG)
    set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_VSG")
  endif()

  swig_add_library(${CHPY_FSI} LANGUAGE python SOURCES ${CHPY_FSI_MODULE_FILE})
  target_link_libraries(${CHPY_FSI} PRIVATE Python3::Python Chrono_core  Chrono_fsi Chrono_fsisph)
  
  if (CH_ENABLE_MODULE_VSG)
    target_link_libraries(${CHPY_FSI} PRIVATE Chrono_vsg Chrono_fsisph_vsg)
  endif()


    # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_FSI} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_FSI} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_FSI} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_FSI} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_FSI} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_FSI} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_FSI} PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_FSI} PROPERTIES
                        PROJECT_LABEL "Chrono_python_fsi"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(${CHPY_FSI} PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(${CHPY_FSI} Chrono_core)
  add_dependencies(${CHPY_FSI} Chrono_fsi)
  add_dependencies(${CHPY_FSI} Chrono_fsisph)
  if (CH_ENABLE_MODULE_VSG)
    add_dependencies(${CHPY_FSI} Chrono_fsisph_vsg)
  endif()

  # Copy PY file in a package dir
  set(CHPY_FSI_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_FSI}.py")
  add_custom_command(TARGET ${CHPY_FSI}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_FSI_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_FSI}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_FSI_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_FSI}")
    message("PY file:         ${CHPY_FSI_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the irrlicht python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_IRRLICHT)

  message(STATUS "Add python IRRLICHT module")
  
  # Python module name
  set(CHPY_IRRLICHT irrlicht)

  # Interface files
  set(CHPY_IRRLICHT_MODULE_FILE ChModuleIrrlicht_python.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_IRRLICHT_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_IRRLICHT_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_IRRLICHT_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  find_package(Irrlicht QUIET REQUIRED)

  swig_add_library(${CHPY_IRRLICHT} LANGUAGE python SOURCES ${CHPY_IRRLICHT_MODULE_FILE})
  target_link_libraries(${CHPY_IRRLICHT} PRIVATE Python3::Python Chrono_core Chrono_irrlicht Irrlicht::Irrlicht)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_IRRLICHT} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_IRRLICHT} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_IRRLICHT} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_IRRLICHT} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_IRRLICHT} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_IRRLICHT} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_IRRLICHT} PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_IRRLICHT} PROPERTIES
                        PROJECT_LABEL "Chrono_python_irrlicht"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(${CHPY_IRRLICHT} PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(${CHPY_IRRLICHT} Chrono_core)
  add_dependencies(${CHPY_IRRLICHT} Chrono_irrlicht)
  add_dependencies(${CHPY_IRRLICHT} core)

  # Copy PY file in a package dir
  set(CHPY_IRRLICHT_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_IRRLICHT}.py")
  add_custom_command(TARGET ${CHPY_IRRLICHT}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_IRRLICHT_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_IRRLICHT}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_IRRLICHT_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_IRRLICHT}")
    message("PY file:         ${CHPY_IRRLICHT_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the Pardiso mkl python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_PARDISO_MKL)

  message(STATUS "Add python PardisoMKL module")

  # Python module name
  set(CHPY_MKL pardisomkl)

  # Interface files
  set(CHPY_MKL_MODULE_FILE ChModulePardisoMkl_python.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_MKL_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_MKL_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_MKL_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  swig_add_library(${CHPY_MKL} LANGUAGE python SOURCES ${CHPY_MKL_MODULE_FILE})
  target_link_libraries(${CHPY_MKL} PRIVATE Python3::Python Chrono_core Chrono_pardisomkl)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_MKL} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_MKL} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_MKL} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_MKL} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_MKL} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_MKL} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_MKL} PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_MKL} PROPERTIES
                        PROJECT_LABEL "Chrono_python_pardisomkl"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(${CHPY_MKL} Chrono_core)
  add_dependencies(${CHPY_MKL} Chrono_pardisomkl)
  add_dependencies(${CHPY_MKL} core)

  # Copy PY file in a package dir
  set(CHPY_MKL_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_MKL}.py")
  add_custom_command(TARGET ${CHPY_MKL}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_MKL_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_MKL}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_MKL_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_MKL}")
    message("PY file:         ${CHPY_MKL_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the cascade python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_CASCADE)

  message(STATUS "Add python CASCADE module")

  # Interface files
  set(CHPY_CASCADE_MODULE_FILE ChModuleCascade_python.i)

  include_directories(${OpenCASCADE_INCLUDE_DIR})

  set_source_files_properties(${CHPY_CASCADE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  swig_add_library(cascade LANGUAGE python SOURCES ${CHPY_CASCADE_MODULE_FILE})
  target_link_libraries(cascade PRIVATE Python3::Python Chrono_core Chrono_cascade)
  if(MSVC)
    set_source_files_properties(${CHPY_CASCADE_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()


  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(cascade PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(cascade PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(cascade PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(cascade PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(cascade PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(cascade PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(cascade PRIVATE "/wd4702")  # unreachable code
    target_compile_options(cascade PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(cascade PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(cascade PRIVATE -flat_namespace -undefined suppress)
  endif()

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(cascade PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(cascade PROPERTIES
                        PROJECT_LABEL "Chrono_python_cascade"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(cascade Chrono_core)
  add_dependencies(cascade Chrono_cascade)
  add_dependencies(cascade core)

  # Copy PY file in a package dir
  set(CHPY_CASCADE_PY_FILE "${CMAKE_SWIG_OUTDIR}/cascade.py")
  add_custom_command(TARGET cascade
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CASCADE_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS cascade
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_CASCADE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Cascade includes:  ${OpenCASCADE_INCLUDE_DIR}")
    message("Module name:     cascade")
    message("PY file:         ${CHPY_CASCADE_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the vehicle python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)

  message(STATUS "Add python VEHICLE module")

  # Python module name
  set(CHPY_VEHICLE vehicle)

  # Interface files
  set(CHPY_VEHICLE_MODULE_FILE ChModuleVehicle_python.i)

  set_source_files_properties(${CHPY_VEHICLE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_VEHICLE_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  # Make sure CHRONO_IRRLICHT is defined for the SWIG processor
  if (CH_ENABLE_MODULE_IRRLICHT)
    set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_IRRLICHT")
  endif()

  # Make sure CHRONO_VSG is defined for the SWIG processor
  if (CH_ENABLE_MODULE_VSG)
      set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_VSG")
  endif()

  # Make sure CHRONO_FSI is defined for the SWIG processor
  if (CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_FSI_SPH)
      set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_FSI")
  endif()

  swig_add_library(${CHPY_VEHICLE} LANGUAGE python SOURCES ${CHPY_VEHICLE_MODULE_FILE})
  target_link_libraries(${CHPY_VEHICLE} PRIVATE Python3::Python Chrono_core Chrono_vehicle ChronoModels_vehicle)
  if(CH_ENABLE_MODULE_IRRLICHT)
    find_package(Irrlicht QUIET REQUIRED)
    target_link_libraries(${CHPY_VEHICLE} PRIVATE Chrono_vehicle_irrlicht Chrono_irrlicht Irrlicht::Irrlicht)
  endif()

  if (CH_ENABLE_MODULE_VSG)
      target_link_libraries(${CHPY_VEHICLE} PRIVATE Chrono_vehicle_vsg Chrono_vsg)
  endif()

  if(CH_ENABLE_MODULE_FSI AND CH_ENABLE_MODULE_FSI_SPH)
    target_link_libraries(${CHPY_VEHICLE} PRIVATE Chrono_fsi Chrono_fsisph)
  endif()

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(${CHPY_VEHICLE} PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(${CHPY_VEHICLE} PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(${CHPY_VEHICLE} PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_VEHICLE} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_VEHICLE} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_VEHICLE} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_VEHICLE} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_VEHICLE} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_VEHICLE} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_VEHICLE} PRIVATE -flat_namespace -undefined suppress)
  endif()


  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(${CHPY_VEHICLE} PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_VEHICLE} PROPERTIES
                        PROJECT_LABEL "Chrono_python_vehicle"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(${CHPY_VEHICLE} Chrono_core)
  add_dependencies(${CHPY_VEHICLE} Chrono_vehicle)
  if(CH_ENABLE_MODULE_IRRLICHT)
    add_dependencies(${CHPY_VEHICLE} Chrono_irrlicht)
  endif()
  add_dependencies(${CHPY_VEHICLE} ChronoModels_vehicle)
  add_dependencies(${CHPY_VEHICLE} core)

  # Copy PY file in a package dir
  set(CHPY_VEHICLE_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_VEHICLE}.py")
  add_custom_command(TARGET ${CHPY_VEHICLE}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_VEHICLE_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_VEHICLE}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_VEHICLE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ${CHPY_VEHICLE}")
    message("PY file:         ${CHPY_VEHICLE_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the sensor python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_SENSOR)
  if(Python3_NumPy_FOUND)
    message(STATUS "Add python SENSOR module")

    # Interface files
    set(CHPY_SENSOR_MODULE_FILE ChModuleSensor_python.i)

    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set_source_files_properties(${CHPY_SENSOR_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
    endif()
    set_source_files_properties(${CHPY_SENSOR_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
    if(MSVC)
      set_source_files_properties(${CHPY_SENSOR_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
    endif()

    swig_add_library(sensor LANGUAGE python SOURCES ${CHPY_SENSOR_MODULE_FILE})
    target_link_libraries(sensor PRIVATE Python3::Python Python3::NumPy Chrono_core Chrono_sensor)

    # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    elseif(MSVC)
      target_compile_options(sensor PRIVATE "/wd4189")  # local variable is initialized but not referenced
      target_compile_options(sensor PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
      target_compile_options(sensor PRIVATE "/wd4456")  # declaration hides previous local declaration
      target_compile_options(sensor PRIVATE "/wd4702")  # unreachable code
      target_compile_options(sensor PRIVATE "/wd4706")  # assignment within conditional expression
    else()
      target_compile_options(sensor PRIVATE "-Wno-unused-variable")
    endif()

    if(APPLE)
      # See https://github.com/swig/swig/issues/3278
      target_link_options(sensor PRIVATE -flat_namespace -undefined suppress)
    endif()

    # Ensure that the PYD library file is generated in the bin/ directory.
    set_target_properties(sensor PROPERTIES
            PROJECT_LABEL "Chrono_python_sensor"
            LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
            SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
            )
  
    add_dependencies(sensor Chrono_core)
    add_dependencies(sensor Chrono_sensor)
    add_dependencies(sensor core)
  
    # Copy PY file in a package dir
    set(CHPY_SENSOR_PY_FILE "${CMAKE_SWIG_OUTDIR}/sensor.py")
    add_custom_command(TARGET sensor
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_SENSOR_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

    # Install .pyd binary module (.so on linux)
    install(TARGETS sensor
            EXPORT ChronoTargets
            RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
            LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
            ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
    # Install .py module wrapper
    install(FILES "${CHPY_SENSOR_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

    if (DBG_SCRIPT)
      message("Module name:     sensor")
      message("PY file:         ${CHPY_SENSOR_PY_FILE}")
    endif()

  else()
    message(WARNING "NumPy cannot be found. PyChrono::Sensor module will not be built.")
  endif()
endif()

#-----------------------------------------------------------------------------
# Module for the robot models
#-----------------------------------------------------------------------------

  message(STATUS "Add python ROBOT module")

  # Python module name
  set(CHPY_RS robot)

  # Interface files
  set(CHPY_ROBOT_MODULE_FILE ChModuleRobot_python.i)

  set_source_files_properties(${CHPY_ROBOT_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_ROBOT_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  swig_add_library(${CHPY_RS} LANGUAGE python SOURCES ${CHPY_ROBOT_MODULE_FILE})
  target_link_libraries(${CHPY_RS} PRIVATE Python3::Python Chrono_core ChronoModels_robot)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(${CHPY_RS} PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(${CHPY_RS} PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(${CHPY_RS} PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(${CHPY_RS} PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(${CHPY_RS} PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(${CHPY_RS} PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(${CHPY_RS} PRIVATE "/wd4702")  # unreachable code
    target_compile_options(${CHPY_RS} PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(${CHPY_RS} PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(${CHPY_RS} PRIVATE -flat_namespace -undefined suppress)
  endif()

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(${CHPY_RS} PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()
  
  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${CHPY_RS} PROPERTIES 
                        PROJECT_LABEL "Chrono_python_robot"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(${CHPY_RS} Chrono_core)
  add_dependencies(${CHPY_RS} ChronoModels_robot)
  add_dependencies(${CHPY_RS} core)

  # Copy PY file in a package dir 
  set(CHPY_ROBOT_PY_FILE "${CMAKE_SWIG_OUTDIR}/${CHPY_RS}.py")
  add_custom_command(TARGET ${CHPY_RS}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_ROBOT_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ${CHPY_RS}
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_ROBOT_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

#-----------------------------------------------------------------------------
# MODULE for the parser python wrapper.
#-----------------------------------------------------------------------------

# Only build if have URDF for now
if(CH_ENABLE_MODULE_PARSERS AND HAVE_URDF)
  message(STATUS "Add python PARSERS module")

  # Interface files
  set(CHPY_PARSERS_MODULE_FILE ChModuleParsers_python.i)

  set_source_files_properties(${CHPY_PARSERS_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_PARSERS_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  swig_add_library(parsers LANGUAGE python SOURCES ${CHPY_PARSERS_MODULE_FILE})
  target_link_libraries(parsers PRIVATE Python3::Python Chrono_core Chrono_parsers)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(parsers PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(parsers PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(parsers PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(parsers PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(parsers PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(parsers PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(parsers PRIVATE "/wd4702")  # unreachable code
    target_compile_options(parsers PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(parsers PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(parsers PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(parsers PROPERTIES
                        PROJECT_LABEL "Chrono_python_parsers"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(parsers Chrono_core)
  add_dependencies(parsers Chrono_parsers)
  add_dependencies(parsers core)

  # Copy PY file in a package dir
  set(CHPY_PARSERS_PY_FILE "${CMAKE_SWIG_OUTDIR}/parsers.py")
  add_custom_command(TARGET parsers
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_PARSERS_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS parsers
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_PARSERS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     parsers")
    message("PY file:         ${CHPY_PARSERS_PY_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the ros python wrapper
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_ROS)
  message(STATUS "Add python ROS module")

  # Force C++17, required by ROS Humble
  set(CMAKE_CXX_STANDARD 17)

  # Interface files
  set(CHPY_ROS_MODULE_FILE
    ../interface/ros/ChModuleROS.i
  )
  set(CHPY_ROS_MODULE_FILE ChModuleROS_python.i)

  set_source_files_properties(${CHPY_ROS_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Make sure CHRONO_PARSERS_URDF and/or CHRONO_SENSOR is defined for the SWIG processor
  if (CH_ENABLE_MODULE_PARSERS AND HAVE_URDF)
    set(CHRONO_SENSOR_SWIG_FLAGS "${CHRONO_SWIG_FLAGS};-DCHRONO_PARSERS_URDF")
  endif()
  if (CH_ENABLE_MODULE_SENSOR)
    set(CHRONO_SENSOR_SWIG_FLAGS "${CHRONO_SENSOR_SWIG_FLAGS};-DCHRONO_SENSOR")
  endif()

  if (CH_ROS_HAS_INTERFACES)
    set(CHRONO_SENSOR_SWIG_FLAGS "${CHRONO_SENSOR_SWIG_FLAGS};-DCHRONO_ROS_HAS_INTERFACES")
  endif()

  if(MSVC)
    set_source_files_properties(${CHPY_ROS_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SENSOR_SWIG_FLAGS})
  endif()

  swig_add_library(ros LANGUAGE python SOURCES ${CHPY_ROS_MODULE_FILE})
  target_link_libraries(ros PRIVATE Python3::Python Chrono_core Chrono_ros)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(ros PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(ros PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(ros PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(ros PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(ros PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(ros PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(ros PRIVATE "/wd4702")  # unreachable code
    target_compile_options(ros PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(ros PRIVATE "-Wno-unused-variable")
  endif()

  if(APPLE)
    # See https://github.com/swig/swig/issues/3278
    target_link_options(ros PRIVATE -flat_namespace -undefined suppress)
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(ros PROPERTIES
                        PROJECT_LABEL "Chrono_python_ros"
                        LIBRARY_OUTPUT_DIRECTORY "${CHPY_LIBRARY_OUTPUT_DIRECTORY}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(ros Chrono_core)
  add_dependencies(ros Chrono_ros)
  add_dependencies(ros core)

  # Copy PY file in a package dir
  set(CHPY_ROS_PY_FILE "${CMAKE_SWIG_OUTDIR}/ros.py")
  add_custom_command(TARGET ros
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_ROS_PY_FILE} ${CHPY_LIBRARY_OUTPUT_DIRECTORY})

  # Install .pyd binary module (.so on linux)
  install(TARGETS ros
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}/pychrono"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}/pychrono")
  # Install .py module wrapper
  install(FILES "${CHPY_ROS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ros")
    message("PY file:         ${CHPY_ROS_PY_FILE}")
    message("PYD file:        ${CHPY_ROS_PYD_FILE}")
  endif()

endif()

#-----------------------------------------------------------------------------
# Display information about setting the PYTHONPATH environment variable
#-----------------------------------------------------------------------------

cmake_path(GET CHPY_LIBRARY_OUTPUT_DIRECTORY PARENT_PATH PYTHONPATH_BUILD)

message(STATUS "")
message(STATUS "To have access to the Chrono::Python wrapper modules, after building and (optionally) installing,")
message(STATUS "append one of the following to the PYTHONPATH environment variable:")
message(STATUS "  For the modules in the BUILD tree:    ${PYTHONPATH_BUILD}")
message(STATUS "  For the modules in the INSTALL tree:  ${CMAKE_INSTALL_PREFIX}/${CH_INSTALL_PYTHON}")
if(MULTI_CONFIG)
  message(STATUS "Above, \$<CONFIG> represents the current build configuration (Release, Debug, RelWithDebInfo, MinSizeRel)")
endif()
message(STATUS "")
