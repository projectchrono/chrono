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
# Enable debugging CMake output
#-----------------------------------------------------------------------------

set(DBG_SCRIPT FALSE)

#-----------------------------------------------------------------------------
# Find PYTHON
#-----------------------------------------------------------------------------

message(STATUS "Find Python")
set(Python_ADDITIONAL_VERSIONS 3.4)
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

# Path to Chrono data/ directory (in SOURCE tree)
set(PYC_DATA_PATH "${PROJECT_SOURCE_DIR}/data/")

if(CH_USE_CUDA_NVRTC)
  set(PYC_SHADER_PATH "${PROJECT_SOURCE_DIR}/src/chrono_sensor/optix/shaders/")
else()
  set(PYC_SHADER_PATH "${CMAKE_BINARY_DIR}/lib/sensor_ptx/")
endif()

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

install(DIRECTORY "${CMAKE_SOURCE_DIR}/src/demos/python/" DESTINATION "${CH_INSTALL_PYTHON}/pychrono/demos")

#-----------------------------------------------------------------------------
# MODULE for the core wrapper, including most of the C::E
#-----------------------------------------------------------------------------

message(STATUS "Add python CORE module")

# Interface files
set(CHPY_CORE_MODULE_FILE ChModuleCore_python.i)

set(CHPY_CORE_WRAPPER_FILES
    ../interface/core/ChCoordsys.i
    ../interface/core/ChVector3.i
    ../interface/core/ChQuaternion.i
    )

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY ON)
source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})
if(MSVC)
  set_source_files_properties(${CHPY_CORE_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
endif()



swig_add_library(core LANGUAGE python SOURCES ${CHPY_CORE_MODULE_FILE})
target_link_libraries(core PRIVATE Python3::Python Chrono_core)

# Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
elseif(MSVC)
   target_compile_options(core PRIVATE "/wd4189")  # local variable is initialized but not referenced
   target_compile_options(core PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
   target_compile_options(core PRIVATE "/wd4456")  # declaration hides previous local declaration
   target_compile_options(core PRIVATE "/wd4702")  # unreachable code
   target_compile_options(core PRIVATE "/wd4706")  # assignment within conditional expression
else()
   target_compile_options(core PRIVATE "-Wno-unused-variable")
endif()

# Ensure that the PYD library file is generated in the bin/ directory.
set_target_properties(core PROPERTIES
                      PROJECT_LABEL "Chrono_python_core"
                      LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                      SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                      )



add_dependencies(core Chrono_core)

# Copy PY file in a package dir
set(CHPY_CORE_PY_FILE "${CMAKE_SWIG_OUTDIR}/core.py")
set(CHPY_CORE_PYD_PATH $<TARGET_FILE_DIR:core>)

add_custom_command(TARGET core
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_CORE_PYD_PATH}/pychrono)
add_custom_command(TARGET core
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CORE_PY_FILE} ${CHPY_CORE_PYD_PATH}/pychrono)

# Install .pyd binary module (.so on linux)
install(TARGETS core
        EXPORT ChronoTargets
        RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
        LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
        ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")

# Install .py module wrapper
install(FILES "${CHPY_CORE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

if (DBG_SCRIPT)
  message("Module name:     core")
  message("PY file:         ${CHPY_CORE_PY_FILE}")
  message("PYD path:        ${CHPY_CORE_PYD_PATH}")
endif()

#-----------------------------------------------------------------------------
# Relocate init.py
#-----------------------------------------------------------------------------

# ---------- Pychrono in BUILD tree

# Copy the __init__.py (BUILD) file in the package
add_custom_command(TARGET core
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_BINARY_DIR}/chrono_python/__init__.py" ${CHPY_CORE_PYD_PATH}/pychrono)

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


  swig_add_library(postprocess LANGUAGE python SOURCES ${CHPY_POSTPROCESS_MODULE_FILE})
  target_link_libraries(postprocess PRIVATE Python3::Python Chrono_core Chrono_postprocess)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(postprocess PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(postprocess PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(postprocess PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(postprocess PRIVATE "/wd4702")  # unreachable code
    target_compile_options(postprocess PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(postprocess PRIVATE "-Wno-unused-variable")
  endif()


  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(postprocess PROPERTIES
                        PROJECT_LABEL "Chrono_python_postprocess"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(postprocess Chrono_core)
  add_dependencies(postprocess Chrono_postprocess)
  add_dependencies(postprocess core)

  # Copy PY file in a package dir
  set(CHPY_POSTPROCESS_PY_FILE "${CMAKE_SWIG_OUTDIR}/postprocess.py")

  set(CHPY_POSTPROCESS_PYD_PATH $<TARGET_FILE_DIR:postprocess>)

  add_custom_command(TARGET postprocess
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_POSTPROCESS_PYD_PATH}/pychrono)
  add_custom_command(TARGET postprocess
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_POSTPROCESS_PY_FILE} ${CHPY_POSTPROCESS_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS postprocess
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_POSTPROCESS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     postprocess")
    message("PY file:         ${CHPY_POSTPROCESS_PY_FILE}")
    message("PYD path:        ${CHPY_POSTPROCESS_PYD_PATH}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the fea python wrapper.
#-----------------------------------------------------------------------------

  message(STATUS "Add python FEA module")

  # Python module name
  set(CHPY_FEA fea)

  # Interface files
  set(CHPY_FEA_MODULE_FILE
      ../interface/fea/ChModuleFea.i
      )
  set(CHPY_FEA_MODULE_FILE ChModuleFea_python.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${CHPY_FEA_MODULE_FILE} PROPERTIES COMPILE_OPTIONS ${CHRONO_SWIG_FLAGS})
  endif()

  swig_add_library(fea LANGUAGE python SOURCES ${CHPY_FEA_MODULE_FILE})
  target_link_libraries(fea PRIVATE Python3::Python Chrono_core)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(fea PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(fea PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(fea PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(fea PRIVATE "/wd4702")  # unreachable code
    target_compile_options(fea PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(fea PRIVATE "-Wno-unused-variable")
  endif()


  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(fea PROPERTIES
                        PROJECT_LABEL "Chrono_python_fea"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(fea Chrono_core)
  add_dependencies(fea core)

  # Copy PY file in a package dir
  set(CHPY_FEA_PY_FILE "${CMAKE_SWIG_OUTDIR}/fea.py")

  set(CHPY_FEA_PYD_PATH $<TARGET_FILE_DIR:fea>)

  add_custom_command(TARGET fea
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_FEA_PYD_PATH}/pychrono)
  add_custom_command(TARGET fea
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_FEA_PY_FILE} ${CHPY_FEA_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS fea
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_FEA_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     fea")
    message("PY file:         ${CHPY_FEA_PY_FILE}")
    message("PYD path:        ${CHPY_FEA_PYD_PATH}")
  endif()

#-----------------------------------------------------------------------------
# MODULE for the irrlicht python wrapper.
#-----------------------------------------------------------------------------



if(CH_ENABLE_MODULE_IRRLICHT)

  message(STATUS "Add python IRRLICHT module")

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

  swig_add_library(irrlicht LANGUAGE python SOURCES ${CHPY_IRRLICHT_MODULE_FILE})
  target_link_libraries(irrlicht PRIVATE Python3::Python Chrono_core Chrono_irrlicht Irrlicht::Irrlicht)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(irrlicht PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(irrlicht PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(irrlicht PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(irrlicht PRIVATE "/wd4702")  # unreachable code
    target_compile_options(irrlicht PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(irrlicht PRIVATE "-Wno-unused-variable")
  endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(irrlicht PROPERTIES
                        PROJECT_LABEL "Chrono_python_irrlicht"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )
  
  target_compile_definitions(irrlicht PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(irrlicht Chrono_core)
  add_dependencies(irrlicht Chrono_irrlicht)
  add_dependencies(irrlicht core)

  # Copy PY file in a package dir
  set(CHPY_IRRLICHT_PY_FILE "${CMAKE_SWIG_OUTDIR}/irrlicht.py")

  set(CHPY_IRRLICHT_PYD_PATH $<TARGET_FILE_DIR:irrlicht>)

  add_custom_command(TARGET irrlicht
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_IRRLICHT_PYD_PATH}/pychrono)
  add_custom_command(TARGET irrlicht
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_IRRLICHT_PY_FILE} ${CHPY_IRRLICHT_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS irrlicht
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_IRRLICHT_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     irrlicht")
    message("PY file:         ${CHPY_IRRLICHT_PY_FILE}")
    message("PYD path:        ${CHPY_IRRLICHT_PYD_PATH}")
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

  swig_add_library(pardisomkl LANGUAGE python SOURCES ${CHPY_MKL_MODULE_FILE})
  target_link_libraries(pardisomkl PRIVATE Python3::Python Chrono_core Chrono_pardisomkl)

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(pardisomkl PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(pardisomkl PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(pardisomkl PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(pardisomkl PRIVATE "/wd4702")  # unreachable code
    target_compile_options(pardisomkl PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(pardisomkl PRIVATE "-Wno-unused-variable")
  endif()


  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(pardisomkl PROPERTIES
                        PROJECT_LABEL "Chrono_python_pardisomkl"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(pardisomkl Chrono_core)
  add_dependencies(pardisomkl Chrono_pardisomkl)
  add_dependencies(pardisomkl core)

  # Copy PY file in a package dir
  set(CHPY_MKL_PY_FILE "${CMAKE_SWIG_OUTDIR}/pardisomkl.py")

  set(CHPY_MKL_PYD_PATH $<TARGET_FILE_DIR:pardisomkl>)

  add_custom_command(TARGET pardisomkl
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_MKL_PYD_PATH}/pychrono)
  add_custom_command(TARGET pardisomkl
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_MKL_PY_FILE} ${CHPY_MKL_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS pardisomkl
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_MKL_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     pardisomkl")
    message("PY file:         ${CHPY_MKL_PY_FILE}")
    message("PYD path:        ${CHPY_MKL_PYD_PATH}")
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

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(cascade PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(cascade PROPERTIES
                        PROJECT_LABEL "Chrono_python_cascade"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(cascade Chrono_core)
  add_dependencies(cascade Chrono_cascade)
  add_dependencies(cascade core)

  # Copy PY file in a package dir
  set(CHPY_CASCADE_PY_FILE "${CMAKE_SWIG_OUTDIR}/cascade.py")

  set(CHPY_CASCADE_PYD_PATH $<TARGET_FILE_DIR:cascade>)

  add_custom_command(TARGET cascade
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_CASCADE_PYD_PATH}/pychrono)
  add_custom_command(TARGET cascade
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_CASCADE_PY_FILE} ${CHPY_CASCADE_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS cascade
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_CASCADE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Cascade includes:  ${OpenCASCADE_INCLUDE_DIR}")
    message("Module name:     cascade")
    message("PY file:         ${CHPY_CASCADE_PY_FILE}")
    message("PYD path:        ${CHPY_CASCADE_PYD_PATH}")
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

  swig_add_library(vehicle LANGUAGE python SOURCES ${CHPY_VEHICLE_MODULE_FILE})
  target_link_libraries(vehicle PRIVATE Python3::Python Chrono_core Chrono_vehicle ChronoModels_vehicle)
  if(CH_ENABLE_MODULE_IRRLICHT)
    find_package(Irrlicht QUIET REQUIRED)
    target_link_libraries(vehicle PRIVATE Chrono_vehicle_irrlicht Chrono_irrlicht Irrlicht::Irrlicht)
  endif()

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(vehicle PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(vehicle PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(vehicle PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(vehicle PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(vehicle PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(vehicle PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(vehicle PRIVATE "/wd4702")  # unreachable code
    target_compile_options(vehicle PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(vehicle PRIVATE "-Wno-unused-variable")
  endif()
  

  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(vehicle PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(vehicle PROPERTIES
                        PROJECT_LABEL "Chrono_python_vehicle"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(vehicle Chrono_core)
  add_dependencies(vehicle Chrono_vehicle)
  if(CH_ENABLE_MODULE_IRRLICHT)
    add_dependencies(vehicle Chrono_irrlicht)
  endif()
  add_dependencies(vehicle ChronoModels_vehicle)
  add_dependencies(vehicle core)

  # Copy PY file in a package dir
  set(CHPY_VEHICLE_PY_FILE "${CMAKE_SWIG_OUTDIR}/vehicle.py")

  set(CHPY_VEHICLE_PYD_PATH $<TARGET_FILE_DIR:vehicle>)

  add_custom_command(TARGET vehicle
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_VEHICLE_PYD_PATH}/pychrono)
  add_custom_command(TARGET vehicle
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_VEHICLE_PY_FILE} ${CHPY_VEHICLE_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS vehicle
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_VEHICLE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     vehicle")
    message("PY file:         ${CHPY_VEHICLE_PY_FILE}")
    message("PYD path:        ${CHPY_VEHICLE_PYD_PATH}")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the sensor python wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_SENSOR AND Python3_NumPy_FOUND)

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

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(sensor PROPERTIES
          PROJECT_LABEL "Chrono_python_sensor"
          _DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
          SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
          )

  add_dependencies(sensor Chrono_core)
  add_dependencies(sensor Chrono_sensor)
  add_dependencies(sensor core)

  # Copy PY file in a package dir
  set(CHPY_SENSOR_PY_FILE "${CMAKE_SWIG_OUTDIR}/sensor.py")

  set(CHPY_SENSOR_PYD_PATH $<TARGET_FILE_DIR:sensor>)

  add_custom_command(TARGET sensor
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_SENSOR_PYD_PATH}/pychrono)
  add_custom_command(TARGET sensor
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_SENSOR_PY_FILE} ${CHPY_SENSOR_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS sensor
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_SENSOR_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     sensor")
    message("PY file:         ${CHPY_SENSOR_PY_FILE}")
    message("PYD path:        ${CHPY_SENSOR_PYD_PATH}")
  endif()

else()
  message(WARNING "NumPy cannot be found. PyChrono::Sensor module will not be built.")
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

  swig_add_library(robot LANGUAGE python SOURCES ${CHPY_ROBOT_MODULE_FILE})
  target_link_libraries(robot PRIVATE Python3::Python Chrono_core ChronoModels_robot)

  IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    target_compile_definitions(robot PRIVATE "/DWNT")
  ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_compile_definitions(robot PRIVATE "-DHAVE_IOSTREAM")
    target_compile_definitions(robot PRIVATE "-DHAVE_LIMITS_H")
  ENDIF()

  # Disable additional warnings for generated C++ code TODO: are they all really needed for all targets?
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  elseif(MSVC)
    target_compile_options(robot PRIVATE "/wd4189")  # local variable is initialized but not referenced
    target_compile_options(robot PRIVATE "/wd4275")  # non-DLL-interface class used as base for DLL-interface class
    target_compile_options(robot PRIVATE "/wd4456")  # declaration hides previous local declaration
    target_compile_options(robot PRIVATE "/wd4702")  # unreachable code
    target_compile_options(robot PRIVATE "/wd4706")  # assignment within conditional expression
  else()
    target_compile_options(robot PRIVATE "-Wno-unused-variable")
  endif()


  #if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  #  set_target_properties(robot PROPERTIES SWIG_COMPILE_OPTIONS "-D_WIN32 -DWNT")
  #endif()
  
  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(robot PROPERTIES 
                        PROJECT_LABEL "Chrono_python_robot"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  add_dependencies(robot Chrono_core)
  add_dependencies(robot ChronoModels_robot)
  add_dependencies(robot core)

  # Copy PY file in a package dir 
  set(CHPY_VEHICLE_PY_FILE "${CMAKE_SWIG_OUTDIR}/robot.py")

  set(CHPY_VEHICLE_PYD_PATH $<TARGET_FILE_DIR:robot>)


  add_custom_command(TARGET robot
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_VEHICLE_PYD_PATH}/pychrono)
  add_custom_command(TARGET robot
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_VEHICLE_PY_FILE} ${CHPY_VEHICLE_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS robot
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_VEHICLE_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

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

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(parsers PROPERTIES
                        PROJECT_LABEL "Chrono_python_parsers"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(parsers Chrono_core)
  add_dependencies(parsers Chrono_parsers)
  add_dependencies(parsers core)

  # Copy PY file in a package dir
  set(CHPY_PARSERS_PY_FILE "${CMAKE_SWIG_OUTDIR}/parsers.py")

  set(CHPY_PARSERS_PYD_PATH $<TARGET_FILE_DIR:parsers>)

  add_custom_command(TARGET parsers
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_PARSERS_PYD_PATH}/pychrono)
  add_custom_command(TARGET parsers
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_PARSERS_PY_FILE} ${CHPY_PARSERS_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS parsers
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_PARSERS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     parsers")
    message("PY file:         ${CHPY_PARSERS_PY_FILE}")
    message("PYD file:        ${CHPY_PARSERS_PYD_FILE}")
    message("PYD path:        ${CHPY_PARSERS_PYD_PATH}")
  endif()

endif()


#-----------------------------------------------------------------------------
# MODULE for the ros python wrapper.
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

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(ros PROPERTIES
                        PROJECT_LABEL "Chrono_python_ros"
                        LIBRARY_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  

  add_dependencies(ros Chrono_core)
  add_dependencies(ros Chrono_ros)
  add_dependencies(ros core)

  # Copy PY file in a package dir
  set(CHPY_ROS_PY_FILE "${CMAKE_SWIG_OUTDIR}/ros.py")

  set(CHPY_ROS_PYD_PATH $<TARGET_FILE_DIR:ros>)

  add_custom_command(TARGET ros
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E make_directory ${CHPY_ROS_PYD_PATH}/pychrono)
  add_custom_command(TARGET ros
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${CHPY_ROS_PY_FILE} ${CHPY_ROS_PYD_PATH}/pychrono)

  # Install .pyd binary module (.so on linux)
  install(TARGETS ros
          EXPORT ChronoTargets
          RUNTIME DESTINATION "${CH_INSTALL_PYTHON}"
          LIBRARY DESTINATION "${CH_INSTALL_PYTHON}"
          ARCHIVE DESTINATION "${CH_INSTALL_PYTHON}")
  # Install .py module wrapper
  install(FILES "${CHPY_ROS_PY_FILE}" DESTINATION "${CH_INSTALL_PYTHON}/pychrono")

  if (DBG_SCRIPT)
    message("Module name:     ros")
    message("PY file:         ${CHPY_ROS_PY_FILE}")
    message("PYD file:        ${CHPY_ROS_PYD_FILE}")
    message("PYD path:        ${CHPY_ROS_PYD_PATH}")
  endif()

endif()

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
