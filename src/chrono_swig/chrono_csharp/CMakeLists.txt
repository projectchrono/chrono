#=============================================================================
# CMake configuration file for the Chrono CSharp module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_CSHARP "Enable the Chrono CSharp module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_CSHARP)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  return()
endif()

message(STATUS "\n==== Chrono CSharp module ====\n")

mark_as_advanced(CLEAR SWIG_EXECUTABLE)

# Optional ready-to-run wrapper packaging; when enabled we gather every SWIG
# .cs file and publish a single chrono_csharp_wrapper.dll for downstream use.
set(_chrono_build_wrapper_default OFF)
option(CH_USE_CSHARP_WRAPPER "Bundle Chrono C# sources into chrono_csharp_wrapper.dll" ${_chrono_build_wrapper_default})
set_property(CACHE CH_USE_CSHARP_WRAPPER PROPERTY HELPSTRING "Bundle Chrono SWIG-generated C# sources into chrono_csharp_wrapper.dll")
mark_as_advanced(CLEAR CH_USE_CSHARP_WRAPPER)

#-----------------------------------------------------------------------------
# Enable debugging CMake output
#-----------------------------------------------------------------------------

set(DBG_SCRIPT false)

#-----------------------------------------------------------------------------
# Find SWIG
#-----------------------------------------------------------------------------

if (POLICY CMP0122) # from 3.21 only
  cmake_policy(SET CMP0122 NEW)
endif()

message(STATUS "Find SWIG")
find_package(SWIG REQUIRED COMPONENTS csharp)

if(SWIG_FOUND)
  message(STATUS "  SWIG version:      ${SWIG_VERSION}")
  message(STATUS "  SWIG csharp found? ${SWIG_csharp_FOUND}")
  message(STATUS "  SWIG executable:   ${SWIG_EXECUTABLE}")
endif()

include(${SWIG_USE_FILE})

# Set location of SWIG-generated files.
set(CHRONO_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/chrono_csharp")

# Set SWIG flags.  Disable selected SWIG warnings 
# set(CMAKE_SWIG_FLAGS "-c++;-w401,503,833")

if(DBG_SCRIPT)
  message(STATUS "SWIG_USE_FILE:      ${SWIG_USE_FILE}")
  message(STATUS "CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")
endif()

#-----------------------------------------------------------------------------
# Set SWIG flags for optional modules
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)
  set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_VSG")
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
  set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_IRRLICHT")
endif()

if(CH_ENABLE_MODULE_FSI_SPH)
  set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_FSI_SPH")
endif()

if(CH_ENABLE_MODULE_PARSERS AND HAVE_URDF)
  set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_PARSERS_URDF")
endif()

if (CH_ENABLE_MODULE_SENSOR)
  set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_SENSOR")
endif()

if(CH_ENABLE_MODULE_VEHICLE AND HAVE_OPENCRG)
  set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DHAVE_OPENCRG")
endif()

#-----------------------------------------------------------------------------

set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

# Output library destination
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(LIB_OUT_DIR ${EXECUTABLE_OUTPUT_PATH})
else()
    set(LIB_OUT_DIR "${PROJECT_BINARY_DIR}/lib")
endif()

#-----------------------------------------------------------------------------
# Post build duplicate removal - for windows and unix (untested)
#-----------------------------------------------------------------------------
# Checks againt the binary directory (core) and removes duplicates in the glob directory (each module)
function(add_csharp_postbuild_command target_name glob_dir binary_dir)
    if(WIN32)
        # Windows-specific command
        add_custom_command(
            TARGET ${target_name} POST_BUILD
            #COMMAND ${CMAKE_COMMAND} -E echo "Removing duplicate .cs files from ${target_name} (Windows)"
            COMMAND for %%f in ("${binary_dir}\\*.cs") do (
                if exist "${glob_dir}\\%%~nxf" (
                    ${CMAKE_COMMAND} -E remove "${glob_dir}\\%%~nxf"
                )
            )
            COMMENT "Removed all detected duplicate .cs files wrapped from ${target_name}"
        )
    else()
        # Unix (possibly MacOS) specific command
        add_custom_command(
            TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "Removing duplicate .cs files from ${target_name} (Unix-like)"
            COMMAND for f in \"${binary_dir}\"/*.cs; do (
                if [ -f \"${glob_dir}\"/$(basename \"$f\") ]; then (
                    ${CMAKE_COMMAND} -E remove \"${glob_dir}\"/$(basename \"$f\")
                )
            ) done
            COMMENT "Removed all detected duplicate .cs files wrapped from ${target_name}"
        )
    endif()
endfunction()

#-----------------------------------------------------------------------------
# Wrapper for CORE Chrono module
#-----------------------------------------------------------------------------

message(STATUS "Add C# CORE module")

set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/core")

# Set interface file.
set(INTERFACE_FILE_CORE ChModuleCore_csharp.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
if(MSVC)
  set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES COMPILE_OPTIONS "-w833")
endif()
set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES CPLUSPLUS ON)

# Create the SWIG module.
swig_add_library(Chrono_csharp_core LANGUAGE csharp SOURCES ${INTERFACE_FILE_CORE})

target_link_libraries(Chrono_csharp_core PRIVATE Chrono_core)

set_target_properties(Chrono_csharp_core PROPERTIES 
                      PROJECT_LABEL "Chrono_csharp_core"
                      OUTPUT_NAME   "Chrono_csharp_core"
                      LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                      SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                      )

if(MSVC)
  # remove warning for unreachable code generated by SWIG
  target_compile_options(Chrono_csharp_core PRIVATE "/wd4702")
endif()

add_dependencies(Chrono_csharp_core Chrono_core)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS Chrono_csharp_core
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
else()
    install(TARGETS Chrono_csharp_core
            EXPORT ChronoTargets
            LIBRARY DESTINATION lib)
endif()

if(DBG_SCRIPT)
  message(STATUS "Module name:     Chrono_csharp_core")
  message(STATUS "SWIG_REAL_NAME:  Chrono_csharp_core")
endif()

#-----------------------------------------------------------------------------
# MODULE for the postprocess csharp wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_POSTPROCESS)

  message(STATUS "Add C# POSTPROCESS module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/postprocess")

  # Interface files
  set(INTERFACE_FILE_POSTPROCESS ChModulePostprocess_csharp.i)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_POSTPROCESS} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_POSTPROCESS} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${INTERFACE_FILE_POSTPROCESS} PROPERTIES COMPILE_OPTIONS "-w401,503")
  endif()

  # Create the SWIG module.
  swig_add_library(Chrono_csharp_postprocess LANGUAGE csharp SOURCES ${INTERFACE_FILE_POSTPROCESS})
  target_link_libraries(Chrono_csharp_postprocess PRIVATE Chrono_core Chrono_postprocess)



  # Ensure that the .cs library files are generated in the bin/ directory.
  set_target_properties(Chrono_csharp_postprocess PROPERTIES
                        PROJECT_LABEL "Chrono_csharp_postprocess"
                        OUTPUT_NAME   "Chrono_csharp_postprocess"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(Chrono_csharp_postprocess PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(Chrono_csharp_postprocess PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(Chrono_csharp_postprocess Chrono_core)
  add_dependencies(Chrono_csharp_postprocess Chrono_postprocess)
  add_dependencies(Chrono_csharp_postprocess Chrono_csharp_core)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS Chrono_csharp_postprocess
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
  else()
    install(TARGETS Chrono_csharp_postprocess
            EXPORT ChronoTargets
            LIBRARY DESTINATION lib)
  endif()

  if (DBG_SCRIPT)
    message("Module name:     Chrono_csharp_postprocess")
    message("SWIG_REAL_NAME:  Chrono_csharp_postprocess")
  endif()

#------------------------------------------------------Remove duplicates
#Uncomment for use
#    # Custom command to remove duplicate .cs files from the vehicle directory and log the actions
#    set(GLOB_DIR ${CMAKE_SWIG_OUTDIR})
#    set(BINARY_DIR ${PROJECT_BINARY_DIR}/chrono_csharp/core)
#
#    # Call the function for the vehicle module
#    add_csharp_postbuild_command(Chrono_csharp_postprocess ${GLOB_DIR} ${BINARY_DIR})
#
#------------------------------------------------------Remove duplicates

endif()

#-----------------------------------------------------------------------------
# MODULE for the irrlicht csharp wrapper.
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_IRRLICHT)

  message(STATUS "Add C# IRRLICHT module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/irrlicht")

  # Interface files
  set(INTERFACE_FILE_IRRLICHT ChModuleIrrlicht_csharp.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_IRRLICHT} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_IRRLICHT} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${INTERFACE_FILE_IRRLICHT} PROPERTIES COMPILE_OPTIONS "-w401,503")
  endif()

  # Create the SWIG module.
  swig_add_library(Chrono_csharp_irrlicht LANGUAGE csharp SOURCES ${INTERFACE_FILE_IRRLICHT})
  find_package(Irrlicht QUIET REQUIRED)
  target_link_libraries(Chrono_csharp_irrlicht PRIVATE Chrono_core Chrono_irrlicht Irrlicht::Irrlicht)


  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(Chrono_csharp_irrlicht PROPERTIES
                        PROJECT_LABEL "Chrono_csharp_irrlicht"
                        OUTPUT_NAME   "Chrono_csharp_irrlicht"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(Chrono_csharp_irrlicht PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(Chrono_csharp_irrlicht PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(Chrono_csharp_irrlicht Chrono_core)
  add_dependencies(Chrono_csharp_irrlicht Chrono_irrlicht)
  add_dependencies(Chrono_csharp_irrlicht Chrono_csharp_core)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS Chrono_csharp_irrlicht
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
  else()
    install(TARGETS Chrono_csharp_irrlicht
            EXPORT ChronoTargets
            LIBRARY DESTINATION lib)
  endif()

  if (DBG_SCRIPT)
    message("Module name:     Chrono_csharp_irrlicht")
    message("SWIG_REAL_NAME:  Chrono_csharp_irrlicht")
  endif()

endif()

#-----------------------------------------------------------------------------
# MODULE for the vsg csharp wrapper
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VSG)

  message(STATUS "Add C# VSG module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/vsg")

  # Interface files
  set(INTERFACE_FILE_VSG ChModuleVsg_csharp.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_VSG} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_VSG} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${INTERFACE_FILE_VSG} PROPERTIES COMPILE_OPTIONS "-w401,503")
  endif()

  # Create the SWIG module
  swig_add_library(Chrono_csharp_vsg LANGUAGE csharp SOURCES ${INTERFACE_FILE_VSG})
  target_link_libraries(Chrono_csharp_vsg PRIVATE Chrono_core Chrono_vsg)

  # Ensure that the library file is generated in the bin/ directory
  set_target_properties(Chrono_csharp_vsg PROPERTIES
                        PROJECT_LABEL "Chrono_csharp_vsg"
                        OUTPUT_NAME   "Chrono_csharp_vsg"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(Chrono_csharp_vsg PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(Chrono_csharp_vsg PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(Chrono_csharp_vsg Chrono_core)
  add_dependencies(Chrono_csharp_vsg Chrono_vsg)
  add_dependencies(Chrono_csharp_vsg Chrono_csharp_core)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS Chrono_csharp_vsg
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
  else()
    install(TARGETS Chrono_csharp_vsg
            EXPORT ChronoTargets
            LIBRARY DESTINATION lib)
  endif()

  if (DBG_SCRIPT)
    message("Module name:     Chrono_csharp_vsg")
    message("SWIG_REAL_NAME:  Chrono_csharp_vsg")
  endif()

endif()

#-----------------------------------------------------------------------------
# Wrapper for VEHICLE Chrono module
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)

  message(STATUS "Add C# VEHICLE module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/vehicle")

  # Set interface file.
  set(INTERFACE_FILE_VEHICLE ChModuleVehicle_csharp.i)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES COMPILE_OPTIONS "-w401,503,833")
  endif()
  
  # Create the SWIG module.
  swig_add_library(Chrono_csharp_vehicle LANGUAGE csharp SOURCES ${INTERFACE_FILE_VEHICLE})
  target_link_libraries(Chrono_csharp_vehicle PRIVATE Chrono_core Chrono_vehicle ChronoModels_vehicle)

  if(CH_ENABLE_MODULE_IRRLICHT)
    find_package(Irrlicht QUIET REQUIRED)
    target_link_libraries(Chrono_csharp_vehicle PRIVATE Chrono_irrlicht Chrono_vehicle_irrlicht Irrlicht::Irrlicht)
    add_dependencies(Chrono_csharp_vehicle Chrono_csharp_irrlicht)
  endif()
  
  if(CH_ENABLE_MODULE_VSG)
      target_link_libraries(Chrono_csharp_vehicle PRIVATE Chrono_vehicle_vsg Chrono_vsg)
      add_dependencies(Chrono_csharp_vehicle Chrono_csharp_vsg)
  endif()

  if(CH_ENABLE_MODULE_POSTPROCESS)
     target_link_libraries(Chrono_csharp_vehicle PRIVATE Chrono_postprocess)
    add_dependencies(Chrono_csharp_vehicle Chrono_csharp_postprocess)
  endif()
  
  if(CH_ENABLE_MODULE_FSI_SPH)
    target_link_libraries(Chrono_csharp_vehicle PRIVATE Chrono_fsi Chrono_fsisph)
  endif()

  set_target_properties(Chrono_csharp_vehicle PROPERTIES 
                        PROJECT_LABEL "Chrono_csharp_vehicle"
                        OUTPUT_NAME   "Chrono_csharp_vehicle"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )
  
  target_compile_definitions(Chrono_csharp_vehicle PRIVATE "CH_IGNORE_DEPRECATED")
  if (HAVE_OPENCRG)
    target_compile_definitions(Chrono_csharp_vehicle PRIVATE "HAVE_OPENCRG")
  endif()
  
  add_dependencies(Chrono_csharp_vehicle Chrono_core Chrono_vehicle ChronoModels_vehicle)

  add_dependencies(Chrono_csharp_vehicle Chrono_csharp_core)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS Chrono_csharp_vehicle
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
  else()
    install(TARGETS Chrono_csharp_vehicle
            EXPORT ChronoTargets
            LIBRARY DESTINATION lib)
  endif()
  
  if (DBG_SCRIPT)
    message(STATUS "Module name:     Chrono_csharp_vehicle")
    message(STATUS "SWIG_REAL_NAME:  Chrono_csharp_vehicle")
  endif()

#------------------------------------------------------Remove duplicates
#Uncomment for use
#    # Custom command to remove duplicate .cs files from the vehicle directory and log the actions
#    set(GLOB_DIR ${CMAKE_SWIG_OUTDIR})
#    set(BINARY_DIR ${PROJECT_BINARY_DIR}/chrono_csharp/core)
#
#    # Call the function for the vehicle module
#    add_csharp_postbuild_command(Chrono_csharp_vehicle ${GLOB_DIR} ${BINARY_DIR})
#
#------------------------------------------------------Remove duplicates

endif()

#-----------------------------------------------------------------------------
# MODULE for the sensor csharp wrapper
#-----------------------------------------------------------------------------

if(CH_ENABLE_MODULE_SENSOR)

  message(STATUS "Add C# SENSOR module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/sensor")

  # Interface files
  set(INTERFACE_FILE_SENSOR ChModuleSensor_csharp.i)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_SENSOR} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_SENSOR} PROPERTIES CPLUSPLUS ON)
  if(MSVC)
    set_source_files_properties(${INTERFACE_FILE_SENSOR} PROPERTIES COMPILE_OPTIONS "-w401,503")
  endif()

  # Create the SWIG module
  swig_add_library(Chrono_csharp_sensor LANGUAGE csharp SOURCES ${INTERFACE_FILE_SENSOR})
  target_link_libraries(Chrono_csharp_sensor PRIVATE Chrono_core Chrono_sensor)

  # Ensure that the library file is generated in the bin/ directory
  set_target_properties(Chrono_csharp_sensor PROPERTIES
                        PROJECT_LABEL "Chrono_csharp_sensor"
                        OUTPUT_NAME   "Chrono_csharp_sensor"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                        )

  target_compile_definitions(Chrono_csharp_sensor PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(Chrono_csharp_sensor PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(Chrono_csharp_sensor Chrono_core)
  add_dependencies(Chrono_csharp_sensor Chrono_sensor)
  add_dependencies(Chrono_csharp_sensor Chrono_csharp_core)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS Chrono_csharp_sensor
            EXPORT ChronoTargets
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
  else()
    install(TARGETS Chrono_csharp_sensor
            EXPORT ChronoTargets
            LIBRARY DESTINATION lib)
  endif()

  if (DBG_SCRIPT)
    message("Module name:     Chrono_csharp_sensor")
    message("SWIG_REAL_NAME:  Chrono_csharp_sensor")
  endif()

endif()

#-----------------------------------------------------------------------------
# Module for the robot models
#-----------------------------------------------------------------------------

message(STATUS "Add C# ROBOT module")

set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/robot")

# Interface files
set(INTERFACE_FILE_ROBOT ChModuleRobot_csharp.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${INTERFACE_FILE_ROBOT} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${INTERFACE_FILE_ROBOT} PROPERTIES CPLUSPLUS ON)
if(MSVC)
  set_source_files_properties(${INTERFACE_FILE_ROBOT} PROPERTIES COMPILE_OPTIONS "-w401,503")
endif()

# Create the SWIG module
swig_add_library(Chrono_csharp_robot LANGUAGE csharp SOURCES ${INTERFACE_FILE_ROBOT})
target_link_libraries(Chrono_csharp_robot PRIVATE Chrono_core ChronoModels_robot)

# Ensure that the library file is generated in the bin/ directory
set_target_properties(Chrono_csharp_robot PROPERTIES
                      PROJECT_LABEL "Chrono_csharp_robot"
                      OUTPUT_NAME   "Chrono_csharp_robot"
                      LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                      SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE
                      )

target_compile_definitions(Chrono_csharp_robot PRIVATE "CH_IGNORE_DEPRECATED")
target_compile_definitions(Chrono_csharp_robot PRIVATE "IGNORE_DEPRECATED_WARNING")

add_dependencies(Chrono_csharp_robot Chrono_core)
add_dependencies(Chrono_csharp_robot ChronoModels_robot)
add_dependencies(Chrono_csharp_robot Chrono_csharp_core)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  install(TARGETS Chrono_csharp_robot
          EXPORT ChronoTargets
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
else()
  install(TARGETS Chrono_csharp_robot
          EXPORT ChronoTargets
          LIBRARY DESTINATION lib)
endif()

if (DBG_SCRIPT)
  message("Module name:     Chrono_csharp_robot")
  message("SWIG_REAL_NAME:  Chrono_csharp_robot")
endif()


#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Swig generated files
install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/core/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")

if(CH_ENABLE_MODULE_POSTPROCESS)
install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/postprocess/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/irrlicht/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")
endif()

if(CH_ENABLE_MODULE_VSG)
install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/vsg/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")
endif()

if(CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/vehicle/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")
endif()

if(CH_ENABLE_MODULE_SENSOR)
install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/sensor/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")
endif()

install(DIRECTORY ${CHRONO_SWIG_OUTDIR}/robot/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")


# Template project
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project_csharp/ DESTINATION "template_project_csharp")
else()
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/template_project_csharp/ DESTINATION "share/chrono/template_project_csharp")
endif()

#=============================================================================
# function for chrono_csharp_wrapper which grabs all generated .cs files and
# bundles into a single managed assembly dll (depeding on user selection in
# cmake build).
#=============================================================================

if(CH_USE_CSHARP_WRAPPER)
  function(chrono_generate_csharp_collect_script output_path)
    file(WRITE "${output_path}" "# Auto-generated by Chrono's C# module\n")
    file(APPEND "${output_path}" "# Copy SWIG-generated C# sources into the assembly staging folder\n\n")
    file(APPEND "${output_path}" "if(NOT DEFINED ASSEMBLY_SOURCE_DIR)\n    message(FATAL_ERROR \"ASSEMBLY_SOURCE_DIR was not provided\")\nendif()\n")
    file(APPEND "${output_path}" "if(NOT DEFINED CORE_SOURCE_DIR)\n    message(FATAL_ERROR \"CORE_SOURCE_DIR was not provided\")\nendif()\n\n")
    file(APPEND "${output_path}" "file(REMOVE_RECURSE \"\${ASSEMBLY_SOURCE_DIR}\")\n")
    file(APPEND "${output_path}" "file(MAKE_DIRECTORY \"\${ASSEMBLY_SOURCE_DIR}\")\n\n")
    file(APPEND "${output_path}" "file(GLOB _core_files \"\${CORE_SOURCE_DIR}/*.cs\")\n")
    file(APPEND "${output_path}" "foreach(_file IN LISTS _core_files)\n    if(EXISTS \"\${_file}\")\n        file(COPY \"\${_file}\" DESTINATION \"\${ASSEMBLY_SOURCE_DIR}\")\n    endif()\nendforeach()\n\n")
    file(APPEND "${output_path}" "foreach(_module_dir IN LISTS MODULE_SOURCE_DIRS)\n    if(NOT EXISTS \"\${_module_dir}\")\n        continue()\n    endif()\n    file(GLOB _module_files \"\${_module_dir}/*.cs\")\n    foreach(_file IN LISTS _module_files)\n        get_filename_component(_filename \"\${_file}\" NAME)\n        if(EXISTS \"\${ASSEMBLY_SOURCE_DIR}/\${_filename}\")\n            continue()\n        endif()\n        file(COPY \"\${_file}\" DESTINATION \"\${ASSEMBLY_SOURCE_DIR}\")\n    endforeach()\nendforeach()\n")
  endfunction()

  find_program(DOTNET_EXE dotnet)
  if(NOT DOTNET_EXE)
    message(FATAL_ERROR "dotnet CLI not found. chrono_csharp_wrapper.dll requires the .NET CLI to publish managed assemblies.")
  endif()

  set(_wrapper_rid_os "")
  if(WIN32)
    set(_wrapper_rid_os "win")
  elseif(APPLE)
    set(_wrapper_rid_os "osx")
  elseif(UNIX)
    set(_wrapper_rid_os "linux")
  endif()

  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _wrapper_proc)
  set(_wrapper_rid_arch "")
  if(_wrapper_proc MATCHES "amd64" OR _wrapper_proc MATCHES "x86_64")
    set(_wrapper_rid_arch "x64")
  elseif(_wrapper_proc MATCHES "aarch64" OR _wrapper_proc MATCHES "arm64")
    set(_wrapper_rid_arch "arm64")
  elseif(_wrapper_proc MATCHES "^i[3-6]86" OR _wrapper_proc STREQUAL "x86")
    set(_wrapper_rid_arch "x86")
  endif()

  set(_wrapper_rid_default "")
  if(_wrapper_rid_os AND _wrapper_rid_arch)
    set(_wrapper_rid_default "${_wrapper_rid_os}-${_wrapper_rid_arch}")
  endif()

  set(CHRONO_CSHARP_WRAPPER_RUNTIME_ID "${_wrapper_rid_default}" CACHE STRING "Runtime identifier used when publishing chrono_csharp_wrapper.dll")
  mark_as_advanced(FORCE CHRONO_CSHARP_WRAPPER_RUNTIME_ID)

  if(NOT CHRONO_CSHARP_WRAPPER_RUNTIME_ID)
    message(FATAL_ERROR "Unable to determine a runtime identifier for chrono_csharp_wrapper.dll. Set CHRONO_CSHARP_WRAPPER_RUNTIME_ID manually.")
  endif()

  set(CHRONO_CSHARP_WRAPPER_TFM "net472")
  set(CHRONO_CSHARP_WRAPPER_ROOT "${PROJECT_BINARY_DIR}/chrono_csharp/wrapper")
  set(CHRONO_CSHARP_WRAPPER_SOURCE_DIR "${CHRONO_CSHARP_WRAPPER_ROOT}/sources")
  set(CHRONO_CSHARP_WRAPPER_PROJECT "${CHRONO_CSHARP_WRAPPER_ROOT}/chrono_csharp_wrapper.csproj")
  if(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CHRONO_CSHARP_WRAPPER_OUTPUT_ROOT "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
  else()
    set(CHRONO_CSHARP_WRAPPER_OUTPUT_ROOT "${PROJECT_BINARY_DIR}/bin")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/chrono_csharp_wrapper.csproj.in ${CHRONO_CSHARP_WRAPPER_PROJECT} @ONLY)

  set(CHRONO_CSHARP_CORE_SOURCE_DIR "${PROJECT_BINARY_DIR}/chrono_csharp/core")
  set(CHRONO_CSHARP_MODULE_SOURCE_DIRS)
  list(APPEND CHRONO_CSHARP_MODULE_SOURCE_DIRS "${PROJECT_BINARY_DIR}/chrono_csharp/robot")
  if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND CHRONO_CSHARP_MODULE_SOURCE_DIRS "${PROJECT_BINARY_DIR}/chrono_csharp/irrlicht")
  endif()
  if(CH_ENABLE_MODULE_VSG)
    list(APPEND CHRONO_CSHARP_MODULE_SOURCE_DIRS "${PROJECT_BINARY_DIR}/chrono_csharp/vsg")
  endif()
  if(CH_ENABLE_MODULE_POSTPROCESS)
    list(APPEND CHRONO_CSHARP_MODULE_SOURCE_DIRS "${PROJECT_BINARY_DIR}/chrono_csharp/postprocess")
  endif()
  if(CH_ENABLE_MODULE_VEHICLE)
    list(APPEND CHRONO_CSHARP_MODULE_SOURCE_DIRS "${PROJECT_BINARY_DIR}/chrono_csharp/vehicle")
  endif()
  if(CH_ENABLE_MODULE_SENSOR)
    list(APPEND CHRONO_CSHARP_MODULE_SOURCE_DIRS "${PROJECT_BINARY_DIR}/chrono_csharp/sensor")
  endif()

  set(_chrono_csharp_module_dirs_arg "")
  if(CHRONO_CSHARP_MODULE_SOURCE_DIRS)
    string(REPLACE ";" "\\;" _chrono_csharp_module_dirs_arg "${CHRONO_CSHARP_MODULE_SOURCE_DIRS}")
  endif()

  set(CHRONO_CSHARP_COLLECT_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/ChronoCSharpCollectSources.cmake")
  chrono_generate_csharp_collect_script("${CHRONO_CSHARP_COLLECT_SCRIPT}")

  # Collect SWIG .cs files and publish the managed wrapper so downstream only needs the chrono_csharp_wrapper.dll
  # on their reference path along with the native chrono_csharp module dll and standard chrono dlls
  add_custom_target(Chrono_csharp_wrapper ALL
    COMMAND ${CMAKE_COMMAND}
            -DASSEMBLY_SOURCE_DIR=${CHRONO_CSHARP_WRAPPER_SOURCE_DIR}
            -DCORE_SOURCE_DIR=${CHRONO_CSHARP_CORE_SOURCE_DIR}
            -DMODULE_SOURCE_DIRS=${_chrono_csharp_module_dirs_arg}
            -P ${CHRONO_CSHARP_COLLECT_SCRIPT}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CHRONO_CSHARP_WRAPPER_OUTPUT_ROOT}/$<CONFIG>"
    COMMAND ${DOTNET_EXE} publish ${CHRONO_CSHARP_WRAPPER_PROJECT}
            --configuration $<CONFIG>
            --runtime ${CHRONO_CSHARP_WRAPPER_RUNTIME_ID}
            --output "${CHRONO_CSHARP_WRAPPER_OUTPUT_ROOT}/$<CONFIG>"
            --nologo
    COMMENT "Publishing chrono_csharp_wrapper.dll ($<CONFIG>)"
    VERBATIM)

  add_dependencies(Chrono_csharp_wrapper Chrono_csharp_core)
  add_dependencies(Chrono_csharp_wrapper Chrono_csharp_robot)
  if(CH_ENABLE_MODULE_POSTPROCESS)
    add_dependencies(Chrono_csharp_wrapper Chrono_csharp_postprocess)
  endif()
  if(CH_ENABLE_MODULE_IRRLICHT)
    add_dependencies(Chrono_csharp_wrapper Chrono_csharp_irrlicht)
  endif()
  if(CH_ENABLE_MODULE_VSG)
    add_dependencies(Chrono_csharp_wrapper Chrono_csharp_vsg)
  endif()
  if(CH_ENABLE_MODULE_VEHICLE)
    add_dependencies(Chrono_csharp_wrapper Chrono_csharp_vehicle)
  endif()
  if(CH_ENABLE_MODULE_SENSOR)
    add_dependencies(Chrono_csharp_wrapper Chrono_csharp_sensor)
  endif()

  if(WIN32)
    set(_chrono_csharp_wrapper_install_dir bin)
  else()
    set(_chrono_csharp_wrapper_install_dir lib)
  endif()
  install(FILES "${CHRONO_CSHARP_WRAPPER_OUTPUT_ROOT}/$<CONFIG>/chrono_csharp_wrapper.dll"
          DESTINATION ${_chrono_csharp_wrapper_install_dir}
          CONFIGURATIONS Debug Release RelWithDebInfo MinSizeRel)
endif()
