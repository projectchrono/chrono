#=============================================================================
# CMake configuration file for the Chrono CSharp module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_CSHARP "Enable the Chrono CSharp module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_CSHARP)
  mark_as_advanced(FORCE SWIG_EXECUTABLE)
  return()
endif()

message(STATUS "\n==== Chrono CSharp module ====\n")

cmake_minimum_required(VERSION 3.8.0)
mark_as_advanced(CLEAR SWIG_EXECUTABLE)

#-----------------------------------------------------------------------------
# CMake policy settings
# TODO: switch to new behavior
#-----------------------------------------------------------------------------

# Fix warnings about the use of SWIG_MODULE_${name}_REAL_NAME with CMake version >= 3.13
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
# Find SWIG
#-----------------------------------------------------------------------------

message(STATUS "...find SWIG")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

# Add /bigobj flag to deal with large generated .cxx wrappers
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS}   /bigobj")
endif()

# Set location of SWIG-generated files.
set(CHRONO_SWIG_OUTDIR "${PROJECT_BINARY_DIR}/chrono_csharp")

# Set SWIG flags.  Disable selected SWIG warnings 
set(CMAKE_SWIG_FLAGS "-c++;-w401,503,833")

if(DBG_SCRIPT)
  message(STATUS "SWIG_USE_FILE:      ${SWIG_USE_FILE}")
  message(STATUS "CMAKE_SWIG_OUTDIR:  ${CMAKE_SWIG_OUTDIR}")
endif()

# Disable additional warnings for generated C++ code
if(MSVC)
   add_compile_options(/wd4189)  # local variable is initialized but not referenced
   add_compile_options(/wd4456)  # declaration hides previous local declaration
   add_compile_options(/wd4702)  # unreachable code
   add_compile_options(/wd4706)  # assignment within conditional expression
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

message(STATUS "...add CSharp CORE module")

# Set module name.
set(CH_CSHARP_CORE ChronoEngine_csharp_core)

set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/core")

# Set interface file.
set(INTERFACE_FILE_CORE ChModuleCore_csharp.i)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
endif()
set_source_files_properties(${INTERFACE_FILE_CORE} PROPERTIES CPLUSPLUS ON)
##set_source_files_properties(${CHPY_CORE_WRAPPER_FILES} PROPERTIES HEADER_FILE_ONLY ON)
##source_group("wrappers" FILES  ${CHPY_CORE_WRAPPER_FILES})

# Create the SWIG module.
SWIG_ADD_LIBRARY(${CH_CSHARP_CORE} LANGUAGE csharp SOURCES ${INTERFACE_FILE_CORE})
SWIG_LINK_LIBRARIES(${CH_CSHARP_CORE} ChronoEngine)

set_target_properties(${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME} PROPERTIES 
                      PROJECT_LABEL "${CH_CSHARP_CORE}"
                      OUTPUT_NAME   "${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME}"
                      LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                      )

target_compile_definitions(${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")

add_dependencies(${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME} ChronoEngine)

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS ${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME}
            LIBRARY DESTINATION bin)
else()
    install(TARGETS ${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME}
            LIBRARY DESTINATION lib)
endif()

if(DBG_SCRIPT)
  message(STATUS "Module name:     ${CH_CSHARP_CORE}")
  message(STATUS "SWIG_REAL_NAME:  ${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME}")
endif()

#-----------------------------------------------------------------------------
# MODULE for the postprocess csharp wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_POSTPROCESS)

  message(STATUS "...add CSharp POSTPROCESS module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/postprocess")

  # Module name
  set(CH_CSHARP_POSTPROCESS ChronoEngine_csharp_postprocess)

  # Interface files
  set(CH_CSHARP_POSTPROCESS_MODULE_FILE ChModulePostprocess_csharp.i)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CH_CSHARP_POSTPROCESS_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CH_CSHARP_POSTPROCESS_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  SWIG_ADD_LIBRARY(${CH_CSHARP_POSTPROCESS} LANGUAGE csharp SOURCES ${CH_CSHARP_POSTPROCESS_MODULE_FILE})
  SWIG_LINK_LIBRARIES(${CH_CSHARP_POSTPROCESS} ChronoEngine ChronoEngine_postprocess)

  # Ensure that the .cs library files are generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} PROPERTIES
                        PROJECT_LABEL "${CH_CSHARP_POSTPROCESS}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        )

  target_compile_definitions(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} ChronoEngine)
  add_dependencies(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} ChronoEngine_postprocess)
  add_dependencies(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} ${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME})

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS ${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME}
            LIBRARY DESTINATION bin)
  else()
      install(TARGETS ${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME}
              LIBRARY DESTINATION lib)
  endif()

  if (DBG_SCRIPT)
    message("Module name:     ${CH_CSHARP_POSTPROCESS}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME}")
  endif()

#------------------------------------------------------Remove duplicates
#Uncomment for use
#    # Custom command to remove duplicate .cs files from the vehicle directory and log the actions
#    set(GLOB_DIR ${CMAKE_SWIG_OUTDIR})
#    set(BINARY_DIR ${PROJECT_BINARY_DIR}/chrono_csharp/core)
#
#    # Call the function for the vehicle module
#    add_csharp_postbuild_command(${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME} ${GLOB_DIR} ${BINARY_DIR})
#
#------------------------------------------------------Remove duplicates

endif()

#-----------------------------------------------------------------------------
# MODULE for the irrlicht csharp wrapper.
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_IRRLICHT)

  message(STATUS "...add csharp IRRLICHT module")

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/irrlicht")

  # Python module name
  set(CH_CSHARP_IRRLICHT ChronoEngine_csharp_irrlicht)

  # Interface files
  set(CH_CSHARP_IRRLICHT_MODULE_FILE ChModuleIrrlicht_csharp.i)

  include_directories(${CH_IRRLICHT_INCLUDES})

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${CH_CSHARP_IRRLICHT_MODULE_FILE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${CH_CSHARP_IRRLICHT_MODULE_FILE} PROPERTIES CPLUSPLUS ON)

  # Create the SWIG module.
  SWIG_ADD_LIBRARY(${CH_CSHARP_IRRLICHT} LANGUAGE csharp SOURCES ${CH_CSHARP_IRRLICHT_MODULE_FILE})
  SWIG_LINK_LIBRARIES(${CH_CSHARP_IRRLICHT} ChronoEngine ChronoEngine_irrlicht)

  # Ensure that the PYD library file is generated in the bin/ directory.
  set_target_properties(${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME} PROPERTIES
                        PROJECT_LABEL "${CH_CSHARP_IRRLICHT}"
                        OUTPUT_NAME   "${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        )

  target_compile_definitions(${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")
  target_compile_definitions(${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME} PRIVATE "IGNORE_DEPRECATED_WARNING")

  add_dependencies(${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME} ChronoEngine)
  add_dependencies(${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME} ChronoEngine_irrlicht)
  add_dependencies(${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME} ${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME})

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS ${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME}
            LIBRARY DESTINATION bin)
  else()
      install(TARGETS ${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME}
              LIBRARY DESTINATION lib)
  endif()

  if (DBG_SCRIPT)
    message("Module name:     ${CH_CSHARP_IRRLICHT}")
    message("SWIG_REAL_NAME:  ${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME}")
  endif()

endif()


#-----------------------------------------------------------------------------
# Wrapper for VEHICLE Chrono module
#-----------------------------------------------------------------------------

if(ENABLE_MODULE_VEHICLE AND ENABLE_MODULE_VEHICLE_MODELS)

  if (ENABLE_MODULE_IRRLICHT)
     #Append the CHRONO_IRRLICHT flag, for vehicle visual irrlicht
     set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DCHRONO_IRRLICHT")
  endif()

  if (HAVE_OPENCRG)
     set(CMAKE_SWIG_FLAGS "${CMAKE_SWIG_FLAGS};-DHAVE_OPENCRG")
  endif()

  message(STATUS "...add CSharp VEHICLE module")
  
  # Set module name.
  set(MODULE_VEHICLE ChronoEngine_csharp_vehicle)

  set(CMAKE_SWIG_OUTDIR "${CHRONO_SWIG_OUTDIR}/vehicle")

  # Set interface file.
  set(INTERFACE_FILE_VEHICLE ChModuleVehicle_csharp.i)
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES COMPILE_FLAGS "-D_WIN32")
  endif()
  set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES CPLUSPLUS ON)

  # Apply custom SWIG flags
  set_source_files_properties(${INTERFACE_FILE_VEHICLE} PROPERTIES COMPILE_FLAGS "${CMAKE_SWIG_FLAGS}")
  
  # Create the SWIG module.
  SWIG_ADD_LIBRARY(${MODULE_VEHICLE} LANGUAGE csharp SOURCES ${INTERFACE_FILE_VEHICLE})
  SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoEngine)
  SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoEngine_vehicle)
  SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoModels_vehicle)

  if (ENABLE_MODULE_IRRLICHT)
     SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoEngine_irrlicht ChronoEngine_vehicle_irrlicht)
  endif()

  if (ENABLE_MODULE_POSTPROCESS)
     SWIG_LINK_LIBRARIES(${MODULE_VEHICLE} ChronoEngine_postprocess)
  endif()
  
  set_target_properties(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} PROPERTIES 
                        PROJECT_LABEL "${MODULE_VEHICLE}"
                        OUTPUT_NAME   "${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${LIB_OUT_DIR}"
                        )
  
  target_compile_definitions(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} PRIVATE "CH_IGNORE_DEPRECATED")
  if (HAVE_OPENCRG)
    target_compile_definitions(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} PRIVATE "HAVE_OPENCRG")
  endif()
  
  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ChronoEngine)
  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ChronoEngine_vehicle)
  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ChronoModels_vehicle)

  ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ${SWIG_MODULE_${CH_CSHARP_CORE}_REAL_NAME})
  if (ENABLE_MODULE_IRRLICHT)
    ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ${SWIG_MODULE_${CH_CSHARP_IRRLICHT}_REAL_NAME})
  endif()
  if (ENABLE_MODULE_POSTPROCESS)
    ADD_DEPENDENCIES(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ${SWIG_MODULE_${CH_CSHARP_POSTPROCESS}_REAL_NAME})
  endif()
  
  
  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      install(TARGETS ChronoEngine_csharp_vehicle
              LIBRARY DESTINATION bin)
  else()
      install(TARGETS ChronoEngine_csharp_vehicle
              LIBRARY DESTINATION lib)
  endif()
  

  if (DBG_SCRIPT)
    message(STATUS "Module name:     ${MODULE_VEHICLE}")
    message(STATUS "SWIG_REAL_NAME:  ${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME}")
  endif()

#------------------------------------------------------Remove duplicates
#Uncomment for use
#    # Custom command to remove duplicate .cs files from the vehicle directory and log the actions
#    set(GLOB_DIR ${CMAKE_SWIG_OUTDIR})
#    set(BINARY_DIR ${PROJECT_BINARY_DIR}/chrono_csharp/core)
#
#    # Call the function for the vehicle module
#    add_csharp_postbuild_command(${SWIG_MODULE_${MODULE_VEHICLE}_REAL_NAME} ${GLOB_DIR} ${BINARY_DIR})
#
#------------------------------------------------------Remove duplicates

endif()

#-----------------------------------------------------------------------------
# Install swig-generate C# files

install(DIRECTORY ${CMAKE_SWIG_OUTDIR}/
        DESTINATION include/chrono_csharp
        FILES_MATCHING PATTERN "*.cs")


