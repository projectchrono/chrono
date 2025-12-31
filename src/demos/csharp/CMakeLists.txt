# ----------------------------------------------------------------------

function(FindDuplicates out_list list1 list2)
  # Union of list1 and list2
  list(APPEND union ${list1} ${list2})
  list(REMOVE_DUPLICATES union)

  # Difference 1 - 2
  list(APPEND diff_12 ${list1})
  list(REMOVE_ITEM diff_12 ${list2})

  # Difference 2 -1
  list(APPEND diff_21 ${list2})
  list(REMOVE_ITEM diff_21 ${list1})

  # Calculate union of (1-2) and (2-1)
  list(APPEND delta ${diff_12} ${diff_21})
  list(REMOVE_DUPLICATES delta)

  # Calculate the intersection as union-delta
  list(APPEND intersection ${union})
  list(REMOVE_ITEM intersection ${delta})

  set(${out_list} "${intersection}" PARENT_SCOPE)
endfunction()

# ----------------------------------------------------------------------

function(CollectCSharpFiles out_list module_list)
  set(file_list)
  set(tmp_list)

  # Core module
  list (FIND module_list "core" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_CORE_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/core")
    file(GLOB Chrono_CSHARP_CORE_SOURCES 
         LIST_DIRECTORIES false 
         RELATIVE "${Chrono_CSHARP_CORE_DIR}" 
         CONFIGURE_DEPENDS 
         "${Chrono_CSHARP_CORE_DIR}/*.cs")
    foreach(CS ${Chrono_CSHARP_CORE_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_CORE_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach() 
  endif()
  
  # Irrlict module
  list (FIND module_list "irrlicht" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_IRRLICHT_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/irrlicht")
    file(GLOB Chrono_CSHARP_IRRLICHT_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_IRRLICHT_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_IRRLICHT_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_IRRLICHT_DUPLICATES "${tmp_list}"
                                                     "${Chrono_CSHARP_IRRLICHT_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_IRRLICHT_SOURCES ${Chrono_CSHARP_IRRLICHT_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_IRRLICHT_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_IRRLICHT_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach()
  endif()
  
  # VSG module
  list (FIND module_list "vsg" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_VSG_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/vsg")
    file(GLOB Chrono_CSHARP_VSG_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_VSG_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_VSG_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_VSG_DUPLICATES "${tmp_list}"
                                                "${Chrono_CSHARP_VSG_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_VSG_SOURCES ${Chrono_CSHARP_VSG_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_VSG_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_VSG_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach()
  endif()
  
  # Postprocess module
  list (FIND module_list "postprocess" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_POSTPROCESS_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/postprocess")
    file(GLOB Chrono_CSHARP_POSTPROCESS_SOURCES 
         LIST_DIRECTORIES false 
         RELATIVE "${Chrono_CSHARP_POSTPROCESS_DIR}" 
         CONFIGURE_DEPENDS 
         "${Chrono_CSHARP_POSTPROCESS_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_POSTPROCESS_DUPLICATES "${tmp_list}"
                                                        "${Chrono_CSHARP_POSTPROCESS_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_POSTPROCESS_SOURCES ${Chrono_CSHARP_POSTPROCESS_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_POSTPROCESS_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_POSTPROCESS_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach()
  endif()
  
  # Vehicle module
  list (FIND module_list "vehicle" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_VEHICLE_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/vehicle")
    file(GLOB Chrono_CSHARP_VEHICLE_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_VEHICLE_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_VEHICLE_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_VEHICLE_DUPLICATES "${tmp_list}"
                                                    "${Chrono_CSHARP_VEHICLE_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_VEHICLE_SOURCES ${Chrono_CSHARP_VEHICLE_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_VEHICLE_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_VEHICLE_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach()
  endif()
  
  # Sensor module
  list (FIND module_list "sensor" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_SENSOR_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/sensor")
    file(GLOB Chrono_CSHARP_SENSOR_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_SENSOR_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_SENSOR_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_SENSOR_DUPLICATES "${tmp_list}"
                                                   "${Chrono_CSHARP_SENSOR_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_SENSOR_SOURCES ${Chrono_CSHARP_SENSOR_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_SENSOR_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_SENSOR_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach()
  endif()
  
  # Robot models
  list (FIND module_list "robot" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_ROBOT_DIR "${PROJECT_BINARY_DIR}/../chrono_csharp/robot")
    file(GLOB Chrono_CSHARP_ROBOT_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_ROBOT_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_ROBOT_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_ROBOT_DUPLICATES "${tmp_list}"
                                                  "${Chrono_CSHARP_ROBOT_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_ROBOT_SOURCES ${Chrono_CSHARP_ROBOT_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_ROBOT_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_ROBOT_DIR}/${CS}")
      list(APPEND tmp_list "${CS}")
    endforeach()
  endif()

  set(${out_list} "${file_list}" PARENT_SCOPE)
endfunction()

# ----------------------------------------------------------------------

cmake_minimum_required(VERSION 3.18)
project(Csharp_demos )

# Enable C# language
include(CheckLanguage)
check_language(CSharp)

if(NOT CMAKE_CSharp_COMPILER)
  message("WARNING: The C# demos require a C# compiler, but none was found; disabling C# demos")
  return()
endif()

enable_language(CSharp)
mark_as_advanced(CMAKE_CSharp_COMPILER)

# Track whether the managed wrapper DLL is available so demos either reference it or fall back to using generated SWIG
# sources directly in each demo
set(CHRONO_CSHARP_USING_WRAPPER OFF)
if(CH_USE_CSHARP_WRAPPER)
  set(CHRONO_CSHARP_USING_WRAPPER ON)
  if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    message(FATAL_ERROR "CMAKE_RUNTIME_OUTPUT_DIRECTORY must be provided when CH_USE_CSHARP_WRAPPER is ON")
  endif()
  if(NOT CMAKE_CFG_INTDIR)
    set(CMAKE_CFG_INTDIR ".")
  endif()
  set(CHRONO_CSHARP_WRAPPER_REFERENCE
      "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_CFG_INTDIR}/chrono_csharp_wrapper.dll")
  message(STATUS "  Using chrono_csharp_wrapper.dll: ${CHRONO_CSHARP_WRAPPER_REFERENCE}")
endif()

# function to inject chrono_csharp_wrapper.dll as a reference when the wrapper
# has been published; keeps each exe's small / avoids embedding all SWIG sources directly in each demo exe
function(chrono_configure_csharp_demo target_name)
  if(CHRONO_CSHARP_USING_WRAPPER)
    get_property(_existing_refs TARGET ${target_name} PROPERTY VS_DOTNET_REFERENCES)
    if(NOT _existing_refs)
      set(_existing_refs "")
    endif()
    list(APPEND _existing_refs "chrono_csharp_wrapper")
    list(REMOVE_DUPLICATES _existing_refs)
    list(JOIN _existing_refs ";" _refs_string)
    set_property(TARGET ${target_name} PROPERTY VS_DOTNET_REFERENCES "${_refs_string}")
    set_property(TARGET ${target_name} PROPERTY VS_DOTNET_REFERENCE_chrono_csharp_wrapper "${CHRONO_CSHARP_WRAPPER_REFERENCE}")
    set_property(TARGET ${target_name} PROPERTY VS_DOTNET_REFERENCEPROPERTY_chrono_csharp_wrapper_CopyLocal "True")
  endif()
endfunction()

# Traverse subdirectories
message(STATUS "Configure C# demo programs")
message(STATUS "  VEHICLE module enabled?     ${CH_ENABLE_MODULE_VEHICLE}")
message(STATUS "  SENSOR module enabled?      ${CH_ENABLE_MODULE_SENSOR}")
message(STATUS "  POSTPROCESS module enabled? ${CH_ENABLE_MODULE_POSTPROCESS}")
message(STATUS "  IRRLICHT module enabled?    ${CH_ENABLE_MODULE_IRRLICHT}")
message(STATUS "  VSG module enabled?         ${CH_ENABLE_MODULE_VSG}")
message(STATUS "  Runtime output directory:   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

add_subdirectory(core)
add_subdirectory(mbs)
add_subdirectory(vehicle)
add_subdirectory(sensor)
