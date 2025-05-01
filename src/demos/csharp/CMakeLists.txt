# Enable C# language
include(CheckLanguage)
check_language(CSharp)

if(NOT CMAKE_CSharp_COMPILER)
  message("WARNING: The C# demos require a C# compiler, but none was found; disabling C# demos")
  return()
endif()

enable_language(CSharp)

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

  # Core module
  list (FIND module_list "core" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_CORE_DIR "${PROJECT_BINARY_DIR}/chrono_csharp/core")
    file(GLOB Chrono_CSHARP_CORE_SOURCES 
         LIST_DIRECTORIES false 
         RELATIVE "${Chrono_CSHARP_CORE_DIR}" 
         CONFIGURE_DEPENDS 
         "${Chrono_CSHARP_CORE_DIR}/*.cs")
    foreach(CS ${Chrono_CSHARP_CORE_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_CORE_DIR}/${CS}")
    endforeach() 
  endif()
  
  # Irrlict module
  list (FIND module_list "irrlicht" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_IRRLICHT_DIR "${PROJECT_BINARY_DIR}/chrono_csharp/irrlicht")
    file(GLOB Chrono_CSHARP_IRRLICHT_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_IRRLICHT_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_IRRLICHT_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_IRRLICHT_DUPLICATES "${Chrono_CSHARP_CORE_SOURCES}"
                                                     "${Chrono_CSHARP_IRRLICHT_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_IRRLICHT_SOURCES ${Chrono_CSHARP_IRRLICHT_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_IRRLICHT_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_IRRLICHT_DIR}/${CS}")
    endforeach()
  endif()
  
  # Postprocess module
  list (FIND module_list "postprocess" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_POSTPROCESS_DIR "${PROJECT_BINARY_DIR}/chrono_csharp/postprocess")
    file(GLOB Chrono_CSHARP_POSTPROCESS_SOURCES 
         LIST_DIRECTORIES false 
         RELATIVE "${Chrono_CSHARP_POSTPROCESS_DIR}" 
         CONFIGURE_DEPENDS 
         "${Chrono_CSHARP_POSTPROCESS_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_POSTPROCESS_DUPLICATES "${Chrono_CSHARP_CORE_SOURCES}"
                                                        "${Chrono_CSHARP_POSTPROCESS_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_POSTPROCESS_SOURCES ${Chrono_CSHARP_POSTPROCESS_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_POSTPROCESS_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_POSTPROCESS_DIR}/${CS}")
    endforeach()
  endif()
  
  # Vehicle module
  list (FIND module_list "vehicle" index)
  if (${index} GREATER -1)
    set(Chrono_CSHARP_VEHICLE_DIR "${PROJECT_BINARY_DIR}/chrono_csharp/vehicle")
    file(GLOB Chrono_CSHARP_VEHICLE_SOURCES
         LIST_DIRECTORIES false
         RELATIVE "${Chrono_CSHARP_VEHICLE_DIR}"
         CONFIGURE_DEPENDS
         "${Chrono_CSHARP_VEHICLE_DIR}/*.cs")
    FindDuplicates(Chrono_CSHARP_VEHICLE_DUPLICATES "${Chrono_CSHARP_CORE_SOURCES}"
                                                    "${Chrono_CSHARP_VEHICLE_SOURCES}")
    list(REMOVE_ITEM Chrono_CSHARP_VEHICLE_SOURCES ${Chrono_CSHARP_VEHICLE_DUPLICATES})
    foreach(CS ${Chrono_CSHARP_VEHICLE_SOURCES})
      list(APPEND file_list "${Chrono_CSHARP_VEHICLE_DIR}/${CS}")
    endforeach()
  endif()

  set(${out_list} "${file_list}" PARENT_SCOPE)
endfunction()

# ----------------------------------------------------------------------


# Traverse subdirectories
message(STATUS "Add demo programs for C# module")

add_subdirectory(core)
add_subdirectory(mbs)
add_subdirectory(vehicle)
