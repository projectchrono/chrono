# Find Blaze
#
# This script requires one of the following input variables:
# - blaze_DIR: directory containing the Blaze package configuration script
# - blaze_INCLUDE_DIR: directory containing the subdirectory 'blaze/'
#
# This script provides the following outputs:
# - blaze_FOUND: a boolean variable indicating whether the library was found
# - blaze::blaze: imported target

if(blaze_INCLUDE_DIR)

  if(EXISTS "${blaze_INCLUDE_DIR}/blaze/system/Version.h")
    if(NOT TARGET blaze::blaze)
      add_library(blaze::blaze INTERFACE IMPORTED)
      set_target_properties(blaze::blaze PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${blaze_INCLUDE_DIR}")
    endif()
    set(blaze_FOUND TRUE)
    if(NOT Blaze_FIND_QUIETLY)
      message(STATUS "Found 'blaze/system/Version.h' in provided blaze_INCLUDE_DIR=${blaze_INCLUDE_DIR}")
    endif()
  else()
    set(blaze_FOUND FALSE)
    if(NOT Blaze_FIND_QUIETLY)
      message(STATUS "Could not find 'blaze/system/Version.h' in the provided blaze_INCLUDE_DIR=${blaze_INCLUDE_DIR}")
    endif()
  endif()

else()

  find_package(blaze NO_MODULE)

  if(blaze_FOUND)
    get_target_property(blaze_INCLUDE_DIR blaze::blaze INTERFACE_INCLUDE_DIRECTORIES)
    if(NOT Blaze_FIND_QUIETLY)
      message(STATUS "Blaze found through config script")
    endif()
  endif()

endif()

# Check Blaze version

if(NOT blaze_FOUND)
  return()
endif()

find_file(blaze_VERSION_FILENAME "Version.h" PATHS "${blaze_INCLUDE_DIR}/blaze/system" NO_CACHE)

if(blaze_VERSION_FILENAME)
  
  file(READ ${blaze_VERSION_FILENAME} Blaze_VERSION_FILE)
  
  string(REGEX MATCH "#define BLAZE_MAJOR_VERSION ([0-9]*)" _Blaze_MAJOR_VERSION ${Blaze_VERSION_FILE})
  set(blaze_MAJOR_VERSION ${CMAKE_MATCH_1})
  string(REGEX MATCH "#define BLAZE_MINOR_VERSION ([0-9]*)" _Blaze_MINOR_VERSION ${Blaze_VERSION_FILE})
  set(blaze_MINOR_VERSION ${CMAKE_MATCH_1})
  set(blaze_VERSION "${blaze_MAJOR_VERSION}.${blaze_MINOR_VERSION}")
  
  if(NOT Blaze_FIND_QUIETLY)
    message(STATUS "Blaze version file: ${blaze_VERSION_FILENAME}")
    message(STATUS "Blaze version: ${blaze_VERSION}")
  endif()

  if (blaze_VERSION VERSION_LESS "3.2")
    set(blaze_FOUND FALSE)
    if(NOT Blaze_FIND_QUIETLY)
      message(STATUS "Blaze version older than 3.2.")
    endif()
    return()
  endif()

else()

  set(blaze_FOUND FALSE)

  if(NOT Blaze_FIND_QUIETLY)
    message(STATUS "Cannot find header 'blaze/system/Version.h'")
  endif()

  return()

endif()

