
# ----- Blaze library -----
set(Blaze_ROOT_TEMP ${Blaze_ROOT})

find_path(Blaze_ROOT NAMES blaze/Blaze.h PATHS ${Blaze_ROOT_TEMP} "/usr/include" "/usr/local/include")
if (NOT Blaze_ROOT)
  message("WARNING Cannot find '<Blaze_ROOT>/blaze/system/Version.h'. Properly set Blaze_ROOT.")
endif()

# Extract Blaze version
find_file(Blaze_VERSION_FILENAME "Version.h" PATHS "${Blaze_ROOT}/blaze/system")
mark_as_advanced(FORCE Blaze_VERSION_FILENAME)
if(Blaze_VERSION_FILENAME)
  file(READ ${Blaze_VERSION_FILENAME} Blaze_VERSION_FILE)
  string(REGEX MATCH "#define BLAZE_MAJOR_VERSION ([0-9]*)" _Blaze_MAJOR_VERSION ${Blaze_VERSION_FILE})
  set(Blaze_MAJOR_VERSION ${CMAKE_MATCH_1})
  string(REGEX MATCH "#define BLAZE_MINOR_VERSION ([0-9]*)" _Blaze_MINOR_VERSION ${Blaze_VERSION_FILE})
  set(Blaze_MINOR_VERSION ${CMAKE_MATCH_1})
  set(Blaze_VERSION "${Blaze_MAJOR_VERSION}.${Blaze_MINOR_VERSION}")
  if(NOT Blaze_FIND_QUIETLY)
    message(STATUS "Blaze version file: ${Blaze_VERSION_FILENAME}")
    message(STATUS "Blaze version: ${Blaze_VERSION}")
  endif()
  set(Blaze_FOUND TRUE)
else()
  message("WARNING Cannot find '<Blaze_ROOT>/blaze/system/Version.h'. Properly set Blaze_ROOT.")
endif()

# ----- BOOST -- required only for older versions of Blaze -----

if (Blaze_VERSION VERSION_LESS "3.2")
  set(BOOST_REQUIRED "TRUE")
else()
  set(BOOST_REQUIRED "FALSE")
endif()

if(BOOST_REQUIRED)
  set(BOOST_ROOT "" CACHE PATH "Where is Boost located?")
  find_package(Boost REQUIRED)

  if (Boost_FOUND)
    if(NOT Blaze_FIND_QUIETLY)
      message(STATUS "Boost include dir: ${Boost_INCLUDE_DIRS}")
    endif()
  else()
    mark_as_advanced(CLEAR BOOST_ROOT)
    message("WARNING Boost required for Blaze version ${Blaze_VERSION}. Specify BOOST_ROOT or use Blaze 3.2 or newer.")
  endif()
endif()

# Create Blaze::Blaze target
if (Blaze_ROOT AND NOT TARGET Blaze::Blaze)
    add_library(Blaze::Blaze INTERFACE IMPORTED)
    set_target_properties(Blaze::Blaze PROPERTIES
                          INTERFACE_INCLUDE_DIRECTORIES "${Blaze_ROOT}")
endif()
