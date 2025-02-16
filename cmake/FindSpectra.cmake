
#-------------------------------------------------------------------------------
# Find Spectra
# ATTENTION: Spectra library (https://github.com/yixuan/spectra) has its own spectra-config.cmake
# but needs to be installed; we provide an easier way

# This find script requires the following input variables:
# - Spectra_INCLUDE_DIR: shall contain the subdirectory named 'Spectra'
# This find script provides the following output variables/targets:
# - Spectra_FOUND: a boolean indicating whether the library was found
# - Spectra::Spectra: imported target

set(Spectra_FOUND TRUE)

find_path(Spectra_INCLUDE_DIR_INTERNAL NAMES SymEigsBase.h PATHS "${Spectra_INCLUDE_DIR}/Spectra" NO_CACHE)
mark_as_advanced(Spectra_INCLUDE_DIR_INTERNAL)

if (NOT Spectra_INCLUDE_DIR_INTERNAL)
  message(NOTICE "Could not find '${Spectra_INCLUDE_DIR}/Spectra/SymEigsBase.h'. Set Spectra_INCLUDE_DIR to a folder containing the subfolder 'Spectra'.")
  return()
endif()

set(Spectra_FOUND TRUE)

if(Spectra_FOUND AND NOT TARGET Spectra::Spectra)
  add_library(Spectra::Spectra INTERFACE IMPORTED)
  set_target_properties(Spectra::Spectra PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${Spectra_INCLUDE_DIR}")
  if(MSVC)
    set_property(TARGET Spectra::Spectra PROPERTY
                 INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CXX>:/bigobj>)
  endif()
               
endif()

