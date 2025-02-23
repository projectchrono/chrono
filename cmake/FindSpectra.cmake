# Find Spectra
#
# This script requires one of the following input variables:
# - spectra_DIR: directory containing the Spectra package configuration script
# - spectra_INCLUDE_DIR: directory containing the subdirectory 'Spectra/'
#
# This script provides the following outputs:
# - spectra_FOUND: a boolean variable indicating whether the library was found
# - Spectra::Spectra: imported target

if(spectra_INCLUDE_DIR)

  if(EXISTS "${spectra_INCLUDE_DIR}/Spectra/KrylovSchurGEigsSolver.h")
    if(NOT TARGET Spectra::Spectra)
      add_library(Spectra::Spectra INTERFACE IMPORTED)
      set_target_properties(Spectra::Spectra PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${spectra_INCLUDE_DIR}")
      if(MSVC)
        set_property(TARGET Spectra::Spectra PROPERTY INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CXX>:/bigobj>)
      endif()             
    endif()
    set(spectra_FOUND TRUE)
    if(NOT Spectra_FIND_QUIETLY)
      message(STATUS "Found 'Spectra/KrylovSchurGEigsSolver.h' in provided spectra_INCLUDE_DIR=${spectra_INCLUDE_DIR}")
    endif()
  else()
    set(spectra_FOUND FALSE)
    if(NOT Spectra_FIND_QUIETLY)
      message(STATUS "Could not find 'Spectra/KrylovSchurGEigsSolver.h' in the provided spectra_INCLUDE_DIR=${spectra_INCLUDE_DIR}")
    endif()
  endif()

else()

  find_package(spectra NO_MODULE)
  if(spectra_FOUND)
    get_target_property(spectra_INCLUDE_DIR Spectra::Spectra INTERFACE_INCLUDE_DIRECTORIES)
    if(NOT Spectra_FIND_QUIETLY)
      message(STATUS "Spectra found through config script")
    endif()
  endif()

endif()
