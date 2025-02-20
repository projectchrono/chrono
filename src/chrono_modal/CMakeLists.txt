#=============================================================================
# CMake configuration file for the Chrono modal module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_MODAL "Enable the Chrono Modal module" OFF)

# Return now if this module is not enabled
IF(NOT CH_ENABLE_MODULE_MODAL)
  MARK_AS_ADVANCED(FORCE Spectra_INCLUDE_DIR)
  RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Modal module ====\n")

MARK_AS_ADVANCED(CLEAR Spectra_INCLUDE_DIR)

#-----------------------------------------------------------------------------

SET(Chrono_MODAL_SOURCES 
    ChModalAssembly.cpp
    ChModalDamping.cpp
    ChSymGenEigenvalueSolver.cpp
    ChModalSolverUndamped.cpp
    ChGeneralizedEigenvalueSolver.cpp
    ChUnsymGenEigenvalueSolver.cpp
    ChModalSolver.cpp
    ChModalSolverDamped.cpp
    ChKrylovSchurEig.cpp
)

SET(Chrono_MODAL_HEADERS
    ChApiModal.h 
    ChModalAssembly.h
    ChModalDamping.h
    ChGeneralizedEigenvalueSolver.h
    ChSymGenEigenvalueSolver.h
    ChModalSolverUndamped.h
    ChUnsymGenEigenvalueSolver.h
    ChModalSolverDamped.h
    ChModalSolver.h
    ChKrylovSchurEig.h
)

if(CH_ENABLE_MODULE_IRRLICHT)
  set(Chrono_MODAL_HEADERS ${Chrono_MODAL_HEADERS}
      ChModalVisualSystemIrrlicht.h
  )
endif()

SOURCE_GROUP("" FILES ${Chrono_MODAL_SOURCES} ${Chrono_MODAL_HEADERS})

#-----------------------------------------------------------------------------
# Find Spectra
#-----------------------------------------------------------------------------

set(Spectra_INCLUDE_DIR "spectra/include" CACHE PATH "Path to Spectra include directory, shall contain the subdirectory named 'Spectra'.")

find_package(Spectra REQUIRED)

# ------------------------------------------------------------------------------
# Set dependencies on other Chrono modules
# ------------------------------------------------------------------------------

set(DEPENDENCIES_MODAL "")
if(CH_ENABLE_MODULE_IRRLICHT)
  list (APPEND DEPENDENCIES_MODAL "IRRLICHT")
endif()

set(DEPENDENCIES_MODAL ${DEPENDENCIES_MODAL} PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Add the Chrono_modal library
#-----------------------------------------------------------------------------

add_library(Chrono_modal ${Chrono_MODAL_SOURCES} ${Chrono_MODAL_HEADERS})
add_library(Chrono::modal ALIAS Chrono_modal)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_modal PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_modal PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_modal PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_modal PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_modal PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_MODAL>)

target_link_libraries(Chrono_modal PRIVATE Chrono_core)
target_link_libraries(Chrono_modal PRIVATE Spectra::Spectra)

install(TARGETS Chrono_modal
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_modal
        )

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_MODAL_HEADERS}
        DESTINATION include/chrono_modal)

# Install files for chrono-config (in both build and install trees)
file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindSpectra.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindSpectra.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})
