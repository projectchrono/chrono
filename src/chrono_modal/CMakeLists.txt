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

SET(ChronoEngine_MODAL_SOURCES 
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

SET(ChronoEngine_MODAL_HEADERS
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
  set(ChronoEngine_MODAL_HEADERS ${ChronoEngine_MODAL_HEADERS}
      ChModalVisualSystemIrrlicht.h
  )
endif()

SOURCE_GROUP("" FILES ${ChronoEngine_MODAL_SOURCES} ${ChronoEngine_MODAL_HEADERS})

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
# Add the ChronoEngine_modal library
#-----------------------------------------------------------------------------

add_library(ChronoEngine_modal ${ChronoEngine_MODAL_SOURCES} ${ChronoEngine_MODAL_HEADERS})
add_library(ChronoEngine::modal ALIAS ChronoEngine_modal)

if(CH_WHOLE_PROG_OPT)
  set_target_properties(ChronoEngine_modal PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(ChronoEngine_modal PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(ChronoEngine_modal PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(ChronoEngine_modal PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(ChronoEngine_modal PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_MODAL>)

target_link_libraries(ChronoEngine_modal PRIVATE ChronoEngine)
target_link_libraries(ChronoEngine_modal PRIVATE Spectra::Spectra)

install(TARGETS ChronoEngine_modal
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_modal
        )

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${ChronoEngine_MODAL_HEADERS}
        DESTINATION include/chrono_modal)

# Install files for chrono-config (in both build and install trees)
file(COPY ${CMAKE_SOURCE_DIR}/cmake/FindSpectra.cmake DESTINATION ${CMAKE_BINARY_DIR}/cmake/)
install(FILES "${CMAKE_SOURCE_DIR}/cmake/FindSpectra.cmake" DESTINATION ${CH_CONFIG_INSTALL_PATH})
