#=============================================================================
# CMake configuration file for Chrono Matlab module
#=============================================================================

option(CH_ENABLE_MODULE_MATLAB "Enable the Chrono Matlab module" OFF)

# Return now if this module is not enabled
IF(NOT CH_ENABLE_MODULE_MATLAB)
  MARK_AS_ADVANCED(FORCE Matlab_ROOT_DIR)
  RETURN()
ENDIF()

message(STATUS "\n==== Chrono Matlab module ====\n")

# --------------------------------------------------------

set(ChronoEngine_MATLAB_HEADERS
  ChApiMatlab.h
  ChMatlabEngine.h
  ChSolverMatlab.h
)

set(ChronoEngine_MATLAB_SOURCES
  ChMatlabEngine.cpp
  ChSolverMatlab.cpp
)

# --------------------------------------------------------

find_package(Matlab COMPONENTS MAT_LIBRARY ENG_LIBRARY REQUIRED)

if(Matlab_FOUND)
  message(STATUS "Matlab version: ${Matlab_VERSION}")
  message(STATUS "Matlab C++ ENGINE library: ${Matlab_ENGINE_LIBRARY}")
  message(STATUS "Matlab C ENG library:      ${Matlab_ENG_LIBRARY}")
  message(STATUS "Matlab MX library:         ${Matlab_MX_LIBRARY}")
  message(STATUS "Matlab MAT library:        ${Matlab_MAT_LIBRARY}")
endif()

# --------------------------------------------------------

add_library(ChronoEngine_matlab ${ChronoEngine_MATLAB_SOURCES} ${ChronoEngine_MATLAB_HEADERS})
add_library(ChronoEngine::matlab ALIAS ChronoEngine_matlab)
 
if(CH_WHOLE_PROG_OPT)
  set_target_properties(ChronoEngine_matlab PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(ChronoEngine_matlab PROPERTIES LINK_FLAGS "/LTCG")
endif()

target_link_libraries(ChronoEngine_matlab PRIVATE ChronoEngine)

target_link_libraries(ChronoEngine_matlab PRIVATE Matlab::MatlabEngine) # C++
target_link_libraries(ChronoEngine_matlab PUBLIC Matlab::eng)           # C
#target_link_libraries(ChronoEngine_matlab PRIVATE Matlab::mat)
#target_link_libraries(ChronoEngine_matlab PRIVATE Matlab::mx)
                          
target_compile_definitions(ChronoEngine_matlab PRIVATE "CH_API_COMPILE_MATLAB")

if(MSVC)
  set_target_properties(ChronoEngine_matlab PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

install(TARGETS ChronoEngine_matlab
  EXPORT ChronoEngineTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_matlab
)      

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${ChronoEngine_MATLAB_HEADERS}
        DESTINATION include/chrono_matlab)
