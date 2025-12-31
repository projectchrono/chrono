#=============================================================================
# CMake configuration file for the Chrono postprocess module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_POSTPROCESS "Enable the Chrono Postprocess module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_POSTPROCESS)
  return()
endif()

message(STATUS "\n==== Chrono Postprocess module ====\n")

#-----------------------------------------------------------------------------

find_package(Gnuplot QUIET)

if(GNUPLOT_FOUND)
  message(STATUS "Gnuplot executable: ${GNUPLOT_EXECUTABLE}")
  message(STATUS "Gnuplot version:    ${GNUPLOT_VERSION_STRING}")
  set(CHRONO_HAS_GNUPLOT "#define CHRONO_HAS_GNUPLOT" PARENT_SCOPE)
endif()

#-----------------------------------------------------------------------------

set(Chrono_POSTPROCESS_SOURCES 
    ChPovRay.cpp
    ChBlender.cpp
)

set(Chrono_POSTPROCESS_HEADERS
    ChApiPostProcess.h 
    ChPostProcessBase.h
    ChPovRay.h
    ChBlender.h
)

if(GNUPLOT_FOUND)
  set(Chrono_POSTPROCESS_HEADERS ${Chrono_POSTPROCESS_HEADERS}
      ChGnuPlot.h)
endif()

source_group("" FILES 
            ${Chrono_POSTPROCESS_SOURCES} 
            ${Chrono_POSTPROCESS_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

add_library(Chrono_postprocess ${Chrono_POSTPROCESS_SOURCES} ${Chrono_POSTPROCESS_HEADERS})
add_library(Chrono::postprocess ALIAS Chrono_postprocess)

set_target_properties(Chrono_postprocess PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_postprocess PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_postprocess PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_postprocess PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_postprocess PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_postprocess PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_POSTPROCESS>)

target_link_libraries(Chrono_postprocess PRIVATE Chrono_core)

install(TARGETS Chrono_postprocess
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_postprocess) 

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_POSTPROCESS_HEADERS}
        DESTINATION include/chrono_postprocess)
