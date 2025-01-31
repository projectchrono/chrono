#=============================================================================
# CMake configuration file for the Chrono postprocess module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_POSTPROCESS "Enable the Chrono Postprocess module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_POSTPROCESS)
	RETURN()
endif()

message(STATUS "\n==== Chrono Postprocess module ====\n")

#-----------------------------------------------------------------------------

find_package(Gnuplot QUIET)

if(GNUPLOT_FOUND)
  message(STATUS "Gnuplot executable: ${GNUPLOT_EXECUTABLE}")
  message(STATUS "Gnuplot version:    ${GNUPLOT_VERSION_STRING}")
endif()

#-----------------------------------------------------------------------------

set(ChronoEngine_POSTPROCESS_SOURCES 
    ChPovRay.cpp
    ChBlender.cpp
)

set(ChronoEngine_POSTPROCESS_HEADERS
    ChApiPostProcess.h 
    ChPostProcessBase.h
    ChPovRay.h
    ChBlender.h
)

if(GNUPLOT_FOUND)
  set(ChronoEngine_POSTPROCESS_HEADERS ${ChronoEngine_POSTPROCESS_HEADERS}
      ChGnuPlot.h)
endif()

source_group("" FILES 
            ${ChronoEngine_POSTPROCESS_SOURCES} 
            ${ChronoEngine_POSTPROCESS_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

add_library(ChronoEngine_postprocess 
			${ChronoEngine_POSTPROCESS_SOURCES}
			${ChronoEngine_POSTPROCESS_HEADERS})
                          
target_compile_definitions(ChronoEngine_postprocess PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_POSTPROCESS>)

target_link_libraries(ChronoEngine_postprocess PRIVATE ChronoEngine)
	
install(TARGETS ChronoEngine_postprocess
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vehicle)		

install(FILES ${ChronoEngine_POSTPROCESS_HEADERS}
        DESTINATION include/chrono_postprocess)


