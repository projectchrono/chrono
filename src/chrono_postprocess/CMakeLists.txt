#=============================================================================
# CMake configuration file for the Chrono postprocess module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(ENABLE_MODULE_POSTPROCESS "Enable the Chrono Postprocess module" OFF)

# Return now if this module is not enabled
IF(NOT ENABLE_MODULE_POSTPROCESS)
	RETURN()
ENDIF()

MESSAGE(STATUS "\n==== Chrono Postprocess module ====\n")

#-----------------------------------------------------------------------------
# LIST THE FILES THAT MAKE THE POSTPROCESS LIBRARY

SET(ChronoEngine_POSTPROCESS_SOURCES 
    ChPovRay.cpp
    ChBlender.cpp
)

SET(ChronoEngine_POSTPROCESS_HEADERS
    ChApiPostProcess.h 
    ChPostProcessBase.h
    ChGnuPlot.h
    ChPovRay.h
    ChBlender.h
)

SOURCE_GROUP("" FILES 
            ${ChronoEngine_POSTPROCESS_SOURCES} 
            ${ChronoEngine_POSTPROCESS_HEADERS})

#-----------------------------------------------------------------------------	
# In most cases, you do not need to edit the lines below.

ADD_LIBRARY(ChronoEngine_postprocess 
			${ChronoEngine_POSTPROCESS_SOURCES}
			${ChronoEngine_POSTPROCESS_HEADERS})
                          
target_compile_definitions(ChronoEngine_postprocess PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_POSTPROCESS>)
target_compile_definitions(ChronoEngine_postprocess PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_IGNORE_DEPRECATED>)

target_link_libraries(ChronoEngine_postprocess PRIVATE ChronoEngine)
	
install(TARGETS ChronoEngine_postprocess
        EXPORT ChronoEngineTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_vehicle)		

install(FILES ${ChronoEngine_POSTPROCESS_HEADERS}
        DESTINATION include/chrono_postprocess)


