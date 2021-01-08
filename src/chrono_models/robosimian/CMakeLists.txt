#=============================================================================
# CMake configuration file for the Chrono Robosimian module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

MESSAGE(STATUS "Model of the ROBOSIMIAN robot...")

#-----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES "ChronoEngine")

SET(ChronoModels_robosimian_SOURCES
    RoboSimian.cpp
)

SET(ChronoModels_robosimian_HEADERS
    ../ChApiModels.h
    RoboSimian.h
)

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    include_directories(${CH_IRRLICHTINC})

    set(ChronoModels_robosimian_SOURCES
        ${ChronoModels_robosimian_SOURCES}
        RoboSimianIrrApp.cpp)
    set(ChronoModels_robosimian_HEADERS
        ${ChronoModels_robosimian_HEADERS}
        RoboSimianIrrApp.h)
endif()

SOURCE_GROUP("" FILES
            ${ChronoModels_robosimian_SOURCES} 
            ${ChronoModels_robosimian_HEADERS})

#-----------------------------------------------------------------------------

ADD_LIBRARY(ChronoModels_robosimian SHARED 
            ${ChronoModels_robosimian_SOURCES}
            ${ChronoModels_robosimian_HEADERS})

SET_TARGET_PROPERTIES(ChronoModels_robosimian PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")
target_compile_definitions(ChronoModels_robosimian PRIVATE "CH_API_COMPILE_MODELS")
target_compile_definitions(ChronoModels_robosimian PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoModels_robosimian ${LIBRARIES})

INSTALL(TARGETS ChronoModels_robosimian
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoModels_robosimian_HEADERS}
        DESTINATION include/chrono_robosimian)
