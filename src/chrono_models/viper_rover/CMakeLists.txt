#=============================================================================
# CMake configuration file for the Chrono viper module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

MESSAGE(STATUS "Model of the VIPER rover...")

#-----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES "ChronoEngine")

SET(ChronoModels_viper_SOURCES
    Viper.cpp
)

SET(ChronoModels_viper_HEADERS
    ../ChApiModels.h
    Viper.h
)

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    include_directories(${CH_IRRLICHTINC})

    set(ChronoModels_viper_SOURCES
        ${ChronoModels_viper_SOURCES})
    set(ChronoModels_viper_HEADERS
        ${ChronoModels_viper_HEADERS})
endif()

SOURCE_GROUP("" FILES
            ${ChronoModels_viper_SOURCES} 
            ${ChronoModels_viper_HEADERS})

#-----------------------------------------------------------------------------

ADD_LIBRARY(ChronoModels_viper SHARED 
            ${ChronoModels_viper_SOURCES}
            ${ChronoModels_viper_HEADERS})

SET_TARGET_PROPERTIES(ChronoModels_viper PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")
target_compile_definitions(ChronoModels_viper PRIVATE "CH_API_COMPILE_MODELS")
target_compile_definitions(ChronoModels_viper PRIVATE "CH_IGNORE_DEPRECATED")

TARGET_LINK_LIBRARIES(ChronoModels_viper ${LIBRARIES})

INSTALL(TARGETS ChronoModels_viper
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

INSTALL(FILES ${ChronoModels_viper_HEADERS}
        DESTINATION include/chrono_viper)
