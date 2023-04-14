#=============================================================================
# CMake configuration file for the Chrono Robosimian module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

MESSAGE(STATUS "Robot models...")

#-----------------------------------------------------------------------------

set(CRM_COMMON_FILES
    ../ChApiModels.h
)
source_group("" FILES ${CRM_COMMON_FILES})

#-----------------------------------------------------------------------------

set(CRM_ROBOSIMIAN_FILES
    robosimian/RoboSimian.cpp
    robosimian/RoboSimian.h
)

if(ENABLE_MODULE_IRRLICHT)
set(CRM_ROBOSIMIAN_FILES ${CRM_ROBOSIMIAN_FILES}
    robosimian/RoboSimianVisualSystemIrrlicht.cpp
    robosimian/RoboSimianVisualSystemIrrlicht.h
)
endif()

source_group("robosimian" FILES ${CRM_ROBOSIMIAN_FILES})

#-----------------------------------------------------------------------------

set(CRM_VIPER_FILES
    viper/Viper.cpp
    viper/Viper.h
)

source_group("viper" FILES ${CRM_VIPER_FILES})

#-----------------------------------------------------------------------------

set(CRM_TURTLEBOT_FILES
    turtlebot/Turtlebot.cpp
    turtlebot/Turtlebot.h
)

source_group("turtlebot" FILES ${CRM_TURTLEBOT_FILES})

#-----------------------------------------------------------------------------

set(CRM_CURIOSITY_FILES
    curiosity/Curiosity.cpp
    curiosity/Curiosity.h
)

source_group("curiosity" FILES ${CRM_CURIOSITY_FILES})


#-----------------------------------------------------------------------------

set(CRM_COPTERS_FILES
    copters/Copter.h
    copters/Little_Hexy.h
    copters/Little_Hexy.cpp
)

source_group("copters" FILES ${CRM_COPTERS_FILES})

#-----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    include_directories(${CH_IRRLICHT_INCLUDES})
endif()

#-----------------------------------------------------------------------------

add_library(ChronoModels_robot SHARED 
            ${CRM_COMMON_FILES}
            ${CRM_ROBOSIMIAN_FILES}
            ${CRM_CURIOSITY_FILES}
            ${CRM_VIPER_FILES}
            ${CRM_TURTLEBOT_FILES}
            ${CRM_COPTERS_FILES}
            )

set_target_properties(ChronoModels_robot PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")

target_compile_definitions(ChronoModels_robot PRIVATE "CH_API_COMPILE_MODELS")
target_compile_definitions(ChronoModels_robot PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoModels_robot ${LIBRARIES})

install(TARGETS ChronoModels_robot
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_models/robot
        FILES_MATCHING PATTERN "*.h")

