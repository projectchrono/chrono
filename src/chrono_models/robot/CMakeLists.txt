#=============================================================================
# CMake configuration file for the Chrono Robosimian module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

MESSAGE(STATUS "Add ChronoModels_robot library")

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

set(CRM_INDUSTRIAL_FILES
    industrial/IndustrialRobot.h industrial/IndustrialRobot.cpp
    industrial/IndustrialRobotSCARA.h industrial/IndustrialRobotSCARA.cpp
    industrial/IndustrialRobotSCARA_CAD.h industrial/IndustrialRobotSCARA_CAD.cpp
    industrial/IndustrialRobot6dof.h industrial/IndustrialRobot6dof.cpp
    industrial/IndustrialRobot6dofCAD.h industrial/IndustrialRobot6dofCAD.cpp
    industrial/IndustrialKinematics.h industrial/IndustrialKinematics.cpp
    industrial/IndustrialKinematicsSCARA.h industrial/IndustrialKinematicsSCARA.cpp
    industrial/IndustrialKinematics6dofSpherical.h industrial/IndustrialKinematics6dofSpherical.cpp
    industrial/IndustrialKinematicsNdofNumerical.h industrial/IndustrialKinematicsNdofNumerical.cpp
    industrial/TrajectoryInterpolator.h industrial/TrajectoryInterpolator.cpp
)

source_group("industrial" FILES ${CRM_INDUSTRIAL_FILES})

#-----------------------------------------------------------------------------

add_library(ChronoModels_robot 
            ${CRM_COMMON_FILES}
            ${CRM_ROBOSIMIAN_FILES}
            ${CRM_CURIOSITY_FILES}
            ${CRM_VIPER_FILES}
            ${CRM_TURTLEBOT_FILES}
            ${CRM_COPTERS_FILES}
            ${CRM_INDUSTRIAL_FILES}
            )
add_library(Chrono::models_robot ALIAS ChronoModels_robot)

set_target_properties(ChronoModels_robot PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})


if(CH_WHOLE_PROG_OPT)
  set_target_properties(ChronoModels_robot PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(ChronoModels_robot PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(ChronoModels_robot PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(ChronoModels_robot PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(ChronoModels_robot PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_MODELS>)

target_link_libraries(ChronoModels_robot PUBLIC Chrono_core)

install(TARGETS ChronoModels_robot
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_models/robot)

# -------------------------------------------------------------------------------------------------
# Install files
# -------------------------------------------------------------------------------------------------

# Old way (install headers preserving directory structure)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_models/robot
        FILES_MATCHING PATTERN "*.h")
