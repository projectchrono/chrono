#=============================================================================
# CMake configuration file for the LongHaul vehicle demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    subsystems/SemiTractor_brake.h
    subsystems/SemiTractor_brake.cpp
    subsystems/SemiTractor_chassis.h
    subsystems/SemiTractor_chassis.cpp
    subsystems/SemiTractor_driveline.h
    subsystems/SemiTractor_driveline.cpp
    subsystems/SemiTractor_front_axle.h
    subsystems/SemiTractor_front_axle.cpp
    subsystems/SemiTractor_rear_axle.h
    subsystems/SemiTractor_rear_axle.cpp
    subsystems/SemiTractor_powertrain.h
    subsystems/SemiTractor_powertrain.cpp
    subsystems/SemiTractor_steering.h
    subsystems/SemiTractor_steering.cpp
    subsystems/SemiTractor_tire.h
    subsystems/SemiTractor_tire.cpp
    subsystems/SemiTractor_wheel.h
    subsystems/SemiTractor_wheel.cpp
    subsystems/SemiTractor_vehicle.h
    subsystems/SemiTractor_vehicle.cpp
    
    subsystems/SemiTrailer_axle.h
    subsystems/SemiTrailer_axle.cpp    
    subsystems/SemiTrailer_brake.h
    subsystems/SemiTrailer_brake.cpp    
    subsystems/SemiTrailer_chassis.h
    subsystems/SemiTrailer_chassis.cpp    
    subsystems/SemiTrailer_tire.h
    subsystems/SemiTrailer_tire.cpp    
    subsystems/SemiTrailer_wheel.h
    subsystems/SemiTrailer_wheel.cpp
    subsystems/SemiTrailer.h
    subsystems/SemiTrailer.cpp    
)

SET(DEMOS
    demo_VEH_LongHaul_ClosedLoop
    demo_VEH_LongHaul_OpenLoop
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR})
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Create the executables

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
    SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp ${MODEL_FILES})

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM}
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle
                      ChronoEngine_vehicle_irrlicht
                      ChronoModels_vehicle)

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
