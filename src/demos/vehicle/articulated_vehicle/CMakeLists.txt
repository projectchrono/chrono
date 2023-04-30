#=============================================================================
# CMake configuration file for the ARTICULATED VEHICLE demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    subsystems/ACV_Vehicle.h
    subsystems/ACV_Vehicle.cpp
    subsystems/ACV_ChassisFront.h
    subsystems/ACV_ChassisFront.cpp
    subsystems/ACV_ChassisRear.h
    subsystems/ACV_ChassisRear.cpp
    subsystems/ACV_ChassisConnector.h
    subsystems/ACV_ChassisConnector.cpp
    subsystems/ACV_RigidSuspension.h
    subsystems/ACV_RigidSuspension.cpp
    subsystems/ACV_Driveline2WD.h
    subsystems/ACV_Driveline2WD.cpp
    subsystems/ACV_Wheel.h
    subsystems/ACV_Wheel.cpp
    subsystems/ACV_BrakeSimple.h
    subsystems/ACV_BrakeSimple.cpp
    subsystems/ACV_RigidTire.h
    subsystems/ACV_RigidTire.cpp
    subsystems/ACV_EngineSimple.h
    subsystems/ACV_EngineSimple.cpp
    subsystems/ACV_AutomaticTransmissionSimple.h
    subsystems/ACV_AutomaticTransmissionSimple.cpp
)

SET(PROGRAM
    demo_VEH_ArticulatedVehicle
)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR})
INCLUDE_DIRECTORIES(${CH_IRRLICHT_INCLUDES})

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${PROGRAM}.cpp ${MODEL_FILES})

SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM}
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle
                      ChronoEngine_vehicle_irrlicht
                      ChronoModels_vehicle)

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
