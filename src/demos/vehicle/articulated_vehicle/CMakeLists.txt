#=============================================================================
# CMake configuration file for the ARTICULATED VEHICLE demo.
#=============================================================================

if(NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

set(MODEL_FILES
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

set(PROGRAM
    demo_VEH_ArticulatedVehicle
)

#--------------------------------------------------------------
# Additional include directories

include_directories(${CMAKE_CURRENT_LIST_DIR})

#--------------------------------------------------------------
# Create the executable

add_executable(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
source_group("subsystems" FILES ${MODEL_FILES})
source_group("" FILES ${PROGRAM}.cpp ${MODEL_FILES})

if(APPLE)
   set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
endif()

if(MSVC)
  set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM}
                      Chrono_vehicle
                      Chrono_vehicle_vsg
                      ChronoModels_vehicle)

install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
