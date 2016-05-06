#=============================================================================
# CMake configuration file for Chrono Vehicle Models
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

message(STATUS "Models for the VEHICLE module...")

# ----------------------------------------------------------------------------
# List the files in the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CVM_COMMON_FILES
    ../ChApiModels.h
)
source_group("" FILES ${CVM_COMMON_FILES})

set(CVM_HMMWV_FILES
    hmmwv/HMMWV.cpp
    hmmwv/HMMWV.h
    hmmwv/HMMWV_BrakeSimple.cpp
    hmmwv/HMMWV_BrakeSimple.h
    hmmwv/HMMWV_DoubleWishbone.cpp
    hmmwv/HMMWV_DoubleWishbone.h
    hmmwv/HMMWV_DoubleWishboneReduced.cpp
    hmmwv/HMMWV_DoubleWishboneReduced.h
    hmmwv/HMMWV_Driveline2WD.cpp
    hmmwv/HMMWV_Driveline2WD.h
    hmmwv/HMMWV_Driveline4WD.cpp
    hmmwv/HMMWV_Driveline4WD.h
    hmmwv/HMMWV_FialaTire.cpp
    hmmwv/HMMWV_FialaTire.h
    hmmwv/HMMWV_LugreTire.cpp
    hmmwv/HMMWV_LugreTire.h
    hmmwv/HMMWV_PitmanArm.cpp
    hmmwv/HMMWV_PitmanArm.h
    hmmwv/HMMWV_Powertrain.cpp
    hmmwv/HMMWV_Powertrain.h
    hmmwv/HMMWV_RackPinion.cpp
    hmmwv/HMMWV_RackPinion.h
    hmmwv/HMMWV_RigidTire.cpp
    hmmwv/HMMWV_RigidTire.h
    hmmwv/HMMWV_SimplePowertrain.cpp
    hmmwv/HMMWV_SimplePowertrain.h
    hmmwv/HMMWV_Vehicle.cpp
    hmmwv/HMMWV_Vehicle.h
    hmmwv/HMMWV_VehicleReduced.cpp
    hmmwv/HMMWV_VehicleReduced.h
    hmmwv/HMMWV_Wheel.cpp
    hmmwv/HMMWV_Wheel.h
)
source_group("hmmwv" FILES ${CVM_HMMWV_FILES})

set(CVM_M113_FILES
    m113/M113_BrakeSimple.h
    m113/M113_DrivelineBDS.cpp
    m113/M113_DrivelineBDS.h
    m113/M113_Idler.cpp
    m113/M113_Idler.h
    m113/M113_RoadWheel.cpp
    m113/M113_RoadWheel.h
    m113/M113_ShaftsPowertrain.cpp
    m113/M113_ShaftsPowertrain.h
    m113/M113_SimpleDriveline.cpp
    m113/M113_SimpleDriveline.h
    m113/M113_SimplePowertrain.cpp
    m113/M113_SimplePowertrain.h
    m113/M113_Sprocket.cpp
    m113/M113_Sprocket.h
    m113/M113_Suspension.cpp
    m113/M113_Suspension.h
    m113/M113_TrackAssembly.cpp
    m113/M113_TrackAssembly.h
    m113/M113_TrackShoe.cpp
    m113/M113_TrackShoe.h
    m113/M113_Vehicle.cpp
    m113/M113_Vehicle.h
)
source_group("m113" FILES ${CVM_M113_FILES})


# ----------------------------------------------------------------------------
# Add the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)
if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
    include_directories(${CH_IRRLICHTINC})
endif()

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

add_library(ChronoModels_vehicle SHARED
    ${CVM_HMMWV_FILES}
    ${CVM_M113_FILES}
 )

set_target_properties(ChronoModels_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_MODELS")

target_link_libraries(ChronoModels_vehicle ${LIBRARIES})

install(TARGETS ChronoModels_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/models/vehicle
        FILES_MATCHING PATTERN "*.h")
