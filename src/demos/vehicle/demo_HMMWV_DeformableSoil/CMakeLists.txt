#=============================================================================
# CMake configuration file for the HMMWV_DEFORMABLE_SOIL demo.
# This example program requires Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    ../hmmwv/HMMWV.h
    ../hmmwv/HMMWV.cpp
    ../hmmwv/vehicle/HMMWV_Vehicle.h
    ../hmmwv/vehicle/HMMWV_Vehicle.cpp
    ../hmmwv/vehicle/HMMWV_VehicleReduced.h
    ../hmmwv/vehicle/HMMWV_VehicleReduced.cpp
    ../hmmwv/suspension/HMMWV_DoubleWishbone.h
    ../hmmwv/suspension/HMMWV_DoubleWishbone.cpp
    ../hmmwv/suspension/HMMWV_DoubleWishboneReduced.h
    ../hmmwv/suspension/HMMWV_DoubleWishboneReduced.cpp
    ../hmmwv/steering/HMMWV_PitmanArm.h
    ../hmmwv/steering/HMMWV_PitmanArm.cpp
    ../hmmwv/steering/HMMWV_RackPinion.h
    ../hmmwv/steering/HMMWV_RackPinion.cpp
    ../hmmwv/driveline/HMMWV_Driveline2WD.h
    ../hmmwv/driveline/HMMWV_Driveline2WD.cpp
    ../hmmwv/driveline/HMMWV_Driveline4WD.h
    ../hmmwv/driveline/HMMWV_Driveline4WD.cpp
    ../hmmwv/powertrain/HMMWV_SimplePowertrain.h
    ../hmmwv/powertrain/HMMWV_SimplePowertrain.cpp
    ../hmmwv/powertrain/HMMWV_Powertrain.h
    ../hmmwv/powertrain/HMMWV_Powertrain.cpp
    ../hmmwv/brake/HMMWV_BrakeSimple.h
    ../hmmwv/brake/HMMWV_BrakeSimple.cpp
    ../hmmwv/wheel/HMMWV_Wheel.h
    ../hmmwv/wheel/HMMWV_Wheel.cpp
    ../hmmwv/tire/HMMWV_RigidTire.h
    ../hmmwv/tire/HMMWV_RigidTire.cpp
    ../hmmwv/tire/HMMWV_LugreTire.h
    ../hmmwv/tire/HMMWV_LugreTire.cpp
    ../hmmwv/tire/HMMWV_FialaTire.h
    ../hmmwv/tire/HMMWV_FialaTire.cpp
)

SET(DEMO
    demo_VEH_HMMWV_DefSoil
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")
INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})

#--------------------------------------------------------------
# Add executable

MESSAGE(STATUS "...add ${DEMO}")

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO}
                      ChronoEngine
                      ChronoEngine_irrlicht
                      ChronoEngine_vehicle)
INSTALL(TARGETS ${DEMO} DESTINATION bin)
