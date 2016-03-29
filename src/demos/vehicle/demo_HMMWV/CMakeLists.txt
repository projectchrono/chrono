#=============================================================================
# CMake configuration file for the HMMWV demo.
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
    demo_VEH_HMMWV
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/src/demos/vehicle")

IF(ENABLE_MODULE_IRRLICHT)
  INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle)

IF(ENABLE_MODULE_IRRLICHT)
  SET(LIBRARIES_IRR
      ChronoEngine_irrlicht
      ${CH_IRRLICHTLIB})
ELSE()
  SET(LIBRARIES_IRR "")
ENDIF()

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${DEMO}")

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${MY_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES} ${LIBRARIES_IRR})
INSTALL(TARGETS ${DEMO} DESTINATION bin)
