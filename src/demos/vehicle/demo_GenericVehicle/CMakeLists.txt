#=============================================================================
# CMake configuration file for the GENERIC_VEHICLE demo.
# This example program works with and without Irrlicht run-time visualization
#=============================================================================

MESSAGE(STATUS "...add GENERIC_VEHICLE demo")

#--------------------------------------------------------------
# List all model files for this demo

SET(MODEL_FILES
    ../generic/Generic_Wheel.h
    ../generic/Generic_FuncDriver.h
    ../generic/Generic_Vehicle.h
    ../generic/Generic_Vehicle.cpp
    ../generic/Generic_SolidAxle.h
    ../generic/Generic_SolidAxle.cpp
    ../generic/Generic_MultiLink.h
    ../generic/Generic_MultiLink.cpp
    ../generic/Generic_RackPinion.h
    ../generic/Generic_Driveline2WD.h
    ../generic/Generic_SimplePowertrain.h
    ../generic/Generic_BrakeSimple.h
    ../generic/Generic_RigidTire.h
    ../generic/Generic_DoubleWishbone.h
    ../generic/Generic_DoubleWishbone.cpp
    ../generic/Generic_HendricksonPRIMAXX.h
    ../generic/Generic_HendricksonPRIMAXX.cpp
    ../generic/Generic_MacPhersonStrut.h
    ../generic/Generic_MacPhersonStrut.cpp
    ../generic/Generic_AntirollBarRSD.h
)

SET(DEMO_FILES
    demo_GenericVehicle.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

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

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

ADD_EXECUTABLE(demo_GenericVehicle ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_GenericVehicle PROPERTIES 
                      COMPILE_FLAGS "${MY_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_GenericVehicle ${LIBRARIES} ${LIBRARIES_IRR})
INSTALL(TARGETS demo_GenericVehicle DESTINATION bin)
