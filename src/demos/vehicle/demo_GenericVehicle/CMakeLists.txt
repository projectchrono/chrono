# Works with and without Irrlicht run-time visualization
# ------------------------------------------------------

MESSAGE(STATUS "...add GENERIC_VEHICLE demo")

SET(MODEL_FILES
	../ModelDefs.h
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
	../generic/Generic_AntirollBarRSD.h
)

SET(DEMO_FILES
	demo_GenericVehicle.cpp
)

SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${DEMO_FILES})

# Create the executable
INCLUDE_DIRECTORIES(${CH_VEHICLE_INCLUDES})
ADD_EXECUTABLE(demo_GenericVehicle ${DEMO_FILES} ${MODEL_FILES})
SET_TARGET_PROPERTIES(demo_GenericVehicle PROPERTIES 
                      COMPILE_FLAGS "${CH_BUILDFLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(demo_GenericVehicle ${CH_VEHICLE_LIBRARIES})
INSTALL(TARGETS demo_GenericVehicle DESTINATION bin)

