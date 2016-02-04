#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for VEHICLE module...")

#ADD_SUBDIRECTORY(slider_crank)
ADD_SUBDIRECTORY(demo_HMMWV9)
ADD_SUBDIRECTORY(demo_HMMWV)
ADD_SUBDIRECTORY(demo_GenericVehicle)
ADD_SUBDIRECTORY(demo_Vehicle)
ADD_SUBDIRECTORY(demo_SuspensionTest)
ADD_SUBDIRECTORY(demo_ArticulatedVehicle)
ADD_SUBDIRECTORY(demo_VehicleAssembly)
ADD_SUBDIRECTORY(demo_SteeringController)
ADD_SUBDIRECTORY(demo_TireTestRig)
ADD_SUBDIRECTORY(demo_DeformableSoil)
