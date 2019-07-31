#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for VEHICLE module...")

ADD_SUBDIRECTORY(demo_HMMWV9)
ADD_SUBDIRECTORY(demo_HMMWV)
ADD_SUBDIRECTORY(demo_HMMWV_Parallel)
ADD_SUBDIRECTORY(demo_WheeledGeneric)
ADD_SUBDIRECTORY(demo_WheeledJSON)
ADD_SUBDIRECTORY(demo_SuspensionTest)
ADD_SUBDIRECTORY(demo_TireTest)
ADD_SUBDIRECTORY(demo_TireTest_Parallel)
ADD_SUBDIRECTORY(demo_TractorTrailer)
ADD_SUBDIRECTORY(demo_ArticulatedVehicle)
ADD_SUBDIRECTORY(demo_WheeledAssembly)
ADD_SUBDIRECTORY(demo_SteeringController)
ADD_SUBDIRECTORY(demo_TwoCars)
ADD_SUBDIRECTORY(demo_Sedan)
ADD_SUBDIRECTORY(demo_CityBus)

ADD_SUBDIRECTORY(demo_ISO2631)

ADD_SUBDIRECTORY(demo_UAZ)

ADD_SUBDIRECTORY(demo_DeformableSoil)
ADD_SUBDIRECTORY(demo_DeformableSoilAndTire)
ADD_SUBDIRECTORY(demo_GranularTerrain)

ADD_SUBDIRECTORY(demo_M113)
ADD_SUBDIRECTORY(demo_M113_Band)
ADD_SUBDIRECTORY(demo_M113_Parallel)
ADD_SUBDIRECTORY(demo_TrackedJSON)
ADD_SUBDIRECTORY(demo_TrackedJSON_Band)
ADD_SUBDIRECTORY(demo_TrackTestRig)
ADD_SUBDIRECTORY(demo_TrackTestRig_Band)

ADD_SUBDIRECTORY(demo_Paths)
ADD_SUBDIRECTORY(demo_RigidTerrain)
ADD_SUBDIRECTORY(demo_CRGTerrain)
