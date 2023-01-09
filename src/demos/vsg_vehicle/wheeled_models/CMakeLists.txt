#--------------------------------------------------------------
# List of all OpenGL demos

SET(DEMOS
  demo_vsg_VEH_HMMWV_DefSoil
  demo_vsg_VEH_HMMWV_Accel
  demo_vsg_VEH_HMMWV_dlc
  demo_vsg_VEH_SuspensionTestRig
  demo_vsg_VEH_TireTestRig
  demo_vsg_VEH_TwoCars
  demo_vsg_VEH_RigidTerrain
  demo_vsg_VEH_WheeledJSON
)

if(HAVE_OPENCRG)
    set(DEMOS ${DEMOS}
        demo_vsg_VEH_CRGTerrain
    )
endif()

set(CMAKE_CXX_STANDARD 17)

#--- maybe it is not necessary to build app bundles with vsg
if(APPLE)
    set(CMAKE_MACOSX_BUNDLE OFF)
endif()

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES(${CH_VSG_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "")

if(ENABLE_MODULE_PARDISO_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for VSG module...")

FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
                        FOLDER demos
                        COMPILE_FLAGS "${COMPILER_FLAGS}"
                        LINK_FLAGS "${LINKERFLAG_EXE}")
  SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  TARGET_LINK_LIBRARIES(${PROGRAM}
                        ChronoEngine
                        ChronoEngine_vehicle
                        ChronoEngine_vehicle_vsg
                        ChronoModels_vehicle
                        ChronoEngine_vsg
                        ${LIBS})

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
