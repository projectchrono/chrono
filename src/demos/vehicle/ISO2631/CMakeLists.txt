#=============================================================================
# CMake configuration file for the ISO2631 demos.
# These example programs work with and without Irrlicht run-time visualization.
#=============================================================================

#--------------------------------------------------------------
# List all ISO2631 demos

SET(DEMOS
    demo_VEH_Ride
    demo_VEH_Shock
)

#--------------------------------------------------------------
# Additional include directories

IF(ENABLE_MODULE_IRRLICHT)
    INCLUDE_DIRECTORIES(${CH_IRRLICHT_INCLUDES})
ENDIF()

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle)

IF(ENABLE_MODULE_IRRLICHT)
    SET(LIBRARIES_IRR 
        ChronoEngine_irrlicht
        ChronoEngine_vehicle_irrlicht)
ELSE()
    SET(LIBRARIES_IRR "")
ENDIF()

#--------------------------------------------------------------
# Create the executables

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
    SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES 
                          COMPILE_FLAGS "${MY_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES} ${LIBRARIES_IRR})

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH()
