#=============================================================================
# CMake configuration file for the TwoCars demo.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

if(NOT ENABLE_MODULE_IRRLICHT AND NOT ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all model files for this demo

SET(PROGRAM demo_VEH_TwoCars)

#--------------------------------------------------------------
# Additional include directories

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHTINC})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBS "ChronoEngine_irrlicht")
    list(APPEND LIBS "ChronoEngine_vehicle_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
    set(CMAKE_CXX_STANDARD 17)

    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
    list(APPEND LIBS "ChronoEngine_vehicle_vsg")
endif()


#--------------------------------------------------------------
# Create the executable

message(STATUS "...add ${PROGRAM}")

add_executable(${PROGRAM} ${PROGRAM}.cpp ${MODEL_FILES})
source_group("" FILES ${PROGRAM}.cpp)

set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
if(APPLE)
   set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
endif()
set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
target_link_libraries(${PROGRAM} ${LIBS})

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
