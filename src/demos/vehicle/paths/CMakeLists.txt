#=============================================================================
# CMake configuration file for the vehicle path demo.
# Requires Chrono:Postprocess for Gnuplot visualization.
#=============================================================================

IF(NOT CH_ENABLE_MODULE_POSTPROCESS)
    RETURN()
ENDIF()

set(PROGRAM demo_VEH_paths)

#--------------------------------------------------------------
# Add executable

# MESSAGE(STATUS "...add ${PROGRAM}")

ADD_EXECUTABLE(${PROGRAM} ${PROGRAM}.cpp)
SOURCE_GROUP("" FILES ${PROGRAM}.cpp)

SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
TARGET_LINK_LIBRARIES(${PROGRAM}
                      Chrono_vehicle
                      Chrono_postprocess)

if(MSVC)
  set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
