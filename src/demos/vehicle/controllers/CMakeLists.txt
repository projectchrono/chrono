#=============================================================================
# CMake configuration file for demos f controllers in Chrono::Vehicle.
# This example program requires Irrlicht run-time visualization.
#=============================================================================

if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List all demos

set(DEMOS
    demo_VEH_Controller_DLC
)

if(CH_ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_VEH_Controller_AIDriver
  )
endif()

#--------------------------------------------------------------
# Include directories, compiler flags, libraries

list(APPEND LIBS "Chrono_vehicle")
list(APPEND LIBS "ChronoModels_vehicle")

if(CH_ENABLE_MODULE_IRRLICHT)
    list(APPEND LIBS "Chrono_vehicle_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_vehicle_vsg")
endif()

if(CH_ENABLE_MODULE_SENSOR)
    list(APPEND LIBS "Chrono_sensor")
endif()

if (CH_ENABLE_MODULE_SENSOR)
    list(APPEND LIBS "Chrono_sensor")
endif()

#--------------------------------------------------------------
# STB files

set(STB_FILES
   ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.h
   ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image.cpp
   ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.h
   ${CMAKE_SOURCE_DIR}/src/chrono_thirdparty/stb/stb_image_write.cpp
)

#--------------------------------------------------------------
# Create the executables

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
