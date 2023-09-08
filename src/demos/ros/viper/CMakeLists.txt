# Demo programs for simulations with the viper model with the Chrono::ROS library
# ------------------------------------------------------------------------------

message(STATUS "Demo programs for Chrono::ROS and Viper...")

# ------------------------------------------------------------------------------

set(DEMOS
    demo_ROS_viper
)

# ------------------------------------------------------------------------------
# Include directories, compiler flags, libraries

set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_ros")
list(APPEND LIBS "ChronoModels_robot")

# Force C++17, required by ROS Humble
set(CMAKE_CXX_STANDARD 17)

if(ENABLE_MODULE_IRRLICHT)
    include_directories(${CH_IRRLICHT_INCLUDES})
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

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

if(ENABLE_MODULE_SENSOR)
  include_directories(${CH_SENSOR_INCLUDES})
  list(APPEND LIBS "ChronoEngine_sensor")
endif()

# ------------------------------------------------------------------------------
# Find the custom viper_msgs
# NOTE: run colcon build and source the setup files to have this work

find_package(viper_msgs)
if (NOT viper_msgs_FOUND)
    message(WARNING "viper_msgs package wasn't found. Make sure you have run colcon build and sourced the setup file.")
    return()
endif()
include_directories(${viper_msgs_INCLUDE_DIRS})
list(APPEND LIBS "${viper_msgs_LIBRARIES}")

#--------------------------------------------------------------
# OS-specific settings

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

#--------------------------------------------------------------
# Set the handler files to include

set(HANDLER_FILES
  ViperDCMotorControlHandler.h
  ViperDCMotorControlHandler.cpp
  ViperHandler.h
  ViperHandler.cpp
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp ${HANDLER_FILES})
    source_group("" FILES ${PROGRAM}.cpp ${HANDLER_FILES})

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
