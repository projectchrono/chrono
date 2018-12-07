#--------------------------------------------------------------
# List of all Godot demos

set(DEMOS
    demo_GD_collisionNSC
    )

#--------------------------------------------------------------
# Additional include paths

include_directories(${CH_GODOT_INCLUDES})

#--------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

#--------------------------------------------------------------
# Add executables

message(STATUS "Demo programs for GODOT module...")

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

# Comment this line if using a shared ChronoEngine_godot library 
  target_compile_options(${PROGRAM} PUBLIC ${CH_GODOT_COMPILE_OPTIONS})

  target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_godot)

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

endforeach()
