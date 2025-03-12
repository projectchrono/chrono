#--------------------------------------------------------------
# List of all Irrlicht demos

set(DEMOS
  demo_IRR_decomposition
  demo_IRR_assets
  demo_IRR_paths
  demo_IRR_visualize_collision
  demo_IRR_visualize_FEA
)

if(CHRONO_THRUST_FOUND)
  set(DEMOS ${DEMOS}
    demo_IRR_raycast_test
  )
endif()

#--------------------------------------------------------------

# Add executables

# MESSAGE(STATUS "Demo programs for IRRLICHT module...")

foreach(PROGRAM ${DEMOS})
  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} Chrono_core Chrono_irrlicht)    

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
