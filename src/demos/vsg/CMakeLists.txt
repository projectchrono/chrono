#--------------------------------------------------------------
# List of all VSG demos

set(DEMOS
    demo_VSG_assets
    demo_VSG_paths
    demo_VSG_shapes
    demo_VSG_helmet
    demo_VSG_vehicles
    demo_VSG_visualize_FEA
)

#--- maybe it is not necessary to build app bundles with vsg
if(APPLE)
    set(CMAKE_MACOSX_BUNDLE OFF)
endif()

#--------------------------------------------------------------
# Additional include paths

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_vsg")


#--------------------------------------------------------------

# Add executables

# MESSAGE(STATUS "Demo programs for VSG module...")

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
