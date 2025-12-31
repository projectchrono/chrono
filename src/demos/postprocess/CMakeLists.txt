#--------------------------------------------------------------

# List of demos with no additional dependencies 
set(POST_DEMOS
	demo_POST_povray1
    demo_POST_povray2
    demo_POST_blender
	demo_POST_gnuplot
	demo_POST_timestepping
	)

# List of demos requiring the Chrono::Irrlicht module
set(POST_DEMOS_IRR
	demo_POST_emit_creation
	demo_POST_emitter_asset
	)

#--------------------------------------------------------------

# MESSAGE(STATUS "Demo programs for POSTPROCESS module...")

foreach(PROGRAM ${POST_DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    
    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")    
    target_link_libraries(${PROGRAM} Chrono_core Chrono_postprocess)
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()

if(CH_ENABLE_MODULE_IRRLICHT)

    foreach(PROGRAM ${POST_DEMOS_IRR})
        # MESSAGE(STATUS "...add ${PROGRAM}")

        add_executable(${PROGRAM}  "${PROGRAM}.cpp")
        source_group(""  FILES  "${PROGRAM}.cpp")
        set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
        set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
        target_link_libraries(${PROGRAM} 
        	Chrono_core
            Chrono_postprocess
            Chrono_irrlicht)
        
        if(MSVC)
          set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
        endif()

        install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
    endforeach()

endif()


