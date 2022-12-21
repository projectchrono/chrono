#--------------------------------------------------------------

# List of demos with no additional dependencies 
set(POST_DEMOS
	demo_POST_povray1
    demo_POST_povray2
	demo_POST_gnuplot
	demo_POST_timestepping
	)

# List of demos requiring the Chrono::Irrlicht module
set(POST_DEMOS_IRR
	demo_POST_emit_creation
	demo_POST_emitter_asset
	)

#--------------------------------------------------------------

message(STATUS "Demo programs for POSTPROCESS module...")

foreach(PROGRAM ${POST_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    
    set_target_properties(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")    
    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_postprocess)
    add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_postprocess)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

if(ENABLE_MODULE_IRRLICHT)

    include_directories(${CH_IRRLICHTINC})

    foreach(PROGRAM ${POST_DEMOS_IRR})
        message(STATUS "...add ${PROGRAM}")

        add_executable(${PROGRAM}  "${PROGRAM}.cpp")
        source_group(""  FILES  "${PROGRAM}.cpp")
        set_target_properties(${PROGRAM} PROPERTIES 
            FOLDER demos
            COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
            LINK_FLAGS "${CH_LINKERFLAG_EXE}")
        set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
        target_link_libraries(${PROGRAM} 
        	ChronoEngine
            ChronoEngine_postprocess
            ChronoEngine_irrlicht)
        add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_postprocess ChronoEngine_irrlicht)

        install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    endforeach()

endif()


