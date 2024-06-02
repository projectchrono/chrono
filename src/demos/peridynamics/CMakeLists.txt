#--------------------------------------------------------------

# List of demos with no additional dependencies 
set(PERI_DEMOS
	)

# List of demos requiring the Chrono::Irrlicht module
set(PERI_DEMOS_IRR
    demo_PERI_elastic
	demo_PERI_fracture
	)

#--------------------------------------------------------------

message(STATUS "Demo programs for PERIDYNAMICS module...")

foreach(PROGRAM ${PERI_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    
    set_target_properties(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")    
    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_peridynamics)
    add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_peridynamics)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

if(ENABLE_MODULE_IRRLICHT)

    include_directories(${CH_IRRLICHT_INCLUDES})

    foreach(PROGRAM ${PERI_DEMOS_IRR})
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
            ChronoEngine_peridynamics
            ChronoEngine_postprocess
            ChronoEngine_irrlicht)
        add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_peridynamics ChronoEngine_postprocess ChronoEngine_irrlicht)

        install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    endforeach()

endif()


