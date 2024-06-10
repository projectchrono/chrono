#--------------------------------------------------------------

# List of demos with no additional dependencies 
set(MULTIDOMAIN_DEMOS
	)

# List of demos requiring the Chrono::Irrlicht module
set(MULTIDOMAIN_DEMOS_IRR
	demo_MDOM_simple
	)

#--------------------------------------------------------------

message(STATUS "Demo programs for MULTIDOMAIN module...")

foreach(PROGRAM ${MULTIDOMAIN_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    
    set_target_properties(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")    
    target_link_libraries(${PROGRAM} 
		ChronoEngine 
		ChronoEngine_multidomain)
    add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_multidomain)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

if(ENABLE_MODULE_IRRLICHT)

    include_directories(${CH_IRRLICHT_INCLUDES})

    foreach(PROGRAM ${MULTIDOMAIN_DEMOS_IRR})
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
            ChronoEngine_multidomain
            ChronoEngine_irrlicht)
        add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_multidomain ChronoEngine_irrlicht)

        install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    endforeach()

endif()


