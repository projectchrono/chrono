#--------------------------------------------------------------

set(PY_DEMOS
    demo_PY_parser
    )

set(PY_DEMOS_IRR
    demo_PY_import_solidworks
    )

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_PYTHONINC} )


#--------------------------------------------------------------
# Add executables

MESSAGE(STATUS "Demo programs for PYTHON module...")

foreach(PROGRAM ${PY_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES  "${PROGRAM}.cpp")
    
    set_target_properties(${PROGRAM} PROPERTIES 
        FOLDER demos
        COMPILE_FLAGS "${CH_CXX_FLAGS}"
        LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")    
    target_link_libraries(${PROGRAM} ChronoEngine ChronoEngine_pyparser)
    add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_pyparser)

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()

if(ENABLE_MODULE_IRRLICHT)

    include_directories(${CH_IRRLICHT_INCLUDES})

    foreach(PROGRAM ${PY_DEMOS_IRR})
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
            ChronoEngine_pyparser
            ChronoEngine_irrlicht)
        add_dependencies(${PROGRAM} ChronoEngine ChronoEngine_pyparser ChronoEngine_irrlicht)
        
        install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
    endforeach()

endif()
