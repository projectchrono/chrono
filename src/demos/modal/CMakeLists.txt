#--------------------------------------------------------------
# List all MODAL demos 

set(DEMOS
    demo_MOD_slewing_beam
    )

# Add demos that require Irrlicht
if(CH_ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_MOD_analysis
       demo_MOD_reduction
   )
endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries
set(LIBS Chrono_core Chrono_modal)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS Chrono_irrlicht)
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS Chrono_pardisomkl)
endif()

if(CH_ENABLE_MODULE_MATLAB)
  list(APPEND LIBS Chrono_matlab)
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  list(APPEND LIBS Chrono_postprocess)
endif()

#--------------------------------------------------------------

message(STATUS "Add demo programs for MODAL module")

foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    
    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)

