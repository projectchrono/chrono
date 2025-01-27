#--------------------------------------------------------------
# List all FEA demos that do have no other dependencies
set(FEA_DEMOS
    demo_FEA_basic
    demo_FEA_dynamics
    demo_FEA_loads_statics
)

# Add demos that require IRRLICHT run-time visualization
if(ENABLE_MODULE_IRRLICHT)

    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_truss
    )

    if(ENABLE_MODULE_PARDISO_MKL)
    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_beams_static
        demo_FEA_beams_extrude
        demo_FEA_beamsIGA
        demo_FEA_cosimulate_load
    )
    endif()

endif()

# Add demos that require run-time visualization (IRRLICHT or VSG)
if(ENABLE_MODULE_IRRLICHT OR ENABLE_MODULE_VSG)

    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_thermal
        demo_FEA_electrostatics
        demo_FEA_contacts_SMC
        demo_FEA_cables
        demo_FEA_shellsANCF_3423
        demo_FEA_shellsANCF_3833
        demo_FEA_hexaANCF_3813
        demo_FEA_loads_dynamics
    )
    
    if(ENABLE_MODULE_MUMPS)
    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_cablesMUMPS
    )
    endif()

    if(ENABLE_MODULE_MATLAB)
    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_cablesMATLAB
    )
    endif()
    
    if(ENABLE_MODULE_PARDISO_MKL)
    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_beamsEuler
        demo_FEA_beams_constr
        demo_FEA_cablesMKL
        demo_FEA_abaqus_wheel
        demo_FEA_shellsReissner
        demo_FEA_shellsBST
        demo_FEA_contacts_NSC
        demo_FEA_contactsANCF_SMC
        demo_FEA_hexaANCF_3813_9
        demo_FEA_beamsANCF
    )
    endif()
    
    # Add demos that require MKL and postprocess
    if(ENABLE_MODULE_PARDISO_MKL AND ENABLE_MODULE_POSTPROCESS)
    set(FEA_DEMOS ${FEA_DEMOS}
        demo_FEA_beams_rotor
    )
    endif()

endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
  include_directories(${CH_VSG_INCLUDES})
  list(APPEND LIBS "ChronoEngine_vsg")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_MUMPS)
  list(APPEND LIBS "ChronoEngine_mumps")
endif()

if(ENABLE_MODULE_MATLAB)
  include_directories(${CH_MATLABINC})
  list(APPEND LIBS "ChronoEngine_matlab")
endif()

if(ENABLE_MODULE_POSTPROCESS)
  list(APPEND LIBS "ChronoEngine_postprocess")
endif()

#--------------------------------------------------------------

# MESSAGE(STATUS "Demo programs for FEA module...")

foreach(PROGRAM ${FEA_DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})
    
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)

