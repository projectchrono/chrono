# ------------------------------------------------------------------------------
# List of demos
# ------------------------------------------------------------------------------

set(DEMOS ${DEMOS}
   demo_FSI_DamBreak
   demo_FSI_Compressibility
   demo_FSI_Poiseuille_flow
   demo_FSI_SingleWheelTest
   demo_FSI_BCE
   demo_FSI_Kernels
   demo_FSI_ObjectDrop
   demo_FSI_CylindricalTank
   demo_FSI_WaveTank
   demo_FSI_EmbeddedObject
   demo_FSI_BaffleFlow
   demo_FSI_CouetteFlow
   demo_FSI_Flap
   demo_FSI_AngleRepose
   demo_FSI_Cratering
)

if(ENABLE_MODULE_PARDISO_MKL)
   set(DEMOS ${DEMOS}
      demo_FSI_Flexible_Plate
      demo_FSI_Flexible_Cable
      demo_FSI_Flexible_Toroidal_Tire
   )
endif()

#--------------------------------------------------------------

list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_VSG)
    list(APPEND LIBS "ChronoEngine_vsg")
endif()

if(ENABLE_MODULE_OPENGL)
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

list(APPEND LIBS "ChronoEngine_fsi_sph")

#--------------------------------------------------------------

# MESSAGE(STATUS "Demo programs for FSI module...")
	
foreach(PROGRAM ${DEMOS})
    # MESSAGE(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)
    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
