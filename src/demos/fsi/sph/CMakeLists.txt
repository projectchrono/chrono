# ------------------------------------------------------------------------------
# List of demos
# ------------------------------------------------------------------------------

set(DEMOS ${DEMOS}
   demo_FSI_DamBreak
   demo_FSI_Compressibility
   demo_FSI_Poiseuille_flow
   demo_FSI_SingleWheelTest
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
   demo_FSI_RassorDrum
   demo_FSI_NormalBevameter
   demo_FSI_ConePenetrometer
   demo_FSI_Hydrostatic
   demo_FSI_Cone_Drop
)

if(CH_ENABLE_MODULE_VSG)
  set(DEMOS ${DEMOS}
      demo_FSI_BCE
  )      
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
   set(DEMOS ${DEMOS}
      demo_FSI_FEAdirections
      demo_FSI_Flexible_Plate
      demo_FSI_Flexible_Cable
      demo_FSI_Flexible_Toroidal_Tire
   )
endif()

#--------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_fsisph")

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS "Chrono_fsisph_vsg")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS "Chrono_pardisomkl")
endif()

#--------------------------------------------------------------

message(STATUS "Add demo programs for FSI module")
	
foreach(PROGRAM ${DEMOS})
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

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
