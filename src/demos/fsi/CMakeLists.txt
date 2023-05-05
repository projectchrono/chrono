# ------------------------------------------------------------------------------
# List of demos
# ------------------------------------------------------------------------------

set(DEMOS ${DEMOS}
   demo_FSI_DamBreak
   demo_FSI_Compressibility
   demo_FSI_Poiseuille_flow
   demo_FSI_SingleWheelTest
   demo_FSI_BCE
   demo_FSI_CylinderDrop
)

if(ENABLE_MODULE_PARDISO_MKL)
   set(DEMOS ${DEMOS}
      demo_FSI_Flexible_Flat_Plate
      demo_FSI_Flexible_Cable
      demo_FSI_Flexible_Toroidal_Tire
   )
endif()

#--------------------------------------------------------------

include_directories(${CH_FSI_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

if(ENABLE_MODULE_VSG)
    set(CMAKE_CXX_STANDARD 17)
    include_directories(${CH_VSG_INCLUDES})
    list(APPEND LIBS "ChronoEngine_vsg")
endif()

if(ENABLE_MODULE_OPENGL)
    include_directories(${CH_OPENGL_INCLUDES})
    list(APPEND LIBS "ChronoEngine_opengl")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
    include_directories(${CH_MKL_INCLUDES})
    set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
    set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
    list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

list(APPEND LIBS "ChronoEngine_fsi")

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for FSI module...")
	
foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
