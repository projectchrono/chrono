message(STATUS "Add demo programs for FSI-TDPF module")

set(DEMOS ${DEMOS}
   demo_FSI-TDPF_sphere_decay
   demo_FSI-TDPF_sphere_reg_waves
   demo_FSI-TDPF_sphere_irreg_waves
   ##demo_FSI-TDPF_floating_platform
)

#--------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_fsitdpf)

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBS Chrono_vsg)
    list(APPEND LIBS Chrono_fsitdpf_vsg)
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
    list(APPEND LIBS Chrono_pardisomkl)
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS Chrono_mumps)
endif()

#--------------------------------------------------------------

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()

    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    target_include_directories(${PROGRAM} PRIVATE "${CH_HYDROCHRONO_DIR}/include")

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
