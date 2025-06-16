#--------------------------------------------------------------
# Add PERIDYNAMICS demos that do have no other dependencies
set(PERI_DEMOS
)

# Add demos that require Irrlicht  and  postprocess
if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_POSTPROCESS)
set(PERI_DEMOS ${PERI_DEMOS}
    demo_PERI_elastic
	demo_PERI_fracture
    demo_PERI_implicit
)
endif()


# Add demos that require Irrlicht and  postprocess and MKL
if(CH_ENABLE_MODULE_IRRLICHT  AND CH_ENABLE_MODULE_POSTPROCESS AND CH_ENABLE_MODULE_PARDISO_MKL)
set(PERI_DEMOS ${PERI_DEMOS}
    # demo_PERI_benchmark
)
endif()


#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_peridynamics)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_POSTPROCESS)
  list(APPEND LIBS "Chrono_postprocess")
endif()


#--------------------------------------------------------------

message(STATUS "Add demo programs for PERIDYNAMICS module...")

foreach(PROGRAM ${PERI_DEMOS})
    message(STATUS "   ...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})
    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()
    
    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)


