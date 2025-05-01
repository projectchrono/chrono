#--------------------------------------------------------------
# Add PERIDYNAMICS demos that do have no other dependencies
set(PERI_DEMOS
)

# Add demos that require Irrlicht  and  postprocess
if(ENABLE_MODULE_IRRLICHT AND ENABLE_MODULE_POSTPROCESS)
set(PERI_DEMOS ${PERI_DEMOS}
    demo_PERI_elastic
	demo_PERI_fracture
    demo_PERI_implicit
)
endif()


# Add demos that require Irrlicht and  postprocess and MKL
if(ENABLE_MODULE_IRRLICHT  AND ENABLE_MODULE_POSTPROCESS AND ENABLE_MODULE_PARDISO_MKL)
set(PERI_DEMOS ${PERI_DEMOS}
    # demo_PERI_benchmark
)
endif()


#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CH_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_peridynamics")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHT_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_PARDISO_MKL)
  include_directories(${CH_MKL_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_MKL_CXX_FLAGS}")
  set(LINKER_FLAGS "${LINKER_FLAGS} ${CH_MKL_LINK_FLAGS}")
  list(APPEND LIBS "ChronoEngine_pardisomkl")
endif()

if(ENABLE_MODULE_POSTPROCESS)
  list(APPEND LIBS "ChronoEngine_postprocess")
endif()

#--------------------------------------------------------------

message(STATUS "Demo programs for PERIDYNAMICS module...")

foreach(PROGRAM ${PERI_DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}")
    if(APPLE)
      set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
    endif()
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})
    
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)


