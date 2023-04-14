#--------------------------------------------------------------
# List all MODAL demos 

set(DEMOS "")

# Add demos that require Irrlicht
if(ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_MOD_analysis
       demo_MOD_assembly
   )
endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${CH_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")

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

if(ENABLE_MODULE_MATLAB)
  include_directories(${CH_MATLABINC})
  list(APPEND LIBS "ChronoEngine_matlab")
endif()

if(ENABLE_MODULE_PARDISO_PROJECT)
  include_directories(${CH_PARDISOPROJECT_INCLUDES})
  list(APPEND LIBS "ChronoEngine_pardisoproject")
  list(APPEND LIBS ${PARDISOPROJECT_LIBRARIES})
endif()

if(ENABLE_MODULE_POSTPROCESS)
  list(APPEND LIBS "ChronoEngine_postprocess")
endif()

#--------------------------------------------------------------

message(STATUS "Demo programs for MODAL module...")

foreach(PROGRAM ${DEMOS})
    message(STATUS "...add ${PROGRAM}")

    add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    source_group(""  FILES "${PROGRAM}.cpp")

    set_target_properties(${PROGRAM} PROPERTIES
        FOLDER demos
        COMPILE_FLAGS "${COMPILER_FLAGS}"
        LINK_FLAGS "${LINKER_FLAGS}")
    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    target_link_libraries(${PROGRAM} ${LIBS})
    
    install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach(PROGRAM)

