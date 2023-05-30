if(NOT ENABLE_MODULE_IRRLICHT AND NOT_ENABLE_MODULE_VSG)
   return()
endif()   

#--------------------------------------------------------------
# List of all Parsers demos

# Add demos that can work with either Irrlicht or VSG,
# or no run-time visualization
set(DEMOS
)

# Add demos that can only work with Irrlicht
if(ENABLE_MODULE_IRRLICHT)
    set(DEMOS ${DEMOS}
      demo_PARSER_OpenSim
      demo_PARSER_Adams
    )
endif()

if(HAVE_URDF)
    set(DEMOS ${DEMOS}
      demo_PARSER_URDF
      demo_PARSER_URDF_RoboSimian
    )
endif()

if(HAVE_PYTHON)
    set(DEMOS ${DEMOS}
      demo_PARSER_Python
    )
    if(ENABLE_MODULE_IRRLICHT)
      set(DEMOS ${DEMOS}
        demo_PARSER_Python_SolidWorks
      )
    endif()
endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries
include_directories(${CH_INCLUDES})
set(COMPILER_FLAGS "${CH_CXX_FLAGS}")
set(LINKER_FLAGS "${CH_LINKERFLAG_EXE}")
list(APPEND LIBS "ChronoEngine")
list(APPEND LIBS "ChronoEngine_parsers")

if(ENABLE_MODULE_IRRLICHT)
  include_directories(${CH_IRRLICHT_INCLUDES})
  set(COMPILER_FLAGS "${COMPILER_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
  list(APPEND LIBS "ChronoEngine_irrlicht")
endif()

if(ENABLE_MODULE_VSG)
  set(CMAKE_CXX_STANDARD 17)

  include_directories(${CH_VSG_INCLUDES})
  list(APPEND LIBS "ChronoEngine_vsg")
endif()

#--------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
endif()

#--------------------------------------------------------------

# Add executables

message(STATUS "Demo programs for Parsers module...")

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos COMPILE_FLAGS "${COMPILER_FLAGS}" LINK_FLAGS "${LINKER_FLAGS}")
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBS})  

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
