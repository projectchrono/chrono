if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT_CH_ENABLE_MODULE_VSG)
   return()
endif()   

#--------------------------------------------------------------
# List of all Parsers demos

set(DEMOS "")

# Add demos that can only work with Irrlicht
if(CH_ENABLE_MODULE_IRRLICHT)
    set(DEMOS ${DEMOS}
      demo_PARSER_OpenSim
      demo_PARSER_Adams
    )
endif()

if(HAVE_URDF AND NOT CH_STATIC)
    set(DEMOS ${DEMOS}
      demo_PARSER_URDF
      demo_PARSER_URDF_RoboSimian
    )
endif()

if(HAVE_PYTHON)
    set(DEMOS ${DEMOS}
      demo_PARSER_Python
    )
    if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_PYTHON)
      set(DEMOS ${DEMOS}
        demo_PARSER_Python_SolidWorks
      )
    endif()
endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries
list(APPEND LIBS Chrono_core)
list(APPEND LIBS "Chrono_parsers")

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS "Chrono_vsg")
endif()

#--------------------------------------------------------------

# Add executables

message(STATUS "Add demo programs for PARSERS module")

foreach(PROGRAM ${DEMOS})
  # MESSAGE(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
  if(APPLE)
    set_target_properties(${PROGRAM} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${MACOSX_BUNDLE_INFO_PLIST})
  endif()
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBS})

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
