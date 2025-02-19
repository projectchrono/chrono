# ------------------------------------------------------------------------------
# List of demos according their dependencies
# ------------------------------------------------------------------------------

set(DEMOS
    demo_SYN_scm
    demo_SYN_scm_tracked
)

if(CH_ENABLE_MODULE_IRRLICHT)
   set(DEMOS ${DEMOS}
       demo_SYN_wheeled
       demo_SYN_tracked
       demo_SYN_copter
       demo_SYN_highway
   )
endif()

#--------------------------------------------------------------

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_vehicle)
list(APPEND LIBS Chrono_synchrono)
list(APPEND LIBS ChronoModels_vehicle)
list(APPEND LIBS ChronoModels_robot)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS Chrono_irrlicht)
endif()

foreach(PROGRAM ${DEMOS})
  # MESSAGE(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  source_group("" FILES ${PROGRAM}.cpp)

  set_target_properties(${PROGRAM} PROPERTIES FOLDER demos)
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")

  target_link_libraries(${PROGRAM} ${LIBS})

  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  install(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
endforeach()
