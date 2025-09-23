if(NOT CH_ENABLE_MODULE_IRRLICHT AND NOT CH_ENABLE_MODULE_VSG)
    return()
endif()

#--------------------------------------------------------------
# List of all MBS demos

# Add demos that can work with either Irrlicht or VSG
set(DEMOS
  demo_MBS_aux_ref
  demo_MBS_object_drop
  demo_MBS_bricks
  demo_MBS_callbackNSC
  demo_MBS_callbackSMC
  demo_MBS_collisionNSC
  demo_MBS_collisionSMC
  demo_MBS_earthquake
  demo_MBS_friction
  demo_MBS_motors
  demo_MBS_spring  
  demo_MBS_inverted_pendulum
  demo_MBS_external_ODE
  demo_MBS_external_DAE  
  demo_MBS_hydraulic_actuator
  demo_MBS_hydraulic_crane
  demo_MBS_hydraulic_crane_cosim
  demo_MBS_custom_contact
)

# Add demos that can only work with Irrlicht
if(CH_ENABLE_MODULE_IRRLICHT)
    set(DEMOS ${DEMOS}
      demo_MBS_cohesion
      demo_MBS_conveyor
      demo_MBS_feeder
      demo_MBS_crank
      demo_MBS_forklift
      demo_MBS_fourbar
      demo_MBS_gears
      demo_MBS_mecanum
      demo_MBS_pendulum
      demo_MBS_suspension
      demo_MBS_tracks
      demo_MBS_convergence
      demo_MBS_collision_trimesh
      demo_MBS_collision_2d
      demo_MBS_emitter
      demo_MBS_emit_cluster
      demo_MBS_emit_processor
      demo_MBS_rev_sph
      demo_MBS_rev_trans
      demo_MBS_rev_limits
      demo_MBS_rev_reactions
      demo_MBS_rot_spring
      demo_MBS_ujoint
      demo_MBS_planar
      demo_MBS_link_bushing
      demo_MBS_prismatic_limits
      demo_MBS_prismatic_force
      demo_MBS_report_collisions
      demo_MBS_imposed_motion
    )

    if(CHRONO_THRUST_FOUND)
      set(DEMOS ${DEMOS}
        demo_MBS_collision_object
      )
    endif()

    if(CH_ENABLE_MODULE_PARDISO_MKL)
      set(DEMOS ${DEMOS}
        demo_MBS_link_mate
      )
    endif()
endif()

#--------------------------------------------------------------

# Set include paths, compiler & linker flags, and libraries

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND LIBS Chrono_core)

if(CH_ENABLE_MODULE_IRRLICHT)
  list(APPEND LIBS "Chrono_irrlicht")
endif()

if(CH_ENABLE_MODULE_PARDISO_MKL)
  list(APPEND LIBS "Chrono_pardisomkl")
endif()

if(CH_ENABLE_MODULE_VSG)
  list(APPEND LIBS "Chrono_vsg")
endif()

if(CH_ENABLE_MODULE_MUMPS)
  list(APPEND LIBS "Chrono_mumps")
endif()

#--------------------------------------------------------------

# Add executables

message(STATUS "Add demo programs for MBS module")

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


  install(TARGETS ${PROGRAM} CONFIGURATIONS Release CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()
