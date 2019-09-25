#--------------------------------------------------------------
# List of all Irrlicht demos

SET(DEMOS
  demo_IRR_collisionNSC
  demo_IRR_bricks
  demo_IRR_cohesion
  demo_IRR_conveyor
  demo_IRR_crank
  demo_IRR_decomposition
  demo_IRR_forklift
  demo_IRR_fourbar
  demo_IRR_friction
  demo_IRR_gears
  demo_IRR_mecanum
  demo_IRR_pendulum
  demo_IRR_sph
  demo_IRR_suspension
  demo_IRR_tire
  demo_IRR_tracks
  demo_IRR_convergence
  demo_IRR_assets
  demo_IRR_soilbin
  demo_IRR_ballSMC
  demo_IRR_collisionSMC
  demo_IRR_emitter
  demo_IRR_emit_cluster
  demo_IRR_emit_processor
  demo_IRR_aux_ref
  demo_IRR_rev_sph
  demo_IRR_rev_trans
  demo_IRR_rev_limits
  demo_IRR_rot_spring
  demo_IRR_spring
  demo_IRR_ujoint
  demo_IRR_earthquake
  demo_IRR_paths
  demo_IRR_collision_2d
  demo_IRR_collision_trimesh
  demo_IRR_plane_plane
  demo_IRR_link_bushing
  demo_IRR_prismatic_limits
  demo_IRR_prismatic_force
  demo_IRR_callbackNSC
  demo_IRR_callbackSMC
  demo_IRR_motors
  demo_IRR_OpenSim_parser
  demo_IRR_report_collisions
  demo_IRR_Adams_parser
  demo_IRR_debugDrawer_collisionModels
)

#--------------------------------------------------------------
# Additional include paths

INCLUDE_DIRECTORIES( ${CH_IRRLICHTINC} )

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

# Add executables

MESSAGE(STATUS "Demo programs for IRRLICHT module...")

FOREACH(PROGRAM ${DEMOS})
  MESSAGE(STATUS "...add ${PROGRAM}")

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

  TARGET_LINK_LIBRARIES(${PROGRAM}
    ChronoEngine
    ChronoEngine_irrlicht
    )
    
  ADD_DEPENDENCIES (${PROGRAM} ChronoEngine ChronoEngine_irrlicht)

  INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH()

