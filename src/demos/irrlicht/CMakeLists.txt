#--------------------------------------------------------------
# List of all Irrlicht demos

SET(DEMOS
  demo_collision
  demo_benchmark
  demo_bricks
  demo_cohesion
  demo_conveyor
  demo_crank
  demo_decomposition
  demo_forklift
  demo_fourbar
  demo_friction
  demo_gears
  demo_mecanum
  demo_pendulum
  demo_sph
  demo_suspension
  demo_tire
  demo_tracks
  demo_convergence
  demo_irr_assets
  demo_soilbin
  demo_ballDEM
  demo_collisionDEM
  demo_emitter
  demo_emit_cluster
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
FOREACH(PROGRAM ${DEMOS})

  ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")

  SOURCE_GROUP(""  FILES  "${PROGRAM}.cpp")

  SET_TARGET_PROPERTIES(${PROGRAM}  PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${CH_BUILDFLAGS}"
    LINK_FLAGS "${CH_LINKERFLAG_EXE}"
    )

  TARGET_LINK_LIBRARIES(${PROGRAM}
    ${CH_IRRLICHTLIB}
    ChronoEngine
    )

  ADD_DEPENDENCIES (${PROGRAM} ChronoEngine)

  INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH()

