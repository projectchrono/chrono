# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_GRANULAR_INCLUDES})

SET(LIBRARIES
		ChronoEngine
		ChronoEngine_granular
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
        demo_GRAN_DamBreak
        metrics_GRAN_suite
        test_GRAN_repose
        metrics_GRAN_settling
        test_GRAN_directshear
        demo_GRAN_ballcosim
        utest_GRAN_testsuite
        test_GRAN_bulkcompress
        demo_GRAN_Bucket
        test_GRAN_mixer
        test_GRAN_couette
        test_GRAN_Coneflow
        demo_GRAN_ShearBand
        demo_GRAN_rover
        test_GRAN_CylinderLift
        metrics_GRAN_milsettle
        demo_GRAN_terrainBox_SMC
)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Demo programs for GRANULAR module...")

FOREACH(PROGRAM ${DEMOS})
		MESSAGE(STATUS "...add ${PROGRAM}")

		ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
		SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

		SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
				FOLDER demos
				COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_GRANULAR_CXX_FLAGS}"
				LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		)

		TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
		ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

		INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})

ENDFOREACH(PROGRAM)
