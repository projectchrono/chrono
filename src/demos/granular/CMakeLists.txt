# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(robosimian)
ADD_SUBDIRECTORY(HMMWV)

INCLUDE_DIRECTORIES(${CH_GRANULAR_INCLUDES})

SET(LIBRARIES
		ChronoEngine
		ChronoEngine_granular
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
		demo_GRAN_BoxSettle_SMC
		demo_GRAN_terrainBox_SMC
		demo_GRAN_Bucket
		demo_GRAN_TriMeshNoFric_SMC
		demo_GRAN_TriMeshNoFric_SMC_tiredrag
		metrics_GRAN_ballcosim
		test_GRAN_milwave
		test_GRAN_milsettle
		utest_GRAN_TriMeshNoFric_SMC_boxroll
		utest_GRAN_meshstep
		test_GRAN_Coneflow
		test_GRAN_HopperReposeAngle
		utest_GRAN_spherefric
		demo_GRAN_DamBreak
		test_GRAN_meshforce
		test_GRAN_hopperflow
		test_GRAN_mixer
		test_GRAN_bulkcompress
		test_GRAN_directshear
		test_GRAN_repose
		metrics_GRAN_settling
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
