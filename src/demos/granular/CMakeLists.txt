# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------
ADD_SUBDIRECTORY(robosimian)

INCLUDE_DIRECTORIES(${CH_GRANULAR_INCLUDES})

SET(LIBRARIES
		ChronoEngine
		ChronoEngine_granular
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# Programs that can work with or without OpenGL
SET(DEMOS
		demo_GRAN_BoxSettlNoFric_SMC
		demo_GRAN_PoiseuilleCylSMC
		demo_GRAN_TriMeshNoFric_SMC
		demo_GRAN_TriMeshNoFric_SMC_tiredrag
		demo_GRAN_TriMeshNoFric_SMC_ballcosim
		test_gran_milwave
		test_gran_milsettle
		utest_GRAN_TriMeshNoFric_SMC_boxroll
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


