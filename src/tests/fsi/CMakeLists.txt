# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

# List all FSI TESTS that use chrono
SET(FSI_TESTS
# add generic fsi TESTS here
)

# List all FSI TESTS that use chrono-parallel
SET(FSI_PARALLEL_TESTS
# add fsi parallel TESTS here
)

# List all FSI TESTS use vehicle
SET(FSI_VEHICLE_TESTS
# add fsi vehicle TESTS here
)

# List all FSI TESTS use parallel and vehicle
SET(FSI_PARALLEL_VEHICLE_TESTS
# add fsi parallel vehicle TESTS here
)

# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Additional test programs for FSI module...")

INCLUDE_DIRECTORIES(${CH_PARALLEL_INCLUDES})

SET(LIBRARIES
    ChronoEngine   
    ChronoEngine_fsi
)

IF(ENABLE_MODULE_PARALLEL)
SET(LIBRARIES ${LIBRARIES} 
    ChronoEngine_parallel
)
ENDIF()

IF(ENABLE_MODULE_OPENGL)
INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)
ENDIF()

IF(ENABLE_MODULE_VEHICLE)
SET(LIBRARIES ${LIBRARIES}
    ChronoEngine_vehicle   
)
ENDIF()

IF(ENABLE_MODULE_OPENGL)
    INCLUDE_DIRECTORIES(${CH_OPENGL_INCLUDES})
    SET(LIBRARIES ${LIBRARIES} ChronoEngine_opengl)
ENDIF()

#--------------------------------------------------------------
# Add executables for TESTS that have no other dependencies
IF(ENABLE_MODULE_PARALLEL)
	IF(ENABLE_MODULE_VEHICLE)
		FOREACH(PROGRAM ${FSI_PARALLEL_VEHICLE_TESTS})
		    MESSAGE(STATUS "...add ${PROGRAM}")

		    CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
		    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

		    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
			FOLDER demos
			COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		    )

		    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
		    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

		    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

		ENDFOREACH(PROGRAM)
	ELSE()
		FOREACH(PROGRAM ${FSI_PARALLEL_TESTS})
		    MESSAGE(STATUS "...add ${PROGRAM}")

		    CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
		    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

		    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
			FOLDER demos
			COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		    )

		    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
		    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

		    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

		ENDFOREACH(PROGRAM)
	ENDIF()
ELSE()
	IF(ENABLE_MODULE_VEHICLE)
		FOREACH(PROGRAM ${FSI_VEHICLE_TESTS})
		    MESSAGE(STATUS "...add ${PROGRAM}")

		    CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
		    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

		    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
			FOLDER demos
			COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		    )

		    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
		    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

		    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

		ENDFOREACH(PROGRAM)
	ELSE()
		FOREACH(PROGRAM ${FSI_TESTS})
		    MESSAGE(STATUS "...add ${PROGRAM}")

		    CUDA_ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
		    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

		    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
			FOLDER demos
			COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_PARALLEL_CXX_FLAGS}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE}"
		    )

		    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
		    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

		    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

		ENDFOREACH(PROGRAM)
	ENDIF()
ENDIF()




