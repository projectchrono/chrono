# ------------------------------------------------------------------------------
# List of demos according their dependencies
# ------------------------------------------------------------------------------

if (NOT USE_FAST_DDS)
	return()
endif()

set(DEMOS
	# demo_SYN_dds
	# demo_SYN_mpi_dds
)

message(STATUS "SynChrono Libraries: ${SYN_LIBRARIES}")

include_directories(${SYN_INCLUDES})

foreach(PROGRAM ${DEMOS})
	message(STATUS "...add ${PROGRAM}")

	add_executable(${PROGRAM} ${PROGRAM}.cpp)

	set_target_properties(${PROGRAM} PROPERTIES
		    COMPILE_FLAGS "${SYN_CXX_FLAGS}"
				LINK_FLAGS "${CH_LINKERFLAG_EXE} ${MPI_LINK_FLAGS}")

	target_link_libraries(${PROGRAM}
							ChronoEngine_synchrono
							${SYN_LIBRARIES}
							)
endforeach()
