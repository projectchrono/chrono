# ------------------------------------------------------------------------------
# List of demos according their dependencies
# ------------------------------------------------------------------------------

set(DEMOS
	demo_SYN_scm
	demo_SYN_scm_tracked
)

#--------------------------------------------------------------

if(USE_FAST_DDS)
  find_package(fastcdr REQUIRED)
  find_package(fastrtps REQUIRED)
endif()

include_directories(${SYN_INCLUDES})

foreach(PROGRAM ${DEMOS})
	message(STATUS "...add ${PROGRAM}")

	add_executable(${PROGRAM} ${PROGRAM}.cpp)
	source_group("" FILES ${PROGRAM}.cpp)

	set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${SYN_CXX_FLAGS}")

	target_link_libraries(${PROGRAM} ChronoEngine_synchrono)
endforeach()
