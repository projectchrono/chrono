# ------------------------------------------------------------------------------
# List of demos according their dependencies
# ------------------------------------------------------------------------------

set(PARKST_DIRECTORY "$ENV{HOME}/Continental/MeshExport" CACHE FILEPATH "Directory location of the Park St. assets")

set(PARKST_COL_MESH_PATH "${PARKST_DIRECTORY}/park_street_col_2.obj" CACHE FILEPATH "Location on disk of the Park St. collision mesh" FORCE)
set(PARKST_VIS_MESH_PATH "${PARKST_DIRECTORY}/park_street_vis_split.obj" CACHE FILEPATH "Location on disk of the Park St. visualization mesh" FORCE)

if(NOT EXISTS ${PARKST_COL_MESH_PATH} OR NOT EXISTS ${PARKST_VIS_MESH_PATH})
	# message(STATUS "PARKST_COL_MESH_PATH and/or PARKST_VIS_MESH_PATH not found. Not building parkst demos")
	set(PARKST_DIRECTORY "PARKST_DIRECTORY-NOTFOUND" CACHE FILEPATH "Directory location of the Park St. assets" FORCE)
	set(PARKST_COL_MESH_PATH "PARKST_COL_MESH_PATH-NOTFOUND" CACHE FILEPATH "Location on disk of the Park St. collision mesh" FORCE)
	set(PARKST_VIS_MESH_PATH "PARKST_VIS_MESH_PATH-NOTFOUND" CACHE FILEPATH "Location on disk of the Park St. visualization mesh" FORCE)
	return()
else()
	if(${CMAKE_VERSION} VERSION_LESS "3.12.4")
		add_definitions(-DPARKST_COL_MESH_PATH=${PARKST_COL_MESH_PATH})
		add_definitions(-DPARKST_VIS_MESH_PATH=${PARKST_VIS_MESH_PATH})
	else()
		add_compile_definitions(PARKST_COL_MESH_PATH=${PARKST_COL_MESH_PATH})
		add_compile_definitions(PARKST_VIS_MESH_PATH=${PARKST_VIS_MESH_PATH})
	endif()
endif()

set(DEMOS
    demo_SYN_parkst
    demo_SYN_parkst_JSON
)
message(${DEMOS})

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	# _interactive includes a joystick file that only works on Linux
	list(APPEND DEMOS
		demo_SYN_parkst_interactive)
endif()

#--------------------------------------------------------------

include_directories(${SYN_INCLUDES})

foreach(PROGRAM ${DEMOS})
	message(STATUS "SynChrono demos -- ${PROGRAM}")

	add_executable(${PROGRAM} ${PROGRAM}.cpp)

	set_target_properties(${PROGRAM} PROPERTIES
		    COMPILE_FLAGS "${SYN_CXX_FLAGS}"
			LINK_FLAGS "${CH_LINKERFLAG_EXE} ${MPI_LINK_FLAGS}")

	target_link_libraries(${PROGRAM}
							ChronoEngine_synchrono
							${SYN_LIBRARIES})
endforeach()
