#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for SYNCHRONO module...")

option(BUILD_PARKST "Build the Park St. demo (requires an SBEL only mesh)" OFF)
mark_as_advanced(BUILD_PARKST)

add_subdirectory(demo_vehicles)
add_subdirectory(demo_platoon)
add_subdirectory(demo_SCM)
add_subdirectory(demo_highway)
add_subdirectory(demo_mpi)
if (BUILD_PARKST)
    add_subdirectory(demo_parkst)
endif()

if (USE_FAST_DDS)
  add_subdirectory(dds/demo_dds)
endif()

#--------------------------------------------------------------
