#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for SynChrono module...")

add_subdirectory(demo_vehicles)
add_subdirectory(demo_platoon)
add_subdirectory(demo_SCM)
add_subdirectory(demo_highway)

#--------------------------------------------------------------
