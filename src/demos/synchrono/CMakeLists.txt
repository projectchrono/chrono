#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

message(STATUS "Add demo programs for SYNCHRONO module")

add_subdirectory(mpi)

if(FASTDDS_FOUND)
  add_subdirectory(dds)
endif()
