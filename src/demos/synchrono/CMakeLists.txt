message(STATUS "Add demo programs for SYNCHRONO module")

add_subdirectory(mpi)

if(FASTDDS_FOUND)
  add_subdirectory(dds)
endif()
