# Test programs for RoboSimian simulation
# ------------------------------------------------------------------------------

if(NOT ENABLE_MODULE_VEHICLE)
  message(STATUS "  Warning: the Robosimian projects require Chrono::Vehicle!")
  return()
endif()

add_subdirectory(sequential)
add_subdirectory(multicore)
