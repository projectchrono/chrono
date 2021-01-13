# Test programs for RoboSimian simulation
# ------------------------------------------------------------------------------

message(STATUS "Demo programs for ROBOSIMIAN model...")

if(NOT ENABLE_MODULE_VEHICLE)
  message("  Warning: the Robosimian projects require Chrono::Vehicle!")
  return()
endif()

add_subdirectory(sequential)
add_subdirectory(multicore)
