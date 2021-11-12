# ------------------------------------------------------------------------------
# List of demos according their dependencies
# ------------------------------------------------------------------------------

# Requires Irrlicht run-time visualization.
if(NOT FAST_DDS_FOUND)
  return()
endif()

set(DEMOS 
    demo_SYN_DDS_distributed
)

# --------------------------------------------------------------

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

include_directories(${SYN_INCLUDES})

foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  source_group("" FILES ${PROGRAM}.cpp)

  set_target_properties(${PROGRAM} PROPERTIES COMPILE_FLAGS "${SYN_CXX_FLAGS}")

  target_link_libraries(${PROGRAM} ChronoEngine_synchrono)
endforeach()
