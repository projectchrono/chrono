message(STATUS "Add demo programs for VEHICLE module")

add_subdirectory(wheeled_models)
add_subdirectory(wheeled_tests)
add_subdirectory(articulated_vehicle)
add_subdirectory(two_cars)

add_subdirectory(tracked_models)

add_subdirectory(test_rigs)

add_subdirectory(terrain)
add_subdirectory(controllers)
add_subdirectory(multicore)
add_subdirectory(paths)
add_subdirectory(visualization)

add_subdirectory(fmi)
add_subdirectory(fsi)

add_subdirectory(cosimulation)
