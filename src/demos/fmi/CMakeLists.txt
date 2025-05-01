message(STATUS "Add demo programs for FMI module")

# Set common output directory for all demo FMUs
set(DEMO_FMU_MAIN_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/demoFMU")

add_subdirectory(cosim)
add_subdirectory(modex)
