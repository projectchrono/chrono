
# Set common output directory for all demo FMUs
set(DEMO_FMU_MAIN_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/demoFMU")

# MESSAGE(STATUS "Demo programs for FMI module (FMI 2.0)...")
add_subdirectory(fmi2_cosim)

# MESSAGE(STATUS "Demo programs for FMI module (FMI 3.0)...")
add_subdirectory(fmi3_cosim)

# MESSAGE(STATUS "Demo programs for Chrono encapsulation of model exchange FMUs")
add_subdirectory(fmi_modex)
