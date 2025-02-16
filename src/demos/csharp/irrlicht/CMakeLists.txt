set(DEMOS
    demo_IRR_assets
)

message(STATUS "-------------------------")
message(STATUS "C# Chrono::Irrlicht demos")
message(STATUS "-------------------------")

# Look for a Chrono installation
find_package(Chrono
             COMPONENTS CSharp
             OPTIONAL_COMPONENTS Irrlicht
             CONFIG)

# Append to the parent's lists of DLLs (and make them visible above)
list(APPEND ALL_DLL_NAMES       "${CHRONO_DLL_NAMES}")
list(APPEND ALL_DEPENDENCY_DLLS "${CHRONO_DEPENDENCY_DLLS}")
set(ALL_DLL_NAMES       "${ALL_DLL_NAMES}"       PARENT_SCOPE)
set(ALL_DEPENDENCY_DLLS "${ALL_DEPENDENCY_DLLS}" PARENT_SCOPE)

# Make CHRONO_PACKAGE_PREFIX_DIR visible above (needed for add_DLL_copy_command)
set(CHRONO_PACKAGE_PREFIX_DIR ${CHRONO_PACKAGE_PREFIX_DIR} PARENT_SCOPE)

# Add C# executables
message(STATUS "Demos:")
foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
endforeach()
