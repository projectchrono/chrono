set(DEMOS
    demo_CH_buildsystem
)

message(STATUS "---------------------")
message(STATUS "C# basic Chrono demos")
message(STATUS "---------------------")

# Look for a Chrono installation
find_package(Chrono
             COMPONENTS CSharp
             CONFIG)

# Append to the parent's lists of DLLs (and make them visible)
list(APPEND ALL_DLL_NAMES       "${CHRONO_DLL_NAMES}")
list(APPEND ALL_DEPENDENCY_DLLS "${CHRONO_DEPENDENCY_DLLS}")
set(ALL_DLL_NAMES       "${ALL_DLL_NAMES}"       PARENT_SCOPE)
set(ALL_DEPENDENCY_DLLS "${ALL_DEPENDENCY_DLLS}" PARENT_SCOPE)

# Add C# executables
foreach(PROGRAM ${DEMOS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} "${PROGRAM}.cs" ${CHRONO_CSHARP_SOURCES})
endforeach()
