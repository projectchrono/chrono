# SPDX-License-Identifier: MIT
# This snippet enables Chrono::CSharp (SWIG-generated C# bindings).
#
# Note on C# bindings:
# The Chrono::CSharp installation guide requires SWIG 4.0 or newer and enables
# the module with CH_ENABLE_MODULE_CSHARP. SWIG is installed by the shared
# chrono.dockerfile dependency layer, so this snippet only needs to enable the
# module.
#
# This mirrors the current Chrono CMake code path:
#
#   - contrib/docker/snippets/chrono.dockerfile:
#       * installs swig with the common Chrono build dependencies
#   - src/chrono_swig/chrono_csharp/CMakeLists.txt:
#       * enables the module with CH_ENABLE_MODULE_CSHARP
#       * calls find_package(SWIG REQUIRED COMPONENTS csharp)
#       * generates native Chrono_csharp_* libraries and SWIG-generated .cs
#         sources for enabled Chrono components
#       * installs the C# template project
#   - cmake/ChronoConfig.cmake.in:
#       * exposes Chrono_CSHARP_AVAILABLE
#       * collects CHRONO_CSHARP_SOURCES for requested Chrono C# components
#
# Do not enable CH_USE_CSHARP_WRAPPER in this image. That optional target
# requires the dotnet CLI to publish a single ready-to-run
# chrono_csharp_wrapper.dll, which would pull the .NET SDK into the image. The
# default SWIG-generated native libraries and .cs sources are sufficient for a
# general Chrono development image and match the documented C# module workflow.

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_CSHARP=ON"
