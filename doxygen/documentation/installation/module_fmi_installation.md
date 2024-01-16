Install the FMI module {#module_fmi_installation}
==========================

[TOC]

This optional module provides support for creation (export) and use (import) of FMUs encapsulating Chrono models and/or simulations. 




Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.

## Features

The Functional Mock-up Interface ([FMI](https://fmi-standard.org/)) is a free standard that defines a container (FMU - Functional Mock-up Unit) and an interface to exchange dynamic simulation models using a combination of XML files, binaries and C code, distributed as a ZIP file.

The `Chrono::FMI` module currently supports the FMI 2.0 standard, with plans to extend support for FMI 3.0 in the future.  Because of limitations of the FMI standard (in terms of defining models described by DAEs), only FMUs for co-simulation can be exported from Chrono.

Unlike other Chrono modules, `Chrono::FMI` does not produce a new Chrono library, but rather exposes support for exporting Chrono co-simulation FMUs and importing FMUs for use in co-simulation with other Chrono models.

If the `Chrono::Vehicle` module is also enabled, several FMUs encapsulating vehicle-related models are generated and made available for use. New Chrono::Vehicle FMUs will be added in the future.

## Requirements

There are no explicit dependencies to build Chrono FMI support. The `Chrono::FMI` module makes use of a generic FMU export/import tool ([fmu_tools](https://github.com/DigitalDynamicsLab/fmu_tools.git)), also developed and maintained by the ProjectChrono team, but that library is used as a git sub-module and as such need not be separately downloaded and installed.

<div class="ce-warning">
The self-encapsulation requirements of an FMU are easiest satisfied by linking static libraries. Because of this, the `Chrono::FMI` module can be enabled **only** if Chrono is configured to create static libraries. Furthermore, when building Chrono with FMI support on Windows, make sure to compile Chrono with a multi-threaded statically-linked runtime library (`/MT` or '/MTd', for Release and Debug modes, respectively). Both of these conditions can be set during CMake configuration, see the building instructions below.
<br><br>
The requirement of using Chrono static libraries means that not all Chrono modules can be used in a build with `Chrono::FMI` enabled. While this issue will be further investigated, it is currently recommended not to enable any of the GPU-based Chrono modules.
<br><br>
The `Chrono::FMI` module has been tested (on both Windowsc and Linux) in conjunction with the following other Chrono optional modules: Chrono::Vehicle, Chrono::Irrlicht, Chrono::Postprocess, and Chrono::PardisoMKL. It is possible that other modules can also be built, but that will require some experimentation.
<br><br>
The `Chrono::FMI` module has **not** been tested on MacOS.
</div>   

## Building instructions

1. During CMake configuration, ensure that `BUILD_SHARED_LIBRARIES` is set to `OFF`. This will force building Chrono static libraries.

2. On Windows, also ensure that `USE_MSVC_STATIC_RUNTIME` is set to `ON`. This will force using a multi-threaded statically-linked runtime library.

3. Set `ENABLE_MODULE_FMI` to `ON`.

4. Set other CMake variables as desired/needed and press/invoke `Configure` until all CMake dependencies are satisfied. Then press/invoke `Generate` to create the build scripts.

5. Proceed with the building of Chrono as usual.

## How to use it

- Consult the [documentation](https://github.com/DigitalDynamicsLab/fmu_tools/blob/main/README.md) of `fmu_tools` for the generic functionality for exporting and importing FMUs.

- Look at the documentation of the Chrono extensions for exporting (`chrono::FmuChronoComponentBase`) and importing (`chrono::FmuChronoUnit`) FMUs that encapsulate Chrono models.

- Look at the C++ source of [Chrono::FMI demos](@ref tutorial_table_of_content_chrono_fmi) to learn how to use the functionality of this module.

