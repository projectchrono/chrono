# Install the preCICE module {#module_precice_installation}

[TOC]

Chrono::preCICE is an optional module that lets a Chrono solver participate in a partitioned multi-physics
simulation driven by the [preCICE](https://precice.org/) coupling library.

<div class="ce-info">
This page is a draft. The Chrono build steps are accurate for the current CMake configuration. For building
preCICE itself, only the source to start from is given below; for its dependencies and build options,
consult the [preCICE installation documentation](https://precice.org/installation-overview.html).
</div>

## Features

The **preCICE module** provides adapters that expose Chrono solvers as preCICE *participants*, so that each
solver runs in its own process and preCICE handles process discovery, data mapping between non-matching
meshes, and the coupled time stepping. Adapters are provided for a Chrono multibody system, for the
Chrono::FSI-SPH fluid solver, and for the Chrono::FSI-TDPF fluid solver. The partner participant need not be
a Chrono solver.

For more detail, read the [Chrono::preCICE](@ref manual_precice) section of the reference manual.

## Requirements

- [preCICE](https://precice.org/) version 3.0 or newer, built and installed so that CMake can find its
  package configuration file.

Chrono::preCICE has been tested with **preCICE 3.4.0**. Build preCICE from source; the repository to start
from differs by platform.

### Linux

Obtain preCICE from its [GitHub repository](https://github.com/precice/precice) and build the 3.4.0 release:

~~~{.sh}
git clone https://github.com/precice/precice.git
cd precice
git checkout v3.4.0
~~~

Then configure, build, and install it following the
[preCICE build documentation](https://precice.org/installation-source-preparation.html).

### Windows

The upstream sources do not build with MSVC without modification. Use instead the fork at
<https://github.com/rserban/precice.git>, branch `fix-3.4.0`, which carries small fixes that allow preCICE
3.4.0 to be built on Windows with MSVC:

~~~{.sh}
git clone --branch fix-3.4.0 https://github.com/rserban/precice.git
~~~

Configure, build, and install it as described in the preCICE documentation, using the dependencies
appropriate for a Windows build.

### MPI

preCICE may be built with or without MPI support. A preCICE built with MPI links the Chrono participants
against an MPI implementation, which affects how they are launched; see
[Running a coupled simulation](#running) below.

## Optional dependencies

The module builds against whichever Chrono modules are enabled, and several of its capabilities are
conditional:

| Chrono module | What it adds |
|---------------|--------------|
| [Chrono::Parsers](@ref module_parsers_installation), with YAML support | Construction of an adapter from a Chrono YAML specification file. Without it, only the constructors that take an existing Chrono solver object are available, and the demos are not built. |
| [Chrono::VSG](@ref module_vsg_installation) | Run-time visualization for a participant. |
| [Chrono::FSI-SPH](@ref module_fsi_installation) | Builds `ChPreciceAdapterSph`. Without it, that adapter is not compiled at all. |
| [Chrono::FSI-TDPF](@ref module_fsi_installation) | Builds `ChPreciceAdapterTdpf`. Without it, that adapter is not compiled at all. |

In addition, reading added mass coefficients from an HDF5 hydrodynamic file requires a Chrono built with
HDF5 support. Without it, that path reports an error at run time; added mass given explicitly in a
participant specification file is unaffected.

## Building instructions

1. Install preCICE 3.0 or newer, as described above.
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
3. Set `CH_ENABLE_MODULE_PRECICE` to 'on' in the CMake configuration.
4. If CMake cannot locate preCICE, set `precice_DIR` to the directory containing the installed
   `preciceConfig.cmake`, then configure again.
5. Enable the optional Chrono modules listed above for the capabilities you need. In particular, enable
   Chrono::Parsers to build the demos, and Chrono::FSI-SPH or Chrono::FSI-TDPF for the corresponding fluid
   adapters.
6. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-warning">
If preCICE is not found, the module does not stop the configuration: it prints
`Error: preCICE 3.0 not found. Set precice_DIR.` and turns `CH_ENABLE_MODULE_PRECICE` back off. The rest of
Chrono then configures normally, so check that the option is still on after configuring, rather than
assuming the module was built.
</div>

## Running a coupled simulation {#running}

A coupled simulation is not a single program. Each participant is launched separately, with its own
participant name and a copy of the shared preCICE configuration file, and the processes rendezvous through
the mechanism declared in that configuration. The shipped examples use `<m2n:sockets>` with an exchange
directory, which means all participants must be started from the same working directory:

~~~{.sh}
demo_PRECICE_sphere_drop -p Solid      -c ../data/precice/sphere_drop/precice_config_AM_explicit.xml
demo_PRECICE_sphere_drop -p Fluid_TDPF -c ../data/precice/sphere_drop/precice_config_AM_explicit.xml
~~~

The order does not matter; the first process to start waits for the other.

If preCICE was built with MPI support, the participants are MPI programs even when each one runs on a single
rank, and they may need to be launched through the MPI launcher rather than directly:

~~~{.sh}
mpiexec -n 1 demo_PRECICE_sphere_drop -p Solid -c <config>
~~~

Whether a direct launch works depends on the MPI implementation and how its runtime is set up. If a
participant aborts during MPI initialization when started directly, launch it through `mpiexec` instead.
On Windows with Intel MPI, this means running from a shell in which the Intel oneAPI environment has been
initialized (`setvars.bat`), and a single-node run may additionally require restricting the fabric to shared
memory by setting `I_MPI_FABRICS=shm`.

<div class="ce-info">
If one participant fails during startup, the others wait indefinitely rather than exiting. Before retrying a
failed run, make sure no participant processes are still alive and delete the `precice-run` directory left
in the working directory; a stale rendezvous directory or a surviving process from a previous attempt can
cause the next run to hang during the connection handshake.
</div>

## How to use it

- Consult the [reference manual](@ref manual_precice).

- Look at the [API section](@ref precice_module) of this module for documentation about classes and
  functions.

- Look at the source of the demos in `src/demos/precice`, together with the corresponding configuration
  files under `data/precice`, to learn how a coupled problem is set up.
