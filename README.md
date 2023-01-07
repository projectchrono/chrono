ATTENTION
=========

The structure of the Chrono git repository was changed as follows:

- The main development branch is now called `main` (previously `develop`)
- The `master` branch, now obsolete, was deleted
- Releases are located in branches named `release/*.*` and have tags of the form `*.*.*`


Project CHRONO
==============

[![pipeline status](https://gitlab.com/uwsbel/chrono/badges/main/pipeline.svg)](https://gitlab.com/uwsbel/chrono/commits/main)
[![BSD License](http://www.projectchrono.org/assets/logos/chrono-bsd.svg)](https://projectchrono.org/license-chrono.txt)


Distributed under a permissive BSD license, Chrono is an open-source multi-physics package used to model and simulate:
-	dynamics of large systems of connected rigid bodies governed by differential-algebraic equations (DAE)
-	dynamics of deformable bodies governed by partial differential equations (PDE)
-	granular dynamics using either a non-smooth contact formulation resulting in differential  variational inequality (DVI) problems or a smooth contact formulation resulting in DAEs
-	fluid-solid interaction problems whose dynamics is governed by coupled DAEs and PDEs
-	first-order dynamic systems governed by ordinary differential equations (ODE)

Chrono provides a mature and stable code base that continues to be augmented with new features and modules.  The core functionality of Chrono provides support for the modeling, simulation, and visualization of rigid and flexible multibody systems with additional capabilities offered through optional modules. These modules provide support for additional classes of problems (e.g., granular dynamics and fluid-solid interaction), modeling and simulation of specialized systems (such as ground vehicles), co-simulation, run-time visualization, post-processing, interfaces to external linear solvers, or specialized parallel computing algorithms (multi-core, GPU, and distributed) for large-scale simulations.

Used in many different scientific and engineering problems by researchers from academia, industry, and government, Chrono has mature and sophisticated support for multibody dynamics, finite element analysis, granular dynamics, fluid-solid interaction, ground vehicle simulation and vehicle-terrain interaction.  

Implemented almost entirely in C++, Chrono also provides Python and C# APIs. The build system is based on CMake. Chrono is platform-independent and is actively tested on Linux, Windows, and MacOS using a variety of compilers.

- [Project website](http://projectchrono.org/)
- [Build and install instructions](https://api.projectchrono.org/development/tutorial_table_of_content_install.html)


### Documentation

- C++ API reference
  - [Main development branch](http://api.projectchrono.org/)
  - [Release 8.0.0](http://api.projectchrono.org/8.0.0/)
  - [Release 7.0.0](http://api.projectchrono.org/7.0.0/)
- Python interface
  - [PyChrono](https://api.projectchrono.org/pychrono_introduction.html)
- Reference manuals
  - [Core module](https://api.projectchrono.org/manual_root.html)
  - [Chrono::Vehicle module](https://api.projectchrono.org/manual_vehicle.html)

### Support

- [Google Groups user forum](https://groups.google.com/g/projectchrono)
