Chrono::FSI Reference Manual {#manual_fsi}
==========================================

The [Chrono::FSI](group__fsi.html) module provides a generic interface between a Chrono solid-phase multibody system and an arbitrary fluid solver for Fluid-Solid Interaction (FSI) problems
Currently, Chrono::FSI provides coupling to:
- a fluid solver based on the Smoothed Particle Hydrodynamics (SPH) method in the the [Chrono::FSI-SPH](group__fsisph.html) sub-module;
- a time-dependent potential flow (TDPF) solver, in the [Chrono::FSI-TDPF](group__fsitdpf.html) sub-module which interfaces to the 3rd-party [HydroChrono](https://github.com/Project-SEA-Stack/HydroChrono) solver.

## Generic Chrono::FSI module

The structure of the generic Chrono::FSI framework can be represented schematically as:

<img src="http://www.projectchrono.org/assets/manual/Chrono_FSI.png" width="800">

The Chrono::FSI module is centered around an abstract FSI system ([ChFsiSystem](@ref chrono::fsi::ChFsiSystem)) which holds references to an (externally-provided) Chrono multibody system, an arbitrary fluid solver system, and an abstract coupling interface which intermediates the data exchange between phases. 

The generic [ChFsiInterface](@ref chrono::fsi::ChFsiInterface) defines the solid phase through its collision geometry (thus consistent with solid-solid contact representation in Chrono) and implements data exchange using fluid solver-agnostic data structures. In particular, rigid bodies are described as a collection of collision shapes (primitives or triangular meshes) with associated transforms relative to the body reference frame, while deformable solids are described by associated collision surfaces (which can be a segment set for beam-like elements or surface meshes for shell and solid finite elements). Among other things, this allows use of arbitrary finite elements.

The FSI problem is solved in an explicit force-displacement co-simulation setting, with the Chrono system communicating full state information of the solid phase (position and velocity level) and the fluid solver sending fluid forces and torques acting on rigid bodies or on nodes of the FEA meshes. ChFsiSystem controls the co-simulation time stepping function, first invoking the data exchange functions through ChFsiInterface then advancing the dynamics of the two phases simultaneously and in parallel (using non-blocking concurrent threads).

## SPH-based FSI module

The [Chrono::FSI-SPH](group__fsisph.html) module provides support for SPH-based FSI, including CFD (incompressible Navier-Stokes) and CRM (continuum granular) problems.

As an instantiation of the generic Chrono::FSI framework, the structure of the SPH-based Chrono::FSI-SPH module can be represented schematically as:

<img src="http://www.projectchrono.org/assets/manual/Chrono_FSI-SPH.png" width="800">

<br>

Further details on the FSI-SPH module are provided in the following pages:
* @subpage manual_fsi_sph_class_guide
* @subpage manual_fsi_rigid_bce_markers
* @subpage manual_fsi_sph_parameter_selection
* @subpage manual_fsi_mu_i_rheology
* @subpage manual_fsi_mcc_rheology

## TDPF-based FSI module

The [Chrono::FSI-TDPF](group__fsitdpf.html) module provides support for FSI problems solved with a time-domain potential flow approach. The TDPF support is provided by the fluid solver in the external [HydroChrono](https://github.com/Project-SEA-Stack/HydroChrono) project, included in Chrono as a git submodule:

The resulting simulation framework is represented schematically in the image below which also illustrates the YAML parsers associated with such FSI problems.

<img src="http://www.projectchrono.org/assets/manual/Chrono_FSI-TDPF.png" width="800">
