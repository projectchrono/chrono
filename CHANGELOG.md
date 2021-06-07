<!-- For syntax, see  https://github.github.com/gfm/ -->


Change Log
==========

- [Unreleased (development version)](#unreleased-development-branch)
  - [Miscellaneous additions to Chrono::Gpu](#added-miscellaneous-additions-to-chronogpu)
  - [New loads for ChNodeFEAxyzrot](#added-new-loads-for-chnodefeaxyzrot)
  - [Analytical box-box collision detection algorithm in Chrono::Multicore](#added-analytical-box-box-collision-detection-algorithm-in-chronomulticore)
  - [Checkpointing capabilities in Chrono::Gpu](#added-checkpointing-capabilities-in-chronogpu)
  - [Fixes to particle volume samplers and generators](#fixed-fixes-to-particle-volume-samplers-and-generators)
  - [SCM deformable terrain improvements](#changed-scm-deformable-terrain-improvements)
  - [Miscellaneous fixes to Chrono::Vehicle API](#changed-miscellaneous-fixes-to-chronovehicle-api)
  - [New tracked vehicle model](#added-new-tracked-vehicle-model)
  - [Support for Z up camera in Chrono::Irrlicht](#changed-support-for-z-up-camera-in-chronoirrlicht)
  - [Reading and writing collision meshes in Chrono::Gpu](#changed-reading-and-writing-collision-meshes-in-chronogpu)
- [Release 6.0.0](#release-600---2021-02-10) 
  - [New Chrono::Csharp module](#added-new-chronocsharp-module)
  - [RoboSimian, Viper, and LittleHexy models](#added-robosimian-viper-and-littlehexy-models)
  - [Contact force reporting through user-provided callback](#added-contact-force-reporting-through-user-provided-callback)
  - [Chrono::Gpu module rename](#changed-chronogpu-module-rename)
  - [Chrono::Multicore module rename](#changed-chronomulticore-module-rename)
  - [Geometric stiffness for Euler beams](#added-geometric-stiffness-for-euler-beams)
  - [New Chrono::Synchrono module](#added-new-chronosynchrono-module)
  - [Rename Intel MKL Pardiso interface module](#changed-rename-intel-mkl-pardiso-interface-module)
  - [Saving POV-Ray files from Irrlicht interactive view](#added-saving-pov-ray-files-from-irrlicht-interactive-view)
  - [Support for modelling wheeled trailers](#added-support-for-modelling-wheeled-trailers)
  - [Enhancements to Chrono::FSI](#changed-enhancements-to-chronofsi)
  - [New Chrono::Sensor module](#added-new-chronosensor-module)
  - [Setting OpenMP number of threads](#changed-setting-openmp-number-of-threads)
  - [Redesigned SCM deformable terrain](#changed-redesigned-scm-deformable-terrain)
  - [Tracked vehicle support in PyChrono](#added-tracked-vehicle-support-in-pychrono)
  - [Constitutive models for Euler beams](#changed-constitutive-models-for-euler-beams)
  - [Constitutive models for IGA beams](#changed-constitutive-models-for-iga-beams)
  - [Obtaining body applied forces](#added-obtaining-body-applied-forces)
  - [Chrono::Vehicle simulation world frame](#added-chronovehicle-simulation-world-frame)
  - [CASCADE module](#changed-cascade-module)
  - [Collision shapes and contact materials](#changed-collision-shapes-and-contact-materials)
- [Release 5.0.1](#release-501---2020-02-29)
- [Release 5.0.0](#release-500---2020-02-24)
  - [Eigen dense linear algebra](#changed-refactoring-of-dense-linear-algebra)
  - [Eigen sparse matrices](#changed-eigen-sparse-matrices-and-updates-to-direct-sparse-linear-solvers)
- [Release 4.0.0](#release-400---2019-02-22)

## Unreleased (development branch)

## [Added] Miscellaneous additions to Chrono::Gpu

The location of the computational domain can now be specified (in addition to its dimensions) through a fourth optional constructor argument of `ChSystemGpu` and `ChSystemGpuMesh`. By default, the axis-aligned computational domain is centered at the origin.  As such,
```cpp
ChSystemGpu gpu_sys(1, 1, ChVector<float>(100, 80, 60));
```
sets the computational domain to be [-50,50] x [-40,40] x [-30,30], while
```cpp
ChSystemGpu gpu_sys(1, 1, ChVector<float>(100, 80, 60), ChVector<float>(10, 20, 30));
```
sets the computational domain to be [-40,60] x [-20,60] x [0,60].
Note also that, for consistency of the API, the type of the domain size (third constructor argument) was changed to `const ChVector<float>&`.

A new function, `ChSystemGpu::GetParticlesKineticEnergy` was added to calculate and return the total kinetic energy of the granular particles.

### [Added] New loads for ChNodeFEAxyzrot

New classes have been added for creating loads (with automatic jacobian generation that allow also stiff loads) for ChNodeFEAxyzrot nodes, in detail:
- on a node of ChNodeFEAxyzrot type (user defined etc.)
- between two ChNodeFEAxyzrot (user defined, spherical bushing, plastic bushing, generic bushing, etc.)
- between a ChNodeFEAxyzrot and a ChBody (user defined, spherical bushing, plastic bushing, generic bushing, etc.)
Previously, these types of loads were available only for the ChNodeFEAxyz node (used in tetahedrons and bricks, for example) but not for ChNodeFEAxyzrot (used in beams and Reissner shells, for example). 

### [Added] Analytical box box collision detection algorithm in Chrono::Multicore

A new algorithm for analytical collision detection for box-box interactions was added to the parallel collision system implemented in Chrono:Multicore.
For collisions involving two boxes, this new algorithm is now used instead of the default MPR algorithm (when using narrow phase type `NARROWPHASE_R` or `NARROWPHASE_HYBRID_MPR`).

The new algorithm relies on the 15-axes test of Gottschalk, Lin, and Manocha (Siggraph 1996) for finding the direction of minimum intersection between two oriented boxes and then the collision detection is special-cased for all possible combinations of interacting features from the two boxes (9 different cases).
The analytical algorithm can produce up to 8 collision pairs and works with or without a collision envelope (thus being appropriate for both SMC and NSC contact forumlations).

### [Added] Checkpointing capabilities in Chrono::Gpu

Chrono::Gpu can now output a checkpoint file to store the current simulation state, then re-start the simulation from that stage. The checkpointed information includes the simulation parameters such as sphere radius and density, the positions and velocities of particles, and the friction history if using a frictional model.

To use checkpointing, at any point after `ChSystemGpu::Initialize()` , call `ChSystemGpu::WriteCheckpointFile(filename)` to generate a checkpoint file named `filename`. Then, a new simulation can be generated from this file, by either:
- constructing a new `ChSystemGpu` system from a checkpoint file; or
- calling `ChSystemGpu::ReadCheckpointFile(filename)` to load the checkpointed state to a existing system (before calling `ChSystemGpu::Initialize()`). This will check if the loaded simulation parameters conflict with the existing system, and throw an error if so; or
- calling `ChSystemGpu::ReadCheckpointFile(filename, true)`, which is similar to above, but overwrites existing simulation parameters with those from the checkpoint file.

A simple example:
```cpp
ChSystemGpu sys1(radius, density, make_float3(box_X, box_Y, box_Z));
/* Set simulation parameters */
sys1.Initialize();
sys1.AdvanceSimulation(1.0);
sys1.WriteCheckpointFile("checkpoint.dat");

ChSystemGpu sys2("checkpoint.dat");
/* Or load checkpoint manually...
ChSystemGpu sys2(radius, density, make_float3(box_X, box_Y, box_Z));
Set simulation parameters...
sys2.ReadCheckpointFile("checkpoint.dat");  
*/
```

`ChSystemGpu::ReadParticleFile` is used to load particle positions and velocities from a CSV file. It is useful if the particle information is meant to be supplied from a file rather than programatically.

See demo_GPU_ballcosim for an example of using checkpointing.

Function renames:
- `ChSystemGpu::WriteFile` renamed to `ChSystemGpu::WriteParticleFile`
- `ChSystemGpu::SetOutputFlags` renamed to `ChSystemGpu::SetParticleOutputFlags`
- `ChSystemGpu::SetOutputMode` renamed to `ChSystemGpu::SetParticleOutputMode`

Notes:
- Default output flags are set to write particle positions and velocity magnitudes only, excluding angular velocity components. The output flags can be set by `ChSystemGpu::SetParticleOutputFlags`.
- If the simulation loads a checkpoint file or a CSV particle file, it will not do the defragment process during `Initialize()`; otherwise it will. The defragment process tries to re-order the particle numbering such that the particles belong to a SD become close together in system arrays. It is by default disabled for re-started simulations to not change the numbering from the previous simulation. The user can manually enforce the defragment process by calling `ChSystemGpu::SetDefragmentOnInitialize(true)`.

Known issues:
- Support for `CHGPU_TIME_INTEGRATOR::CHUNG` is partial. The checkpoint file does not store velocities of the previous time step, so if a checkpoint is loaded while `CHGPU_TIME_INTEGRATOR::CHUNG` is in use, the physics will change. It is therefore best to avoid `CHGPU_TIME_INTEGRATOR::CHUNG` if checkpointing is needed. No demo uses `CHGPU_TIME_INTEGRATOR::CHUNG`.
- The checkpoint file does not store any manually defined boundaries (those defined by `ChSystemGpu::CreateBC*`) or meshes (those defined by `ChSystemGpuMesh::AddMesh`). For now, these need to be manually added before initializing the re-started simulation.


### [Fixed] Fixes to particle volume samplers and generators

- An incorrect implementation of the HCP (Hexagonally Close Packed) sampler, `utils::HCPSampler`, resulting in the wrong lattice was fixed. 

- The API of the various particle generator functions `utils::Generator::CreateObjects***` was changed to take as first argument a reference to a volume sampler.  Previous code such as:
```cpp
    utils::Generator gen(system);
    gen.createObjectsBox(utils::SamplingType::POISSON_DISK, sep, center, hdims);
```
should be changed to:
```cpp
   utils::PDSampler<double> sampler(sep);
   utils::Generator gen(system);
   gen.CreateObjectsBox(sampler, center, hdims);
```
  This change was necessary to obtain proper randomization (where applicable) when generating particles in successive layers; indeed, the previous implementation created a new sampler at each function invocation resulting in layers with the same distribution of particle positions.  

### [Changed] SCM deformable terrain improvements

The reference frame for calculation of normal and tangential forces has been changed to be aligned with the local normal (as opposed to always being aligned with the SCM frame).  This fixes the generated terrain forces for SCM patches defined from height maps.  The Bekker forces are aligned with the local terrain normal, while the tangential shear forces (Janosi-Hanamoto) lie in the local tangent plane.  Note that the normal at each grid node is based on the undeformed terrain.

In addition, the SCM implementation was changed to extend the logical grid beyond the (horizontal) limits specified by the user during initialization; this continuation is done by extending to infinity the logical patch using the terrain height and normal from the closest grid node within the specified domain.

Finally, support was added for inclusion of tire-soil parameters (in addition to soil-soil parameters), if these are available. To provide this information, attach a structure of type `SCMContactableData` as user-data to the desired contactable object (e.g. a tire or track shoe body):
```cpp
    auto tire_data = chrono_types::make_shared<chrono::vehicle::SCMContactableData>(Aratio, Mcoh, Mfric, Jshear);
    tire_body->SetUserData(tire_data);
```
The necessary data includes the SCM tangential force parameters, Mohr cohesion (Pa), friction angle (degrees), and the Janosi shear parameter (m), as well as a ratio that represents the weight of the tire-soil parameters in calculating the tangential force (using linear interpolation). A ratio value of 0 indicates using only the soil-soil parameters, while a value of 1 indicates using only the tire-soil parameters.  Typically, this ratio is set as the area ratio of tread surface over tire surface.  

### [Changed] Miscellaneous fixes to Chrono::Vehicle API

- Changed enum class names for suspension, driveline, and steering types to properly differentiate between wheeled and tracked vehicles.
   The new enum classes, defined in `ChSubsysDefs.h` are SuspensionTypeWV, DrivelineTypeWV, and SteeringTypeWV for wheeled vehicles and DrivelineTypeTV for tracked vehicles.

- Eliminated the setting for differential ratio in the various driveline templates.
   To model a differential using the Chrono class `ChShaftsPlanetary`, this value must always be -1 (and represents the speed ratio of the inverted planetary) and is therefore hard-coded.
   This affects driveline models that use the Chrono 1-D shaft modeling elements and the schema of associated JSON specifation files.

- Modified all shafts-based driveline templates to expect a positive value for the conical gear ratios.

- Added option (`ChPowertrain::SetTransmissionMode`) for setting the transmission mode of a powertrain to either `AUTOMATIC` or `MANUAL` (the latter modeling a manumatic-type transmission).  
   If in `MANUAL` mode, gear shifting can be controlled with `ChPowertrain::ShiftUp`  and `ChPowertrain::ShiftDown`.

- Modified the "Simple CVT" powertrain template.
   In the new template specification, a parameter for max engine speed was added and the parameter for critical engine speed was removed.

- Added utility function to programatically generate a sprocket visualization mesh (`ChSprocket::CreateVisualizationMesh`).
   All Chrono::Vehicle sprocket profiles are defined as a succession of line segments and circle arcs. The default visualization is of type `VisualizationType::PRIMITIVES` and is a 3-D line for the profile.  The utility function `ChSprocket::CreateVisualizationMesh` creates a trimesh that can be used to visualize the sprocket when in `VisualizationType::MESH` mode.

- Changed sign of sprocket angular speed reported by GetSprocketSpeed so that a positive value corresponds to forward vehicle movement.
   This change was made simply for convenience and consistency.

- Completed the braked differential steering driveline for tracked vehicles (`ChTrackDrivelineBDS`) to properly implement steering.
    In this driveline model, steering is achieved through braking; this is implemented through a driveline-specific utility function that combines the steering and braking controls.

- Added function `RandomSurfaceTerrain::EnableCollisionMesh` to optionally generate a terrain collision mesh.
    This is necessary for tracked vehicles or wheeled vehicles with rigid tires (which rely on the underlying Chrono contact system).

### [Added] New tracked vehicle model

The Marder ("marten" in German) is a tracked infantry fighting vehicle used by the German Bundeswehr since 1969. It has a running gear with 12 road wheels, sprocket, idler and 3 support rollers. The first two and the last two road wheels on every side are damped by telescopic dampers. It is driven by a 444 kW Diesel engine, torque converter with lockup and 4 gear automatic gearbox. It carries up to nine soldiers (commander, gunner, driver and six infantrymen).

The Chrono::Vehicle model is based only on public data available online and information found in literature. Although the original vehicle emplys double-pin tracks, the current Chrono model only implements a single-pin track.

### [Changed] Support for Z up camera in Chrono::Irrlicht

While the default remains to construct a camera with Y up, the ChIrrApp class was modified to also support a camera with Z up.  To create a Z up Irrlicht visualization application, pass `VerticalDir::Z` as the 4th (optional) argument to the ChIrrApp constructor. For example:
```cpp
    ChIrrApp application(&system, L"Demo", irr::core::dimension2d<irr::u32>(800, 600), VerticalDir::Z);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(1, 1, 1));
```
Note that this will also properly orient the sky box.
Rotating with the left mouse button and panning with the arrow and PageUp/PageDwn keys works the same as with a Y up camera.

This API change also eliminates classes with only static methods (ChIrrTools and ChIrrWizard), replacing them with free functions in the `chrono::irrlicht::tools` namespace.  See the various Chrono demos for required changes to user code.

### [Changed] Reading and writing collision meshes in Chrono::Gpu

The mechanism for specifying collision meshes in a `ChSystemGpuMesh` was changed to allow adding meshes in a sequential manner, at any point and as many times as desired, prior to invoking `ChSystemGpuMesh::Initialize()`. Various different functions are provided for adding a mesh from memory:
```cpp
    unsigned int AddMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                         float mass);
```
from a Wavefron OBJ file:
```cpp
    unsigned int AddMesh(const std::string& filename,
                         const ChVector<float>& translation,
                         const ChMatrix33<float>& rotscale,
                         float mass);
```
or adding multiple meshes from a list of Wavefront OBJ files:
```cpp
    std::vector<unsigned int> AddMeshes(const std::vector<std::string>& objfilenames,
                                        const std::vector<ChVector<float>>& translations,
                                        const std::vector<ChMatrix33<float>>& rotscales,
                                        const std::vector<float>& masses);
```

All meshes such specified are offloaded to the GPU upon calling `ChSystemGpuMesh::Initialize()`.  Note that these functions return an integral mesh identifier which can be used in subsequent function calls (e.g., `ChSystemGpuMesh::ApplyMeshMotion()`) to identify a particular mesh.

The Wavefront OBJ file format requirement is changed. The nodal normal information of the meshes, a.k.a. the `vn` lines, are no longer needed by default. The meshes are still directional in contact force calculation, and the normal directions are now implicitly determined by orderings of facet nodes, using the Right-Hand Rule (RHR).

This should not impact the usage of meshes, since for a properly generated OBJ mesh, the orderings of nodes are in line with the outward normals. The users can however, restore the old behavior by calling `ChSystemGpuMesh::UseMeshNormals(true)` before `ChSystemGpuMesh::Initialize()`. If it is called, Chrono::Gpu module will try to rearrange the orderings of facet nodes so that the RHR normals agree with the normals given by the corresponding `vn` lines. 

Chrono::Gpu module now outputs VTK meshes correctly by writing to files the nodal coordinates and connectivities, instead of triangle soups. It also no longer appends `_mesh` to the output filenames. Users can still write all meshes to a single file by 
```cpp
    void WriteMeshes(const std::string& outfilename) const;
```
or write a particular mesh to a file by
```cpp
    void WriteMesh(const std::string& outfilename, unsigned int i) const;
```

## Release 6.0.0 - 2021-02-10

### [Added] New Chrono::Csharp module

The new Chrono::Csharp module provides a C# interface to selected Chrono functionality.  This allows using Chrono from C# programs and facilitates the integration of Chrono with external engines such as Unity.

The module relies on SWIG to automatically generate the interface library and wrapper C# classes.  Upon build, the module creates the wrapper C# files under a `chrono_csharp/` directory in the build tree and a number of shared libraries (dll on Windows, so on Linux) in either the `bin/` or `lib/` directory, depending on platform. Currently, the Chrono::Csharp module provides an interface to the multibody dynamics capabilities in the core Chrono module, as well as to Chrono::Vehicle and the associated vehicle models.

### [Added] RoboSimian, Viper, and LittleHexy models

Models of the legged RoboSimian robot, the wheeled Viper rover, and the six-propeller LittleHexy copter are now included in the collection of Chrono models.  These models have no dependencies beyond the core Chrono module, except for an optional utility class for RoboSimian visualization with Irrlicht. Python wrappers are also provided, allowing use of these models with PyChrono. Related demo programs illustrate the robots moving over rigid or SCM deformable terrain (using a core Chrono system) and over granular terrain (using the Chrono::Multicore module).

### [Added] Contact force reporting through user-provided callback

The `OnReportContact` method of a user-supplied reporter callback (derived from `ChContactContainer::ReportContactCallback`) is now called with the proper force and torque for the current contact when using a Chrono::Multicore parallel system (of either NSC or SMC type).  The reported contact force and torque are provided at the contact point and expressed in the *contact frame* (defined by the provided rotation matrix).

For examples of using the contact reporting feature with a Chrono::Multicore system, see `demo_MCORE_callbackNSC` and `demo_MCORE_callbackSMC`.

### [Changed] Chrono::Gpu module rename

For consistency and to better reflect the purpose of this module, Chrono::Granular was renamed to **Chrono::Gpu**.
With this change, the set of three Chrono modules targeting different parallel hardware (each providing different level of support for different types of simulations) are:
- Chrono::Multicore (for shared-memory multicore parallel computing using OpenMP)
- Chrono::Gpu (for GPU parallel computing using CUDA)
- Chrono::Distributed (for distributed-memory parallel computing using MPI)

The name change for Chrono::Gpu and its associated classes was done in conjunction with a relatively extensive refactoring of its API.  The user's interaction with the Chrono::Gpu module was streamlined by exposing in the public API a single Chrono::Gpu system object (of type `ChSystemGpu` or `ChSystemGpuMesh`) and hidding the underlying implementation in a private class.

See the various Chrono::Gpu demos in the Chrono distribution (e.g., demo_GPU_ballcosim) for usage of the new Chrono::Gpu module. The main API changes are as follows:
- user code only needs to include one Chrono::Gpu header, namely `chrono_gpu/physics/ChSystemGpu.h`;
- optional utilities are available in the `utils/` subdirectory (e.g. `chrono_gpu/utils/GpuJsonParser.h` and `chrono_gpu/utils/ChGpuSphereDecomp.h`; see demo_GPU_ballcosim and demo_GPU_fixedterrain, respectively);
- user must create a Chrono::Gpu object (of type `ChSystemGpu` or `ChSystemGpuMesh`, as appropriate) by specifying the radius of the granular material spherical particles, their density, and the domain size.   This system object intermediates all interactions with the solver (through various setter and getter methods) and provides wrapper functions to initialize the problem (before the simulation loop) and advance the system state (inside the simulation loop);
- note that names of ChSystemGpu methods were changed throughout for uniformity and coherence.

As part of this refactoring, we have also added run-time visualization support for Chrono::Gpu simulations using the Chrono::OpenGL module (if the latter is not enabled in your Chrono build, run-time visualization support is disabled).  While run-time visualization adds some overhead, it may prove to be a useful debugging tool.  To use it:
- include the header `chrono_gpu/utils/ChGpuVisualization`;
- create the visualization object by passing it a pointer to the Chrono::Gpu system and (optionally) a pointer to a Chrono system (if one already exists, e.g. for a co-simulation problem;  if passing `nullptr`, such a system is created automatically);
- initialize the visualization system (this must be done after the Chrono::Gpu system itself was initialized) by invoking the function ChGpuVisualization::Initialize();
- in the simulation loop, at any desired frequency, invoke the function ChGpuVisualization::Render().

See demo_GPU_ballcosim, demo_GPU_mixer, or demo_GPU_repose for use of the run-time visualization option.


Finally, note that a future version of the Chrono::Gpu module may simplify its public API even further by collapsing the two current classes ChsystemGpu and ChSystemGpuMesh into a single one.

### [Changed] Chrono::Multicore module rename

For consistency and to better reflect the purpose of this module, Chrono::Parallel was renamed to **Chrono::Multicore**.

The related API changes are simply replacements of *parallel* with *multicore*, keeping the same capitalization:
- `chrono_multicore/` replaces `chrono_parallel/`
- class names use `Multicore` instead of `Parallel` (e.g.; `ChSystemMulticore`)
- macro names use `MULTICORE` instead of `PARALLEL` (e.g.; `CHRONO_MULTICORE`)
- the CMake project configuration script ChronoConfig.cmake expects the component name `Multicore` instead of `Parallel` 

In addition, names of related demos, unit tests, and benchmark tests include the string `MCORE` instead of `PAR` (e.g.; `demo_MCORE_mixerNSC`).

Users of the Chrono::Multicore module should rerun CMake since the variables related to this module have also changed name (e.g.; `ENABLE_MODULE_MULTICORE`).

### [Added] Geometric stiffness for Euler beams

The geometric stiffness term is now introduced also for the chrono::ChElementBeamEuler beam element (Euler-Bernoulli corotational beams). It is turned on by default, and it is computed via an analytical expression, with minimal cpu overhead. 
Note that geometric stiffness was already considered in IGA and ANCF beams, only the Euler beam was missing. Geometric stiffness is responsible of the fact that if you pull a thin beam like a string, its natural frequencies will increase, or viceversa, if you push it, its lateral stiffness decreases up to buckling instability. 
Note that Euler beams ware able to simulate buckling or pulled-string stiffening even before, but only doing time integration in multiple time steps: instead, if one exported the M,K matrices for doing modal analysis of a pre-stretched Euler beam after a static analysis, the K matrix was missing the contribution of the geometric stiffness hence frequencies were uncorrect only in modal analysis.


### [Added] New Chrono::Synchrono module

The new `Chrono::SynChrono` (or simply SynChrono) module has been introduced. SynChrono aims to provide an easier entry point for physics-based autonomous vehicle simulations, and to this end it uses MPI to parallelize simulations in the case where there is no physical interaction between agents. For example in a simulation of two vehicles driving separately, there is no need to simulate interaction between them, yet they must have some knowledge of each other for visualization and for any sensors that they may carry.

SynChrono is equipped to synchronize any "agent" (e.g. an arbitrary robot whose state is defined by some combination of mechanisms), but currently there are concrete wrapper classes for synchronizing `Chrono::Vehicle`'s, these are `SynWheeledVehicleAgent` and `SynTrackedVehicleAgent`. Another example of an agent, that can be used as a model for a user-defined agent, is `SynEnvironmentAgent` which represents a smart traffic intersection. Synchronization of `SCMDeformableTerrain` is also possible.

While SynChrono's primary purpose is to synchronize the state of agents, the MPI communication that synchronizes state data can also be used to send other messages. Examples of these could be messages from an intelligent traffic light to a vehicle (see `flatbuffer/message/Syn[SPAT/MAP]Message`) or from a vehicle to a vehicle with some sensor information (see `SynSensorMessage`). SynChrono supports both `Chrono::Irrlicht` and `Chrono::Sensor`-camera rendering of scenes, and `visualization/` packages some simple defaults along with a class to facilitate easy swapping between the two.

### [Changed] Rename Intel MKL Pardiso interface module

For consistency and clarity, the `Chrono::MKL` module was renamed to `Chrono::PardisoMKL` (indeed, this module interfaces only to the sparse direct linear solver Pardiso from the Intel MKL library).  From a public API perspective, this name change requires the following changes to user code:

- Include header

   ```cpp
   #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
   ```

- The new solver name is `ChSolverPardisoMKL`.  For example:

   ```cpp
   auto my_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
   my_solver->LockSparsityPattern(true);
   my_system.SetSolver(my_solver);
   ```

- Solver type enum value for this solver is now `ChSolver::Type::PARDISO_MKL`

- Use in CMake project configuration script

  To request this new module when configuring an external project to use Chrono, use the component name `PardisoMKL` in your CMake call to `find_pakage(Chrono...)`.  Recall that the names of the components are case insensitive


### [Added] Saving POV-Ray files from Irrlicht interactive view

New feature in the Irrlicht interactive 3D view. When pressing the F12 key, a directory `povray_project` is immediately created on disk, and .pov .ini .assets etc. files are created inside it, so that later you can use POVray to load the .ini and render the simulation with high quality ray tracing. Press F12 again to stop saving the POVray files. Note that you must later edit the `povray_project/render_frames.pov` to change/add the lights, global illumination, and other specific raytracing settings.
This feature is available only if you build also the `POSTPROCESS` module, so check *both* `ENABLE_MODULE_IRRLICHT` and  `ENABLE_MODULE_POSTPROCESSING` in CMake.

Also, the API of the `ChPovRay` class has been simplified. One just uses the new `SetBasePath()` function to set the directory that will contain all .ini, .pov, etc. files, and anim/, output/ subdirectories. The user does not need to create these folders anymore, these are automatically generated if necessary, when setting up ChPovRay with `ExportScript()`. Also, some features of ChPovRay have been fixed / improved.


### [Added] Support for modelling wheeled trailers

New templates were added to Chrono::Vehicle to allow creating wheeled trailers.  A trailer is an assembly consisting of a "rear chassis" (see `ChChassisRear`), an arbitrary number of `ChAxle` subsystems (each including a suspension subsystem, 2 or 4 wheels, and optionally brake subsystems), and a hitch connector (see `ChChassisConnectorHitch`) for attaching the trailer to a vehicle chassis.

Similar to a wheeled vehicle, a concrete trailer system can be implemented in concrete C++ classes (see the Kraz semi-trailer truck model and `demo_VEH_Kraz_OpenLoop`), or else through JSON specification files (see the files in `data/vehicle/ultra_tow` and `demo_VEH_WheeledJSON`).

A concrete wheeled trailer system class implements the abstract class `ChWheeledTrailer`. A trailer can be attached to any vehicle chassis and is initialized based on the vehicle pose, the hitch location on the vehicle chassis (see `ChChassis::GetLocalPosRearConnector`), and the hitch location on the trailer chassis (see `ChChassisRear::GetLocalPosFrontConnector`):

```cpp
WheeledTrailer trailer(vehicle.GetSystem(), trailer_JSON_file);
trailer.Initialize(vehicle.GetChassis());
trailer.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
trailer.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
trailer.SetWheelVisualizationType(VisualizationType::NONE);
for (auto& axle : trailer.GetAxles()) {
    for (auto& wheel : axle->GetWheels()) {
        auto tire = ReadTireJSON(trailer_tire_JSON_file);
        trailer.InitializeTire(tire, wheel, VisualizationType::PRIMITIVES);
    }
}
```

### [Changed] Enhancements to Chrono::FSI

- The WCSPH based explicit solver now supports both fluid dynamics and granular material dynamics.

	- The fluid dynamics is executed by default.
	- The granular material dynamics is executed by setting an "Elastic SPH" option in the input JSON file.

- Add a consistent SPH discretization into the explicit SPH solver.

	- Both the gradient and Laplacian operators in the NS equations are discretized by a consistent format.
	- The correction matrices are calculated for both operators to enhance the consistency.
	- A second-order accuracy will be recovered by this consistent discretization.

- Add a new particle shifting technique into Chrono::FSI.

	- The particle shifting strategy is a penetration-based particle shifting technique.
	- It supports three-dimensional fluid/granular material dynamics problems with a free surface.

- Make the granular material solver more stable and accurate in the framework of WCSPH.

	- The Drucker-Prager yield criterion is implemented in conjunction with a four-step update strategy for the stress tensor of the granular material.
	- The interaction between a rigid multibody system and granular material is supported.

### [Added] New Chrono::Sensor module

A new module (`Chrono::Sensor`) has been introduced to allow for sensor simulation within Chrono. `Chrono::Sensor` provides an interface for modeling and simulating sensors in the Chrono system to provide input for perception and control algorithms. For example, `Chrono::Sensor` may be used in combination with `Chrono::Vehicle` to simulate an autonomous vehicle equipped with multiple cameras and lidars. The module containes a API for modeling sensors with noise and distortion using a filter-graph paradigm for customization. Rendered sensors (camera and lidar) utilize ray tracing via OptiX to generate synthetic data.

Parameterized models for camera, lidar, GPS and IMU have been added with the ability to extend or implement custom sensors.

`Chrono::Sensor` is designed around a `ChSensorManager` which maintains all time synchronization and resources management between the sensing module and the core chrono system. Sensors such as a `ChCameraSensor` and `ChLidarSensor` can be added to the manager and mounted to a Chrono body which will determine the sensor's dynamics. The sensor are maintained by the `ChSensorManager` and all data is received via an added `ChFilterAccess` in the filter graph, determined by the sensor's parameters.

Locations:
- `Chrono::Sensor` source code is maintained under `src/chrono_sensor/`
- Demos are located in `src/demos/sensor/`
- Sensor specific data is located in `data/sensor/`

### [Changed] Setting OpenMP number of threads

The mechanism for setting the number of OpenMP threads used in various parts of Chrono has been modified and unified. The API is common to Chrono and Chrono::Parallel; however, the number of OpenMP threads is set differently for the two classes of systems.

- ChSystem: ChSystemNSC and ChSystemSMC

  OpenMP (enabled by default) may be used in three different places. The number of threads used for each can be set separately and independently, using:
  ```cpp
  my_system.SetNumThreads(nthreads_chrono, nthreads_collision, nthreads_eigen);
  ```
  If passing a value of 0 for either `nthreads_collision` or `nthreads_eigen` these values are set to be equal to `nthreads_chrono`. 

  - Currently, Chrono itself uses OpenMP for the parallel evaluation of internal forces and Jacobians for FEA and for parallel ray-casting in SCM deformable terrain. In both cases, the value `nthreads_chrono` is used in a **num_threads** clause for the OpenMP parallel for loops.
  - The latest Bullet collision detection system embedded in Chrono is built by default with OpenMP support (this can be disabled during CMake configuration). The value `nthreads_collision` is used through **num_threads** clauses in all Bullet-internal OpenMP parallel for loops (note that this is a Chrono-specific modification to Bullet).
  - Eigen uses OpenMP in a few algorithms. For Chrono use, the most relevant ones are the Eigen sparse direct solvers, SparseLU and SparseQR. These will employ the number of threads specified as `nthreads_eigen`.

  By default, that is if `SetNumThreads` is not called, we use `nthreads_chrono=omp_get_num_procs()`, `nthreads_collision=1`, and `nthreads_eigen=1`.

- ChSystemParallel: ChSystemParallelNSC and ChSystemParallelSMC

  In Chrono::Parallel, the same number of OpenMP threads (default `omp_get_num_procs()`) is used for both the parallel collision detection algorithm and for the parallel iterative solvers.
  In the call to `SetNumThreads`, the value `nthreads_collision` is ignored and automatically set to be equal to `nthreads_chrono`.  As such, typical user code will have
  ```cpp
  my_system.SetNumThreads(nthreads);
  ```

- The number of OpenMP threads used by the sparse direct solvers in Chrono::MKL (Pardiso) and Chrono::MUMPS are specified as an optional constructor argument.  By default, both solvers use a number of threads equal to the number of available processors (as returned by `omp_get_num_procs`).


### [Changed] Redesigned SCM deformable terrain

The SCM deformable terrain was completely redesigned for improved performance. Compared to the previous implementation based on an underlying trimesh representation, the new code - using a Cartesian grid - is significantly faster (speedups of 50x and more).  The API changes are minimal:

- Initialization from a Wavefront OBJ file was removed.  An SCM terrain patch can be specified as a flat rectangular area or else as a height-field obtained from a (gray-scale) image.
  
  A flat SCM terrain patch can be initialized using
  ```cpp
  terrain.Initialize(length, width, resolution);
  ```
  where `length` and `height` are the patch dimensions in the reference plane and `resolution` is the grid spacing. Note that the height (level) of the patch is implicitly defined by the center of the ACM reference plane (specified through `SCMDeformableTerrain::SetPlane`).

  A height-field SCM terrain patch can be initialized using
  ```cpp
  terrain.Initialize(filename, sizeX, sizeY, min_height, max_height, resolution);
  ```
  where `filename` is the name of an image file, `sizeX` and `sizeY` specify the patchg extents in the reference plane, `min_height` and `max_height` define the height range (a purely black image pixel corresponds to min_height, a purely white pixel corresponds to max_height) and `resolution` is the SCM grid spacing.

- The option for adaptive mesh refinement was obsoleted. Performance of the new implementation is limited by the ray-casting operations and as such no additional benefits are obtained from starting with a coarse grid.

- A "moving patch" is now defined by specifying an object-oriented-box attached to a moving body. For example,
  ```cpp
  terrain.AddMovingPatch(my_body, ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));
  ``` 
  associates a moving patch with the box of size (5,3,1) attached at the center of the body reference frame of `my_body`.  Ray casting is performed only for the SCM grid nodes that are in the current projection of this OBB onto the SCM reference plane.

  If the user does not define any moving patches, SCM uses the projection of the current bounding box of all collision shapes in the system.

- Bulldozing effects are enabled using `SCMDeformableTerrain::EnableBulldozing`.
- SCM soil parameters and bulldozing settings are specified as before.

### [Added] Tracked vehicle support in PyChrono

Tracked vehicle templates and models are now exposed in Chrono::Python and available for use through PyChrono.


### [Changed] Constitutive models for EULER beams

Section properties of the ChElementBeamEuler are now defined via a **new class** `ChBeamSectionEuler` and subclasses. Old classes for Euler sections have been renamed and rewritten, the old classes have been **deprecated** and will be removed in future:
 - `ChBeamSectionBasic`, use  `ChBeamSectionEulerSimple` instead
 - `ChBeamSectionAdvanced`, use  `ChBeamSectionEulerAdvanced` instead
 
Note that in the previous release, the Sy and Sz values for **shear center** offset in `ChBeamSectionAdvanced` were assumed with **opposite sign** respect to the description and illustrative figure: now this bug is fixed, and shear center offset works the same as in Cosserat beams.

Also, a new class  `ChBeamSectionEulerGeneric` has been added, that does not make the assumption of uniform density and uniform elasticity, so it accepts directly the beam rigidity values bypassing the E and Izz Iyy values. 
 
To speed up coding in case of simple beams, two new classes `ChBeamSectionEulerEasyRectangular` and `ChBeamSectionEulerEasyCircular` have been added.


### [Changed] Constitutive models for IGA beams

Inertial properties of Cosserat beams, such as the ChElementBeamIGA, are now defined via a **new class** `ChInertiaCosserat` and subclasses, that can be composed into `ChBeamSectionCosserat` just like we do for elastic properties, damping, etc. This is more flexible than before. Therefore these functions have been **removed**:
 - `ChBeamSectionCosserat::SetDensity()`
 - `ChBeamSectionCosserat::SetArea()`

**Consequences:**
 - the ChBeamSectionCosserat constructor has changed: it always requires a ChInertiaCosserat too.
 - you should create a ChInertiaCosserat, like ChInertiaCosseratUniformDensity, set density and area there, and add the ChInertiaCosserat to the ChBeamSectionCosserat.
 - the polar part of inertia was using the width-height values of the visualization, that was limiting and often not related to the beam; now it is a physical value
  - the rotational inertia (in lumped mass matrix) of the beam is more precise

Moreover, also because of this modification, and in order to make the API less ambiguous, these functions have been **removed**:
 - `SetAsCircularSection()`
 - `SetAsRectangularSection()`
from the ChBeamSectionCosserat *and* from all elastic/damping/plastic models. We kept them only for the elastic models where they make sense. 

**Consequences:** 
 - previously one could call `ChBeamSectionCosserat::SetAsRectangularSection()` and automatically invoke the same for all sub-models (elasticity, etc.) whereas now one has to call `SetAsRectangularSection()` for the single sub-models, only when needed and when available. 
 - To make things easier, we provide the new classes `ChBeamSectionCosseratEasyRectangular`
and `ChBeamSectionCosseratEasyCircular` that in a single shot create elastic and inertia models, sets them as rectangular or circular, and sets the visualization type.


### [Added] Obtaining body applied forces

The new functions `ChBody::GetAppliedForce` and `ChBody::GetAppliedTorque` return the body resultant applied force and torque, respectively.

1. These functions include contributions from all external applied loads acting on a body (e.g., gravitational forces, forces defined through ChForce elements, forces specified through ChLoad, springs, dampers, etc).
   However, they **do not** include any constraint forces.  In particular, this means that contact forces are not included when using the NSC formulation, but are included when using the SMC formulation.
   For the former case, use `ChBody::GetContactForce` and `ChBody::GetContactTorque` to obtain the resultant contact force and torque, respectively.
   
2. Note that reporting this information requires a traversal of the entire system and caching the generalized forces, a quantity that is otherwise not computed in the form required for this reporting.  To prevent any additional overhead when this information is not requested by the user, this is done using lazy evaluation.   In other words, no overhead is incurred at a simulation step if no applied forces are requested. On the other hand, there is a small (but non-zero) cost when a call to `ChBody::GetAppliedForce` or `ChBody::GetAppliedTorque` is made; however, this cost is constant at any given time, regardless of how many queries are made.  Note also that this additional cost is not incurred for Chrono::Parallel.


### [Added] Chrono::Vehicle simulation world frame

While the default world frame for Chrono::Vehicle simulations is an ISO (Z up) frame, we now provide support to simulate vehicles in a scene specified in a different reference frame (for example, an Y up frame).
The world frame is uniquely defined through a rotation matrix (the rotation required to align the ISO frame with the desired world frame). To change the world frame definition from the default ISO convention, the desired world frame must be set **before** any Chrono::Vehicle library call:
```cpp
ChMatrix33<> world_rotation = ...
ChWorldFrame::Set(world_rotation);
```
A shortcut is provided to specify a world frame with Y up (and X forward, Z to the right):
```cpp
ChWorldFrame::SetYUP();
```


### [Changed] CASCADE module

1.	Support for OpenCASCADE 7.4.0. The API of OpenCASCADE introduced some changes in the 7.4.0 version so we also updated the CASCADE module of Chrono. Please download and upgrade your OpenCASCADE version as it is not backward compatible. (The module is optionally built via CMake configuration flag ENABLE_MODULE_CASCADE, also remember to update the CASCADE_INCLUDE_DIR and CASCADE_LIBDIR paths and to update your PATH if you added the path to Cascade dlls)

2.	The method `ChCascadeDoc::CreateBodyFromShape()` is obsolete. Just use the `ChBodyEasyCascade` class to obtain the same result, for example:
    ```cpp
    auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(myshape, ...);
    ```

3.	The mesh tesselation algorithm could give coarser or finer meshes with respect to the previous release.


### [Changed] Collision shapes and contact materials

The main change is that now contact materials are associated with collision shapes, as opposed to bodies.  We've always had the underlying concept of a collision shape, with a body's collision model potentially including multiple shapes, but these were always sharing the exact same contact material.   With the new code, each collision shape in a collision model can have its own contact material properties.  Of course, through shared pointers, collision shapes in the same model or even in different models can still point to the same (shared) material.   Also, a nice consequence of this change is that now a ChBody is agnostic of contact method or contact materials (as it should be).

The new API requires you to always pass a material (as a shared pointer) whenever you create a collision shape or other contactable "primitive" (such as a contact surface or node cloud for FEA collision).   The material you pass must still be consistent with the *contact method* (NSC or SMC) of the containing system.

Here's a summary of the main API changes (there were many other changes under the hood, but here we only highlight changes to the public API).  Moreover, we discuss only the C++ API, but the equivalent changes also apply to the Python API.

1. Renamed headers.  For consistency, we renamed various files under src/chrono/collision which had a prefix `ChC` to simply have the prefix `Ch`.  For example, `ChCCollisionModel.h`  -->  `ChCollisionModel.h`

2. The contact method, NSC or SMC, is now a top-level enum class named `ChContactMethod` (previously it was nested under ChMaterialSurface).  So use things like:
   ```cpp
   if (system->GetContactMethod() == ChContactMethod::NSC) {
       ...
   }
   ```

3. Contact materials. The functionality of the base class `ChMaterialSurface` and derived classes `ChMaterialSurfaceNSC` and `ChMaterialSurfaceSMC` is unchanged.  However, for convenience, material properties common between NSC and SMC were moved to the base class (these include coefficients of friction for sliding, rolling, spinning and the coefficient of restitution). Furthermore, we provide a utility method to create a contact material of the specified type with corresponding default parameters.  In a program that can switch between SMC and NSC formulations (say, based on a flag `ChContactMethod contact_method`), you can then write
   ```cpp
   ChContactMethod contact_method = ChContactMethod::SMC;  // or ChContactMethod::NSC
   ...
   auto mat = ChMaterialSurface::DefaultMaterial(contact_method);
   mat->SetFriction(0.7f);
   ```
   If you also want to change a contact method-specific property, you must then use something like:
   ```cpp
   if (contact_method == ChContactMethod::SMC) {
      std::static_pointer_cast<ChMaterialSurfaceSMC>(mat)->SetYoungModulus(1e7f);
   }
   ```

4. The `ChBody` constructor does not take the contact method as an argument anymore (which previously defaulted to NSC)

5. `ChBodyEasy***` classes. The list of constructor arguments here has changed, for a more natural order.  All collision-related arguments have been moved to the end of the list. In particular, the flag to indicate whether or not to create a visual asset (default `true`) comes before the flag to indicate whether or not to create a collision shape (default `false`).  If the latter is `true`, the next argument must be a contact material (default `nullptr`).

   Be careful here, as it's easy to overlook the correct changes (because of arguments with default values).  For example, the old:
   ```cpp
   auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, density, true);  // old
   ```

   would create a sphere with visualization and collision enabled (with material properties from the body itself).  This is also valid with the new code, but this will now create a body with a sphere visualization asset but **no** collision shape.  To get the expected result, you need to use:

   ```cpp
   auto body = chrono_types::make_shared<ChBodyEasySphere>(radius, density, true, true, material);  // new
   ```
   and pass a valid material (consistent with the system to which you will add the body).

6. There is now a proper `ChCollisionShape` class which is very light weight (it carries only the type of shape, from an enum ChCollisionShape::Type, and a (shared) pointer to a contact material).  There are derived classes for the various collision systems (namely the one based on Bullet and the one used in Chrono::Parallel), but few users would have to worry about or work with those.

7. A collision model maintains a vector of collision shapes (in the order they were added to the model by the user).  There are public accessor methods on ChCollisionModel to get the list of shapes or individual shapes (by its index in the list).

8. Adding collision shapes.  All `ChCollisionModel::Add***` methods now take as their first argument a (shared) pointer to a contact material.  There is no "default" material automatically constructed under the hood for you anymore.   However, the NSC and SMC contact materials still have constructors that set their respective properties to some default values.  So you can write something like:
   ```cpp
   // construct an SMC material with default properties
   auto my_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>(); 
   auto my_body = chrono_types::make_shared<ChBody>();  // note: no need to specify SMC contact here!
   ...
   my_body->GetCollisionModel()->AddSphere(my_mat, radius);
   ...
   my_system.AddBody(my_body);  // it is assumed that my_system is a ChSystemSMC
   ```

9. Utility functions in ChUtilsCreators.  Similar to `ChBodyEasy***`, the various `utils::Add***Geometry` functions now also require a contact material;  this is always their 2nd argument.

10. FEA contact surfaces.  The two options, `ChContactSurfaceMesh` and `ChContactSurfaceNodeCloud` now require a contact material at construction (there is no SetSurfaceMaterial() method anymore).  As you already know, the only acceptable type of material in this case is ChMaterialSurfaceSMC.  Note that the contact material passed at construction is shared by all components of the FEA contact surface (triangles or nodes, respectively).  Sample code:
    ```cpp
    auto my_surf = chrono_types::make_shared<ChContactSurfaceMesh>(my_materialSMC);
    my_mesh->AddContactSurface(my_surf);
    my_surf->AddFacesFromBoundary(0.05);
    ```

11. `ChCollisionInfo` and user-provided contact callbacks.  A ChCollisionInfo object (which encapsulates information about a collision pair and is used internally to create contacts) now also include pointers to the colliding shapes in the two interacting collision models.   When a contact must be created internally, a composite contact material is created on the fly from the contact materials of the two colliding shapes.

    This has a direct impact on how a user can add custom contacts through a `ChSystem::CustomCollisionCallback` object.   Indeed, in its override of OnCustomCollision, a derived callback class is expected to create a ChCollisionInfo object and then pass it to the AddContact method of the underlying contact container.  However, such a custom collision has no collision shapes to work with!   For this purpose, we added a new form of `AddContact` that takes as additional argument to ChMaterialSurface objects.   In this case, the ChCollisionInfo object can set its collision shape members to `nullptr`.

    For examples, look at two new demos: `demo_IRR_custom_contact` and `demo_PAR_custom_contact`.

12. `Chrono::Vehicle` changes.  Most of the changes here were low-level (and as such transparent to the user).  The notable difference has to do with terrain specification.  The RigidTerrain::AddPatch function now expect as their first argument a contact material (consistent with the containing system) that will be used for that one patch (box patch, trimesh patch , or height-field patch).  

    If you are using vehicles specified through JSON files, beware that for some subsystems the schema has changed for entries related to collision shapes and contact materials.   See the examples provided in the various sub-directories of `data/vehicle/`.

13. `Chrono::Parallel` and `Chrono::Distributed`.  Here too, the vast majority of the changes happened under the hood.  Besides the changes described above (related to how collision shapes are defined), there are only a couple of things to mention:
 - In Chrono::Parallel, the custom specification of collision shapes was merged into a new class ChCollisionShapeParallel.  If, for any reason, you need to traverse the list of collision shapes in a collision model, simply loop through the vector of shapes in ChCollisionModel (see #7 above).
 - In Chrono::Distributed, the analytical plane boundaries now require a contact material at construction (internally, this is used in a custom collision callback, as described in #11 above).


## Release 5.0.1 - 2020-02-29

### [Fixed]

- Correct the ChElementBeamANCF applied moment calculation so that it uses normalized shape function derivatives
- Comment out code related to applying moments on ANCF elements (requires further testing))

## Release 5.0.0 - 2020-02-24

### [Changed] Refactoring of dense linear algebra

Starting with this release, Chrono relies on Eigen3 for all dense linear algebra.

1. With this change, Chrono requires `Eigen3` version 3.3.0 or newer.  Unless not possible for other reasons, we suggest you use their latest version, 3.3.7.
	- Eigen is available for download at http://eigen.tuxfamily.org/
	- Eigen is a headers-only library, so no install is required in order to use it in Chrono.
	- If the location of the Eigen headers is not automatically detected by CMake, manually specify it by setting the CMake variable `EIGEN3_INCLUDE_DIR`.
	- Note: CUDA 9.1 has removed a file (`math_functions.hpp`) which is referenced by Eigen prior to version 3.3.6.  As such, if you build any of the GPU-based Chrono modules (FSI or Granular) and use CUDA 9.1, make sure to use Eigen 3.3.7.

2. The switch to Eigen does come with a few strings attached
	- On the flip side, this makes the code a lot cleaner, easier to understand and maintain, and facilitates several further developments on our road-map.
	- Furthermore, performance did not degrade and in fact improved meaningfully on practically all our benchmark tests (measured with GCC and clang on Linux and MacOS).
	- On the other hand, libraries that use expression templates metaprogramming techniques (as Eigen does) lead to longer compile times. Moreover, while release (optimized) code is faster, code built in debug mode will likely be (significantly) slower.
	- Finally, because several Chrono classes now need to use an (Eigen-provided) overloaded operator new to ensure memory alignment and because of some limitations of the C++ language, this change has an important consequence on user code: std::make_shared **cannot always be used safely**. The solution we adopted is to provide `chrono_types::make_shared` replacement functions which should be used throughout (see below).

3. The new matrix and vector classes
	- The "old" Chrono matrix types (`ChMatrixNM`, `ChMatrixDynamic`, `ChVectorDynamic`) are now nothing but *type aliases* of appropriate Eigen types (`see ChMatrix.h`). 
	In other words, use them as you would Eigen types (see https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html).
	- For completeness, we introduced additional types (such as `ChVectorN`,  `ChRowVectorN`, `ChRowVectorDynamic`), all defined in ChMatrix.h.
	- We only implemented a handful of extensions needed for Chrono (such as a method for computing the WRMS vector norm) using the "back-door" extension mechanism provided by Eigen (see `ChMatrixEigenExtensions.h`)
	- The "old" base class ChMatrix was **eliminated**.  For the instances where a Chrono function needs to accept either a dynamic (ChMatrixDynamic) or a static (ChMatrixNM) argument, we rely on the `Eigen::Ref` mechanism. For this, we defined various type aliases, such as `ChMatrixRef` and `ChMatrixConstRef` (see ChMatrix.h).  Currently, these assume a 'double' scalar type as this is all that's needed in Chrono (they could be templated by the scalar type, but this will be done only if absolutely needed).
	- For 3x3 matrices, the `ChMatrix33` class (as before, templated by the scalar type) is derived from a 3x3 fixed-size vectorizable Eigen matrix.  Using inheritance here was needed in order to implement various custom constructors and methods.  
	- For clarity and to properly separate responsibilities, we added a small set of 3x4 and 4x4 matrices specific to multibody dynamics.  These derived classes have strict and limited functionality and are unlikely to show up in user code (see `ChMatrixMBD.h`).
	- Note: for now, the ChVector and ChQuaternion classes were left unchanged.
	- See `demo_CH_linalg.cpp` for simple examples of matrix operations with the new Chrono classes and types.

4. Obsolete/eliminated Chrono classes
	- With the switch to Eigen, we **removed** `ChLinearAlgebra`.  More robust and varied matrix factorization and linear system solvers can be used directly from Eigen (see http://eigen.tuxfamily.org/dox/group__DenseLinearSolvers__chapter.html).
	- We also **removed** the custom sparse matrix classes `ChLinkedListMatrix` and `ChMapMatrix`.  

5. Some considerations for developers
	- Consult the Eigen documentation (http://eigen.tuxfamily.org/dox/) and FAQs to understand what Eigen provides and how it should be used (e.g., when it's meaningful/appropriate to use fixed-size Eigen objects)
	- Look at block operations with Eigen matrices and vectors (https://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html)These make code much more clear (contrast this with the use of the old "paste" functions which was obfuscated and prone to mistakes).   However, always measure performance impacts;  in some cases (especially with smaller matrices), explicit loops may be more efficient than Eigen block operations. 
	- Beware of aliasing issues (https://eigen.tuxfamily.org/dox/group__TopicAliasing.html).  
	- Beware of memory alignment issues (https://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html)
	In particular, if you create a class that has a fixed-size vectorizable Eigen type (a ChMatrixNM, a ChVectorN, or most common a ChMatrix33) make sure to overload its operator new.  For that, use the Eigen-provided macro `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` (see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)
	- When creating shared pointers, make sure to use **`chrono_types::make_shared`** (see below)

6. API and user code
	- Except for the limited situations where user code explicitly used the old-style matrix operations (which should now use the cleaner Eigen API; see e.g. https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html), the main impact on users has to do with the shared pointers in the Chrono public API.
	- The issue has to do with the fact that std::make_shared uses 'placement new' to create a shared pointer with a single memory allocation.  This means that std::make_shared will not use an overloaded 'operator new' for classes that provide it (for memory alignment considerations).
	- To maintain encapsulation (as much as possible) and not require users to know/care about the guts of Chrono (namely know which classes overload, or have a parent that overloads, their operator new), the solution we adopted is to provide **custom** make_shared functions (in the namespace chrono_types) and a mechanism to pick at compile-time which one should be used (conditional on the object's class having or not an overloaded operator new).   For classes that require aligned memory (i.e. classes that have an overloaded operator new), **`chrono_types::make_shared`** explicitly creates the shared_ptr.   Otherwise, it falls back on using std::make_shared (see `ChTypes.h`).
	- As such, user code should always create a shared_ptr using something like:
	~~~.{.cpp}
	auto foo = chrono_types::make_shared<T>(bar);
	~~~
	- See the various Chrono demos and tests for examples.

### [Changed] Eigen sparse matrices and updates to direct sparse linear solvers

Starting with this release, Chrono also uses Eigen for all sparse matrix needs (which are relatively limited and have little, if any, direct consequence on the public API).
A `ChSparseMatrix` is just a **type alias** to an Eigen SparseMatrix with double scalar type, row-major storage order, and int storage index type.  

1. The main effects on public API relate to concomitant updates we made to the sparse direct linear solvers (the Chrono::MKL interface to the Intel MKL Pardiso solver and the Chrono::Mumps interface to the MUMPS solver).  While we now rely on Eigen’s own interface to Pardiso, Chrono::Mumps still implements a custom interface to MUMPS (ChMumpsEngine).  

	For examples of usage, see `demo_FEA_cablesMKL` and `demo_FEA_cablesMUMPS`.

2. Both sparse direct solvers (and any others that may be provided in the future) share the same functionality in terms of controlling the identification and update of the matrix sparsity pattern.  The main features implemented in the base class `ChSolverDirect` are:
	- *sparsity pattern lock*
	The sparsity pattern lock skips sparsity identification or reserving memory for non-zeros on all but the first call to the solver setup. This feature is intended for problems where the system matrix sparsity pattern does not change (or changes very little) from call to call.  See ChSolverDirect::LockSparsityPattern.
	- *sparsity pattern learning*
	The sparsity pattern learning feature acquires the sparsity pattern in advance, in order to speed up matrix assembly.  Enabled by default, the sparsity matrix learner identifies the exact matrix sparsity pattern (without actually setting any non-zeros). See ChSolverDirect::UseSparsityPatternLearner.
	- In situations where the problem structure changes but relatively infrequently, it is still desirable to lock the sparsity pattern.  However, if using the sparsity pattern learner, an update must be explicitly triggered by the user after a problem modification (by calling `ChSparseDirect::ForceSparsityPatternUpdate`).  For an example, `see demo_FEA_beams_extrude`.
	- Finally, there is an option to provide an estimate for the matrix sparsity (a value in [0,1], with 0 corresponding to a fully dense matrix). When the sparsity pattern learner is disabled, this value is used if/when required to reserve space for matrix indices and non-zeros. See `ChSolverDirect::SetSparsityEstimate`.

3. If appropriate and warranted by the problem setup, it is *highly recommended* to enable the sparsity pattern lock. This can significantly improve performance for more complex problems (larger size and/or problems which include constraints).

## Release 4.0.0 - 2019-02-22

In addition to various bug fixes and enhancements, Chrono v4.0.0 includes the following main updates:
- Adopted Google test and benchmark libraries for unit testing and performance benchmarking.
	- these are set up as Git submodules
	- unit tests relocated under src/tests/unit_tests; several unit tests were migrated to use the Google test framework
  	- examples in src/tests/benchmark_tests illustrate use of the Google benchmark framework
- Core Chrono module
	- new LinkMotor elements, for modeling linear and rotational motors
	- new parsers for generating Chrono models from ADAMS adm files and OpenSim osim files (these parsers provide partial support)
- Chrono::FEA
	- FEA support is now included in the core Chrono module and not as an optional module anymore
	- new IGA (isogeometric) beam element
	- new 8-node, high-order ANCF shell element
- Chrono::MUMPS
	- new optional module providing an interface to the MUMPS sparse direct linear solver
- Chrono::Distributed
	- new optional module for MPI_based distributed parallel simulation of granular dynamics problems
	- currently only supports SMC contact formulation (penalty approach) and a reduced number of collision shapes (spheres and triangular meshes)
- Chrono::FSI
	- support for Implicit Incompressible Smoothed Particle Hydrodynamics
	- support for fluid-solid interaction with ANCF cable/shell elements
	- new GPU-based sparse linear solvers via CUBLAS and CUSPARSE libraries
- Chrono::Python
	- Python interface to Chrono now distributed as an Anaconda package
	- new interface to the Chrono::MKL and Chrono::Cascade modules
	- several new demos, including demo for use with TensorFlow
- Chrono::Vehicle
	- new tire models (TMeasy, Pacejka89)
	- new templates for continuous-band tracks
	- new suspension templates
	- new vehicle models (generic sedan, UAZ van)
