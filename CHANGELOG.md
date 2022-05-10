<!-- For syntax, see  https://github.github.com/gfm/ -->


Change Log
==========

- [Unreleased (development version)](#unreleased-development-branch)
  - [Redesigned run-time visualization system](#changed-redesigned-run-time-visualization-system)
  - [Vehicle inertia properties](#changed-vehicle-inertia-properties)
  - [CMake project configuration script](#changed-cmake-project-configuration-script)
  - [Right-handed frames in Chrono::Irrlicht](#changed-right-handed-frames-in-chronoirrlicht)
  - [Modal analysis module](#added-modal-analysis-module)
  - [Callback mechanism for collision debug visualization](#added-callback-mechanism-for-collision-debug-visualization)
  - [Translational and rotational spring-damper-actuators](#changed-translational-and-rotational-spring-damper-actuators)
  - [Refactor Chrono::Vehicle suspension test rigs](#changed-refactor-chronovehicle-suspension-test-rigs)
- [Release 7.0.3](#release-703---2022-04-17)
- [Release 7.0.2](#release-702---2022-04-03)  
- [Release 7.0.1](#release-701---2022-01-07)  
- [Release 7.0.0](#release-700---2021-11-15) 
  - [DDS communicator in Chrono::Synchrono module](#added-dds-communicator-in-chronosynchrono-module)
  - [New terramechanics co-simulation module](#added-new-terramechanics-co-simulation-module)
  - [Chrono::Fsi API redesign](#changed-chronofsi-api-redesign)
  - [Sensor performance improvements and feature additions](#changed-sensor-to-improve-performance-and-added-features)
  - [ANCF element improvements and additions](#changed-ancf-element-improvements-and-additions)
  - [New Chrono::Vehicle features](#added-new-chronovehicle-features)
  - [New robot models](#added-new-robot-models)
  - [New multicore collision detection system](#added-new-multicore-collision-detection-system)
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
  - [Support compiling to WebAssembly](#added-support-for-the-emscripten-compiler-targeting-webassembly)
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

### [Changed] Redesigned run-time visualization system

The entire mechanism for defining visualization models, shapes, and materials, as well as constructing and attaching a run-time visualization system to a Chrono system was redefined for more flexibility and to allow plugging in alternative rendering engines.  The new class hierarchy allows sharing of visualization models among different physics items, visualization shapes among different models, and visualization materials among different shapes.

- A visualization material (`ChVisualMaterial`) defines colors (diffuse, ambient, specular, and emissive), textures, and other related properties.
- A visualization shape (`ChVisualShape`) is a geometric shape (primitive, curve, surface, or triangular mesh) with one or more associated visualization materials. If a shape has no associated material, a default material is used.
- A visualization model (`ChVisualModel`) is an aggregate of (pointers to) shapes and a transform which specifies the shape position relative to the model reference frame. Visualization shapes in a model are maintained in a vector of `ShapeInstance` (which is simply a typedef for a pair containing a shared pointer to a `ChVisualShape` and a `ChFrame`). Note that, currently a visualization model instance cannot be placed inside another visualization model, but that may be added in the future.
- A visualization model instance (`ChVisualModelInstance`) is a reference to a visualization model with an associated physics item.  A physics item may have an associated visualization model instance.  

`ChVisualSystem` defines a base class for possible concrete run-time visualization systems and imposes minimal common functionality. A visual system is attached to a ChSystem using `ChSystem::SetVisualSystem`. The Chrono physics system will then trigger automatic updates to the visualization system as needed, depending on the particular type of analysis being conducted.

**Defining visualization models**

The new nechanism for defining visualization shapes and materials for a Chrono physics item is illustrated in the following typical sequence:
```cpp
    // Create a visual material and set properties
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetDiffuseColor(ChColor(0.9f, 0.4f, 0.2f));
    // set other material properties
    
    // Create a visual shape and add one or more materials
    auto vis_sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.5;
    sphere->AddMaterial(vis_mat);
    
    // Create a visual model and add one or more shapes
    auto vis_model = chrono_types::make_shared<ChVisualModel>();
    vis_model->AddShape(vis_sphere);
    // add more visual shapes to the model

    // Attach an instance of a visual model to a physics item
    body->AddVisualModel(vis_model);
```

Note that FEA visualization requires defining objects of type `ChVisualShapeFEA`.  An FEA visualization shape is in fact a pair of two visual shapes, one for visualizing the FEA mesh, the other for visualizing so-called glyphs (e.g., representation of the FEA nodes). 

For convenience, several shortcuts are provided:
- the diffuse color or diffuse map texture can be set directly on a visual shape, using `ChVisualShape::SetColor` and `ChVisualShape::SetTexture`.  If the shape has visual materials defined, these functions affect the 1st material in the list. Otherwise, a new material is created and associated with the given shape, and its properties set appropriately.
- a visual shape instance can be added directly to a physics item, using `ChPhysicsItem::AddVisualShape`. If the physics item already has an associated visual model instance, the new shape is added to that model at the specified transform.  Otherwise, a new visual model instance is created and associated with the physics item, and the given shape instance created within the model.
- an individual visual shape in the visual model of a physics item can be accessed through its index with `ChPhysicsItem::GetVisualShape`. This can then be used to change shape parameters or parameters of the associated visual material.

**Defining a visualization system**

While specification of visualization assets (materials, shapes, and models) must now be done as described above for any Chrono run-time visualization system, the Chrono API does not impose how a particular rendering engine should interpret, parse, and render the visual representation of a Chrono system.  
The suggested mechanism is to define a concrete visualization system (derived from `ChVisualSystem`) and attach it to the Chrono system. Currently, only an Irrlicht-based visualization system is provided through `ChVisualSystemIrrlicht`. This object replaces the now obsolete ChIrrApp. 

A typical sequence for creating and attaching an Irrlicht-based visualization system to a Chrono simulation is illustrated below:
```cpp
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Chrono::Irrlicht visualization");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, 2, 3));
    vis->AddTypicalLights();
```

Notes:
- Various parameters (such as windows size, windows title, camera vertical direction) can be set with various `Set***` methods.  These must be called before the visual system is initialized with ChVisualSystemIrrlicht::Initialize().
- After the call to `Initialize`, additional rendering elements (such as lights, a sky box, a camera) can be added to the visualizatioon system with various `Add***` methods.
- Once the visual system is initialized **and** attached to the Chrono system, all currently defined visual models are processed and Irrlicht nodes created.  
- If visual models are created at a later time, these must be converted to Irrlicht nodes through a call to `ChVisualSystemIrrlicht::BindAll()` (to process all visual models in the Chrono system) or to `ChVisualSystemIrrlicht::BindItem` (to process the visual model for the specified Chrono physics item).

Additional rendering options can be enabled with calls to `Enable***` methods which must be made only after the visualization system was initialized and attached to a Chrono system.  These options include enabling various ways of rendering collision and contact information and body or link reference frames.

A typical simulation loop with Irrlicht-based run-time visualization has the form:
```cpp
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        // make additional Irrlicht-based rendering calls, for example to display a grid:
        irrlicht::tools::drawGrid(vis->GetVideoDriver(), 0.5, 0.5, 12, 12,
                                  ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(CH_C_PI_2)),
                                  irr::video::SColor(50, 80, 110, 110), true);
        vis->EndScene();
        sys.DoStepDynamics(step_size);
    }
```


The Irrlicht visualization system also provides a GUI (displayed using the `i` key during simulation) which allows changing rendering options at run-time.

See the various Chrono demos (in `src/demos/irrlicht/`) for different ways of using the new visualization system mechanism in Chrono.  Note that the Chrono::Vehicle Irrlicht-based visualization systems (`ChWheeledVehicleVisualSystemIrrlicht` and `ChTrackedVehicleVisualSystemIrrlicht`) use a correspondingly similar mechanism for their definition and usage in a vehicle  simulation loop. See demos under `src/demos/vehicle/`. 

### [Changed] Vehicle inertia properties

The underlying mechanism for setting and querying inertia properties (mass, COM location, and inertia matrix) for vehicle systems and subsystems was redesign for consistency.  At the user API level, this change is reflected through a uniform manner to hoe these quantities are reported.

Any vehicle subsystem (of type `ChPart`), as well as any vehicle system (`ChWheeledVehicle` or `ChTrackedVehicle`) provide the following set of accessor methods:
- `GetMass()` returns the mass of the (sub)system.  In the case of a vehicle, this includes the mass of all vehicle subsystems.  Furthermore, the mass of a wheeled vehicle includes the mass of the tires.
- `GetCOMFrame()` returns the current COM (centroidal) frame.  This frame is relative to and expressed in the reference frame of the part or of the vehicle.
- `GetInertia()` returns the current inertia matrix (that is the articulated inertia).  The reported inertia matrix is given with respect to the centroidal frame of the part or vehicle.

In addition, a `ChPart` or `ChVehicle` also provide a method `GetTransform()` which returns the vehicle transform (translation and orientation encapsulated in a `ChFrame`) relative to the global (absolute) frame. Recall that, by convention, the vehicle reference frame is that of its main chassis.

### [Changed] CMake project configuration script

The CMake script `ChronoConfig.cmake`, generated automatically during Chrono CMake configuration and used in configuring third-party applications that depend on Chrono (via calls to `find_project(Chrono ...)`) was modified to produce the compiler and linker flags in CMake list variables (as opposed to space-separated strings as before).  The variables affected by this change are `CHRONO_CXX_FLAGS`, `CHRONO_C_FLAGS`, and `CHRONO_LINKER_FLAGS`.

This allows use of modern CMake in the configuration scripts for such an external project.  See the example in the `template_project/` directory in the Chrono distribution:
```
target_compile_definitions(myexe PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(myexe PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(myexe PUBLIC ${CHRONO_LINKER_FLAGS})
```

### [Changed] Right-handed frames in Chrono::Irrlicht

The Irrlicht library, wrapped in the Chrono::Irrlicht run-time visualization library uses the DirectX convention of left-handed frames.  This has been a long standing source of confusion for all Chrono users since Chrono simulations (always conducted using right-handed frames) were "mirrored" during rendering.

This set of changes forces Chrono::Irrlicht to use right-hand projection matrices resulting in renderings that are consistent with the underlying models and simulations.  All changes are internal and transparent to the user.  

We took this opportunity to make a small set of minor API changes, most of them simple function renames:
- ChIrrApp::AddTypicalLogo() was renamed to ChIrrApp::AddLogo().
- ChIrrApp::AddTypicalCamera() was renamed to ChIrrApp::AddCamera().
- ChIrrApp::AddTypicalSky() was renamed to ChIrrApp::AddSkyBox().
- ChIrrApp::AddTypicalLights() was changed to always construct two point lights with default settings (postions, radii, and colors).  The positions of these lights are different for a Y or Z camera vertical direction.  A user interested in changing the settings of the default lights should use the function ChIrrApp::AddLight() which allows specifying position, radius, and color.
- ChVehicleIrrApp::SetSkyBox() was obsoleted (the Chrono sky box is automatically added).

### [Added] Modal analysis module

A new module `MODULE_MODAL` has been added. The module uses an external dependency (the [Spectra](https://spectralib.org/) library for eigenvalue computation). 
Follow the [installation guide](@ref module_modal_installation) for instructions on how to enable it.

The new class `ChModalAssembly` offer three main functionalities:

- **undamped modal analysis** of all the system being created within the sub assembly will be obtained. The modes and frequencies can be also displayed interactively if using the Irrlicht visualization system. 
	- The subassembly can also contain constraints between its sub parts. 
	- Rigid modes (for free-free structures) are supported
	- A custom genaralized, sparse, constrained eigenvalue solver of Krylov-Schur type allows the computation of only the n lower modes. This allows handling large FEA systems. 
	
- **damped (complex) modal analysis** of the subsystem: this is like the previous case, but damping matrix is used too, hence obtaining complex eigenvalues/eigenvectors. Damping factors for the modes are output too, indicating stability or instability. *NOTE: while we wait that Spectra will enable complex eigenvalues in Krylov-Schur, a more conventional solver is used, that is not sparse - hence requiring more time and memory*

- **modal reduction** of the subassembly. Example of a scenario where this is useful: you have a tower modeled with thousands of finite elements, but you are just interested in the small oscillations of its tip, because you will mount a windmill on its tip. If you simulate thousands of finite elements just for this purpose, you waste CPU time, hence a modal reduction of the tower will discard all the DOFs of the finite elements and represent the overall behaviour of the tower using just few modal shapes (ex. fore aft bending, lateral bending, etc.), with extreme CPU performance at the cost of a small reduction of fidelity.
	- Bodies and FEA nodes can be added to the subassebly as *internal*  or *boundary* interface nodes. Later one can call `ChModalAssembly::SwitchModalReductionON(int n_modes)` to replace the complexity of the internal nodes with few `n_modes` modal coordinates.
	- Boundary interface nodes can be connected to the rest of the multibody system as usual, using constraints, forces, etc.
	- Internal constraints can be used between internal nodes. Their effect too will be condensed in the modal reduction.
	- *NOTE: at the moment only linear dynamics is supported for the subassembly, in the sense that the subassembly cannot withstand large rotations, ex. in a helicopter blade. Future developments will address this*


### [Added] Callback mechanism for collision debug visualization

A formal callback mechanism was added to `ChCollisionSystem` which allows user-controlled visualization of collision detection information for debug purposes.
This mechanism allows overlaying collision detection debug information (wireframe rendering of the collision shapes, axis-aligned bounding boxes, contact points and normals) using any visualization system.
The only requirement for this capability is the ability of rendering lines between two given 3D points (expressed in the absolute coordinate system).

To use this capability, users must implement a custom callback class derived from `ChCollisionSystem::VisualizationCallback` and override the `DrawLine` method to render a line in 3D using their visualization system of choice.
This callback object is attached to the Chrono system using `ChCollisionSystem::RegisterVisualizationCallback` and rendering of collision information is triggered by calling `ChCollisionSystem::Visualize` from within the simulation loop.
The type of information that will be rendered is controlled by an integer flag argument to `Visualize` which can be any of the enum `ChCollisionSystem::VisualizationModes` or a combination of these (using bit-wise or).

For example:
```cpp
class MyDrawer : public ChCollisionSystem::VisualizationCallback {
public:
  MyDrawer() {...}
  virtual void DrawLine(...) override {...}
};

ChSystemNSC sys;
...
auto drawer = chrono_types::make_shared<MyDrawer>();
sys.GetCollisionSystem()->RegisterVisualizationCallback(drawer);
...
while (...) {
  ...
  sys.GetCollisionSystem()->Visualize(ChCollisionSystem::VIS_Shapes | ChCollisionSystem::VIS_Aabb);
}
```

A demonstration of this capability, with either the Bullet-based or the parallel Chrono collision system, is given in `demo_IRR_visualize_collision`. The custom collision visualization callback class in this demo uses Irrlicht for rendering lines.



### [Changed] Translational and rotational spring-damper-actuators

- The classes `ChLinkSpring` and `ChLinkSpringCB` were obsoleted, with their functionality superseded by `ChLinkTSDA`.  
- For consistency, the class `ChLinkRotSpringCB` was renamed to `ChLinkRSDA`.

Both `ChLinkTSDA` and `ChLinkRSDA` default to a linear spring-damper model, but an arbitrary user-defined spring-damper-actuation force can be implemented through functor classes (`ChLinkTSDA::ForceFunctor` and `ChLinkRSDA::TorqueFunctor`, respectively).  When using the PyChrono python wrappers, these functor classes are named `ForceFunctor` and `TorqueFunctor`. When using the C# wrappers, these functor classes are inherited as outside classes named `TSDAForceFunctor` and `RSDATorqueFunctor`, respectively.

**ChLinkRSDA**

- `ChLinkRSDA` is now derived directly from `ChLink` and properly accounts for possible full revolutions. 
- A rotational spring is initialized by specifying the two connected bodies and the RSDA frames on each of them.  It is assumed that the mechanism kinematics are such that the two RSDA frames maintain their Z axes parallel at all times.
- The angle is measured starting from the X axis of the RSDA frame on the first body towards the X axis of the RSDA frame on the second body and its sign is dictated by the right-hand rule.
- Unless `SetRestAngle` is explicitly called, the spring rest (free) angle is inferred from the initial configuration.
- The signature of the virtual method `ChLinkRSDA::TorqueFunctor::evaluate` was changed to take a const reference to the RSDA element as its last argument.
- A new visual asset (`ChRotSpringShape`) was added for run-time visualization of a rotational spring.

**ChLinkTSDA**

- For consistency, the mechanism for specifying the spring rest (free) length was changed: unless `SetRestLength` is explicitly called, the spring rest (free) angle is inferred from the initial configuration.
- The signature of the virtual method `ChLinkTSDA::ForceFunctor::evaluate` was changed to take a const reference to the TSDA element as its last argument.

### [Changed] Refactor Chrono::Vehicle suspension test rigs

The wheeled vehicle suspension test rig (STR) was modified to accept an arbitrary number of tested axles from any given vehicle.

The new STR will create posts / pushrods for all spindles (left and right) from all axles specified as "test axles".
Like before, one can construct an STR from a given vehicle (from one of the models in the Chrono vehicle models library or else created from a JSON specification file) or else from a JSON specification file for an STR.  However, the latter approach will now construct the entire vehicle (specified though a vehicle JSON file) but include only a user-specified subset of its axles for testing.
Note that this is not a limitation because Chrono::Vehicle was also modified to allow specification in a JSON file of a stripped-down vehicle model which need not include a driveline nor a steering mechanism and may even define a single axle.

Additional vehicle subsystems (such as steering mechanisms or subchassis components) can be adding to either type of STR (`ChSuspensionTestRigPlatform` or `ChSuspensionTestRigPushrod`) using the functions `IncludeSteeringMechanism` and `IncludeSubchassis`. This simply means that: (i) run-time visualization of the additional subsystem can be enabled and (ii) the additional subsystem is included in the rig output (if that is enabled).
The associated vehicle is initialized with its chassis fixed and its driveline automatically disconnected. Simulation of the test rig (through the function `ChSuspensionTestRig::Advance`) performs a simulation of the entire vehicle with all its components, but vehicle subsystems not explicitly included in testing are invisible and do not participate in any output.

See `demo_VEH_SuspensionTestRig` for various examples and options, and look at the JSON files used in that demo for changes in their formats.

Note also that the format for a data file with STR actuation information (used by a ChDataDriverSTR) was modified by moving the steering input in the 2nd column.
In other words, each line of this ASCII file should now contain:<br>
`    time  steering_input  left_post_0  right_post_0 left_post_1 right_post_1 …`

### Release 7.0.3 - 2022-04-17

### [Fixed]

- SIMD detection is combined into one cmake script
- Fixed SIMD feature detection with Clang, allowing support for Apple-M1 and generic AArch64 CPUs

### Release 7.0.2 - 2022-04-03

### [Fixed]

- Fixed bug in ANCF shells 3443 and 3883 where the incorrect Gauss quadrature weights and Jacobian elements were used when multiple layers of different sizes are defined
- Fixed bug where the active flag for a sub-block of DOFs for a ChBody was incorrectly set
- Updates to the continuous integration scripts

## Release 7.0.1 - 2022-01-07

### [Fixed]

- Fixed Chrono::Sensor class export (Windows)
- Fixed bug in ChPovRay related to processing of OBJ files
- Fixed demo program in sample project for vehicle co-simulation
- Fixed setting of MPI linker flags in CMake project configuration script

## Release 7.0.0 - 2021-11-15

### [Added] DDS communicator in Chrono::Synchrono module

`Chrono::SynChrono` used to rely only on MPI to pass message between ranks. We added a different `SynCommunicator` derived class called `SynDDSCommunicator`. This communicator relies on e-Prosima implementation of DDS, called [fastDDS](https://www.eprosima.com/index.php/products-all/eprosima-fast-dds) and it is alternative to MPI communication. Please note that while DDS implementations are interoperable, they are not compatible at the implementation level, therefore to use this functionality please download or clone and build fastDDS and follow the [instructions on our website](https://github.com/projectchrono/chrono/tree/develop/src/chrono_synchrono). The main purpose of SynChrono-DDS is to perform distributed simulation across different machines, hence overcoming MPI limitations: as long as two machines can establish a UDP/TCP communication they can participate in a distributed SynChrono-DDS communication. 

- From the user API perspective, the change is minimal: for example, in `demo_SYN_DDS_wheeled.cpp` the only change is the communicator itself, after including the proper header:
   ```cpp
   #include "chrono_synchrono/communication/dds/SynDDSCommunicator.h"
   .....
   auto communicator = chrono_types::make_shared<SynDDSCommunicator>(node_id);
   ```
- Launching the jobs: instead of using mpiexec/mpirun, DDS ranks are started separately (either manually or through a job scheduler). DDS implements a _Barrier_ such that jobs freeze until the expected number of participant is found. This means that jobs can be launched even minutes apart and they will simply wait for each other before stepping forward together.
- UDP communication: the most useful application of SynChronoDDS is using it across UDP, to perform distributed simulation without MPI boundaries. It is sufficient to provide the IP address of the machines to connect with to set up communication (given that the Firewall is not preventing it) as in `demo_SYN_DDS_distributed.cpp`: 
 ```cpp
qos.transport().user_transports.push_back(std::make_shared<UDPv4TransportDescriptor>());
qos.transport().use_builtin_transports = false;
qos.wire_protocol().builtin.avoid_builtin_multicast = false;
// Set the initialPeersList
for (const auto& ip : ip_list) {
    Locator_t locator;
    locator.kind = LOCATOR_KIND_UDPv4;
    IPLocator::setIPv4(locator, ip);
    qos.wire_protocol().builtin.initialPeersList.push_back(locator);
}
auto communicator = chrono_types::make_shared<SynDDSCommunicator>(qos);
```

### [Added] New terramechanics co-simulation module

This new module provides support for co-simulating various Chrono models of ground wheeled vehicles.  This framework implements an explicit co-simulation model (of force-displacement type) and uses an MPI layer for exchanging data between the participant nodes.

The co-simulation framework was architected to support:
- any of the terramechanics simulation capabilities in Chrono (rigid terrain; deformable SCM; granular terrain with Chrono::Multicore, Chrono::Gpu, or Chrono::Distributed; continuous granular terrain representation with Chrono::Fsi);
- external, third-party terramechanics simulation packages (regardless of implementation and/or parallel programing paradigm);
- terramechanics packages that do not advance themselves the dynamics of the tires (if any) or else treat both tire and terrain simulation;
- rigid or flexible tires (the latter assumed to rely on the Chrono FEA simulation capabilities);
- a variety of Chrono wheeled vehicle models (including any Chrono::Vehicle wheeled model, as well as single-wheel test rigs, or wheeled rover models such as the Curiosity and Viper).

The new module is build automatically if MPI is found and available and if the Chrono::Vehicle module is enabled (at CMake configuration time).  Support for different terramechanics models is enabled within the module if the corresponding Chrono module is enabled (note that each of the following Chrono modules has its own additional dependencies):
- Chrono::Multicore for granular multi-core simulation (OpenMP-based)
- Chrono::Gpu for granular GPU simulation (CUDA-based)
- Chrono::Distributed for granular distributed simulation (MPI-based)
- Chrono::FSI for continuous granular terrain representation (CUDA-based)

Terrain simulation is conducted on one or more `Terrain` nodes (MPI ranks).  If the terramechanics simulation is itself using MPI, support is provided to generate and provide an MPI sub-communicator consisting of all `Terrain` ranks; only the main `Terrain` rank (rank 0 in the sub-communicator) participates in the co-simulation communication.

Several types of `MBS` nodes are provided, representing different wheeled mechanisms and vehicles:
- `ChVehicleCosimRigNode` wraps a model of a single-wheel test rig;
- `ChVehicleCosimVehicleNode` wraps a Chrono::Vehicle ground wheeled vehicle model (with arbitrary number of wheels);
- `ChVehicleCosimCuriosityNode` wraps the Curiosity Mars rover available in the Chrono robot models library;
- `ChVehicleCosimViperNode` wraps the Viper lunar rover available in the Chrono robot models library;

Three different types of `Tire` nodes are provided to intermediate simulation and data-exchanged between the MBS node and the main Terrain node:
- `ChVehicleCosimTireNodeRigid` is a simple conduit between the MBS node and the Terrain node; it does not perform any dynamics of its own, but does maintain a physical representation of the associated tire for simulation and visualization output purposes;
- `ChVehicleCosimTireNodeFlexible` wraps a deformable tire modeled with Chrono::FEA elements; this node performs its own dynamics (accelerated by the use of OpenMP parallel loops in Chrono::FEA);
- `ChVehicleCosimTireNodeBypass` provides a pure short-circuit between the MBS and Terrain nodes and is intended for coupling to terramechanics external packages which simulate simultaneously the tire, the terrain, and the tire-terrain interaction.

The architecture of the framework thus implements a "three way" co-simulation setup.  The data exchange between the three different node types is as follows:
- the MBS node sends spindle body states to the appropriate Tire nodes;  a Tire node sends terrain forces and moments acting on the spindle body to the MBS node;
- a Tire node sends tire body state (for a rigid tire) or tire FEA mesh state (for a flexible tire) to the Terrain node; the (main) Terrain node send terrain force on the spindle (rigid tire) body or nodal terrain forces (for a flexible tire) to the appropriate Tire node.

The co-simulation framework also provides the ability to attach a drawbar pull rig mechanism to any of the supported MBS nodes. Two variants are provided:
- `ChVehicleCosimDBPRigImposedSlip` allows imposing known (fixed) vehicle forward linear velocity and wheel angular velocity to maintain a prescribed value of the longitudinal slip. The actuation specifies if the linear velocity or angular velocity is considered as "base velocity", with the other one derived from the slip value. The DBP force is extracted as the reaction force required to enforce the vehicle forward linear velocity (at steady state).  Each run of this experiment produces one point on the slip-DBP curve.
- `ChVehicleCosimDBPRigImposedAngVel` enforces a prescribed wheel angular velocity. A linearly increasing resistive force is applied against the forward motion of the vehicle and the experiment is ended when the vehicle stops. At each time, the vehicle forward speed and resulting slip are calculated and stored together with the current resistive force (DBP). This experiment produces the entire slip-DBP curve at once.

Output feature include:
- Run-time visualization.  Available if the Chrono::OpenGL module is enabled, this option permits run-time visualization from the Terrain node.  The only exception is the SCM deformable terrain node which uses Chrono::Irrlicht (if available)
- Simulation output.  A variety of output files are created from each type of co-simulation node, as well as from the drawbar-pull rig (if one is present).  These include both files with information from all time steps and individual frame files created at each output step.
- Off-line visualization.  If desired, the user can invoke functions to generate post-processing visualization output.  The visualization output files (where possible) are meant to be used with the new Blender-based python scripts available in Chrono and allow rendering in a single scene the visualization assets from all co-simulation nodes.

The design of the co-simulation framework is such that all inter-node co-simulation communication is transparent to the user.  User code need only instantiate the appropriate number of co-simulation nodes of the appropriate type (MBS, Tire, or Terrain), select simulation options, and make calls to advance the state of the coupled system from step to step.  At a minimum, the main user simulation loop must call `Synchronize` followed by `Advance` for all co-simulation nodes; optional calls may be made to functions controlling simulation and off-line visualization output.  A set of demo programs (named `demo_VEH_Cosim***`) are provided to illustrate the use of the co-simulation framework with different multibody systems and terrain models.


### [Changed] Chrono::Fsi API redesign

For consistency with the main Chrono module and other optional Chrono modules, the Chrono::FSI API was changed as follows: 

- The user's interaction with the Chrono::FSI module was streamlined by exposing in the public API a single Chrono::FSI system object (of type `ChSystemFsi` ) and hiding the underlying implementation in a private class. 
- User code only needs to include one Chrono::Fsi header in their project, namely `chrono_fsi/ChSystemFsi.h` and need not include any of the utility header files from `utils/`.
- Users can use standard C++ types to declare a scalar, and use Chrono types (`ChVector`, `ChQuaternion`, etc) to declare vectors, quaternions, etc. 
- The initialization of the parameters from a JSON file was changed from fsi::utils::ParseJSON() to `myFsiSystem.SetSimParameter()`, assuming the user has created an FSI system `myFsiSystem`. 
- A new function was added to set periodic boundary condition: `ChSystemFsi::SetBoundaries()`. 
- The function used to finalize the subdomains was changed from fsi::utils::FinalizeDomain() to `ChSystemFsi::SetSubDomain()`.
- The function used to set the output directory was changed from utils::PrepareOutputDir() to `ChSystemFsi::SetFsiOutputDir()`.
- The function used to add SPH particles was changed from myFsiSystem.GetDataManager()->AddSphMarker() to `ChSystem::AddSphMarker()`. 
- The functions used to add BCE particles were changed along the same lines; for instance, to add BCE particles for a cylinder, use `ChSystemFsi::AddBceCylinder()`. 
- The function used to output data was changed from fsi::utils::PrintToFile() to `ChSystemFsi::PrintParticleToFile()`. 

See the updated FSI demo programs for usage of the new Chrono::Fsi API.

**Added - Option to build Chrono::FSI in single precision**

- Users can optionally configure Chrono::FSI in single precision by unsetting the CMake variable `USE_FSI_DOUBLE`
- By default, Chrono::FSI is configured and built in double precision
- Users should be careful opting for single precision as this can adversely impact simulation results


### [Changed] Sensor to improve performance and added features 

**Changed - Optix 7.2 as Dependency:**
 - Upgraded to Optix 7.2 from 6.5. 7.2 (exactly) is the only version supported.

**Changed - Refactored sensor code:**
 - sensors have been moved to `src/chrono_sensor/sensors/` to cleanup directory structure
 - all optix-dependent code was moved to `src/chrono_sensor/optix` to consolidate the dependency

**Changed - IMU to accelerometer and gyroscope:**
 - Split the IMU sensor into its components (ChAccelerometerSensor and ChGyroscopeSensor) to facilitate additional sensors. Using both sensors together with same update rate will produce the same behavior as the original IMU. These sensors are still maintained under `ChIMUSensor.h and ChIMUSensor.cpp`
  ```cpp
  ChAccelerometerSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose, std::shared_ptr<ChNoiseModel> noise_model);

  ChGyroscopeSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose, std::shared_ptr<ChNoiseModel> noise_model);
  ```
**Added - Magnetometer:**
 - Added magnetometer sensor alongside accelerometer and gyroscope. Sensor is maintained in `ChIMUSensor.h` and `ChIMUSensor.cpp`. 
 - Returns a magentic field strength vector based on the orientation of the sensor, and the GPS location of the sensor and simulation.
 - Can be permuted with noise using same noise models available for accelerometer and gyroscope.
  ```cpp
  ChMagnetometerSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose, std::shared_ptr<ChNoiseModel> noise_model, ChVector<double> gps_reference);
  ```

**Removed - Keyframe user configuration:**
 - Removed the need for users to set the number of keyframes used for motion blur. Will now automatically find these internally.

**Changed - Scene API:**
 - Point lights can be created then added to the scene rather than adding directly though a function call
 - Point lights must be modified based on index rather than reference: `void ChScene::ModifyPointLight(unsigned int id, PointLight p)`
 - Background is created by the user and passed to the scene via the sensor manager `void ChScene::SetBackground(Background b)`
 - Ambient light is now configurable by the user through the scene `void ChScene::SetAmbientLight(ChVector<float> color)`
 - The ray tracing epsilon used to prevent self-intersections is configurable in the scene to allow the user to adjust the parameter when artifacts are present. `void ChScene::SetSceneEpsilon(float e)`
 - 

**Added - Gradient background:**
 - Can add gradient colors for sky alongside solid color or sky map.
  ``` cpp
  enum class BackgroundMode {
      SOLID_COLOR,     ///< single solid color defined by RGB
      GRADIENT,        ///< color gradient used for upper hemisphere
      ENVIRONMENT_MAP  ///< image used for spherical sky map
  };
  ```

**Changed - ChOptixEngine to hide optix-dependent code:**  
 - ChOptixEngine no longer supports returning the optix context to the user.

**Changed - Automatic mesh and object instancing:**
 - Objects that use the same mesh will automatically instance the mesh (instanced if using same chrono::geometry::ChTriangleMeshConnected)
 - Removed the ability to add instanced objects explicitely.
 - Recommended instancing is to create single ChTriangleMeshConnected, then adding that with many scales (using ChTriangleMeshShape) and positions (using ChBody).

**Changed - Shaders for visualization:**
 - Improved the material shaders to support physically-based materials and phong materials in the same scene. 
 - Shading calls do NOT change API, but WILL be visible on objects.
 - Expanded parameters contained in `chrono::ChVisualMaterial` to include metalic, roughness, and other textures as well as whether to use a specular or metalic workflow. Will be detected for meshes loaded from file.

**Added - Global Illumination with Optix Denoiser:**
Added option for cameras to use global illumination with a denoiser to reduce stochastic noise imparted by the ray tracing algorithm. 
 - enable global illumination and gamma correction exponent in camera constructor:
 ``` cpp
  ChCameraSensor(std::shared_ptr<chrono::ChBody> parent, // object to which the sensor is attached
                float updateRate,                        // rate at which the sensor updates
                chrono::ChFrame<double> offsetPose,      // position of sensor relative to parent
                unsigned int w,                          // image width
                unsigned int h,                          // image height
                float hFOV,                              // horizontal field of view
                unsigned int supersample_factor = 1,     // supersample diameter
                CameraLensModelType lens_model = CameraLensModelType::PINHOLE, //lens model
                bool use_gi = false,  // global illumination enable/disable
                float gamma = 2.2);   // gamma correction exponent
 ```
- denoiser will automatically be used internally

**Changed - Lidar sensor beam divergence**
 - beam divergence can be configured by beam shape (rectangular or elliptical)
  ```cpp
  enum class LidarBeamShape {
    RECTANGULAR,  ///< rectangular beam (inclusive of square beam)
    ELLIPTICAL    ///< elliptical beam (inclusive of circular beam)
  };  
  ```
 - vertical and horizontal divergence angles independently parameterized
 - Dual return mode added
  ``` cpp
  enum class LidarReturnMode {
    STRONGEST_RETURN,  ///< range at peak intensity
    MEAN_RETURN,       ///< average beam range
    FIRST_RETURN,      ///< shortest beam range
    LAST_RETURN,       ///< longest beam range
    DUAL_RETURN        ///< first and strongest returns
  };
  ```

```cpp
ChLidarSensor(std::shared_ptr<chrono::ChBody> parent,
              float updateRate,
              chrono::ChFrame<double> offsetPose,
              unsigned int w,
              unsigned int h,
              float hfov,
              float max_vertical_angle,
              float min_vertical_angle,
              float max_distance,
              LidarBeamShape beam_shape = LidarBeamShape::RECTANGULAR,
              unsigned int sample_radius = 1,
              float vert_divergence_angle = .003f,
              float hori_divergence_angle = .003f,
              LidarReturnMode return_mode = LidarReturnMode::MEAN_RETURN,
              float clip_near = 1e-3f);
```

**Added - Radar sensor:**
- A radar sensor was added, with the initial version similar to lidar. Will return set of points that include range, azimuth, elevation, doppler velocity, ampliture of detection, and object id used for clustering
- radar is configurable based on update rate, position, vertical and horizontal resolutions, vertical and horizontal field of view, and maximum distance.
``` cpp
ChRadarSensor(std::shared_ptr<chrono::ChBody> parent,
              const float updateRate,
              chrono::ChFrame<double> offsetPose,
              const unsigned int w,
              const unsigned int h,
              const float hfov,
              const float vfov,
              const float max_distance,
              const float clip_near = 1e-3f);
```

**Added - Segmentation camera:**
- Added a segmentation camera `ChSegmentationCamera` which returns an image with class ID and instance ID for each pixel in the image.
- If paired with an RGB camera at same frequency, position, fiew of view, and resolution, can be used to generate automatically segmented images
- Instance ID and class ID are set in the material, defaulting to 0. See `demo_SEN_camera` and `chrono::ChVisualMaterial` for details on configuration.

```cpp
ChSegmentationCamera(std::shared_ptr<chrono::ChBody> parent,  // object to which the sensor is attached
                      float updateRate,                       // rate at which the sensor updates
                      chrono::ChFrame<double> offsetPose,     // position of sensor relative to parent
                      unsigned int w,                         // image width
                      unsigned int h,                         // image height
                      float hFOV,                             // horizontal field of view
                      CameraLensModelType lens_model = CameraLensModelType::PINHOLE);  // lens model type
```

### [Changed] ANCF element improvements and additions

**Changed - Element Naming Convention:** 

The following ANCF elements have been renamed according to the 4-digit ANCF naming convention from: *Dmitrochenko, O., Mikkola, A.: Digital nomenclature code for topology and kinematics of finite elements based on the absolute nodal co-ordinate formulation. Proc. Inst. Mech. Eng., Part K: J. Multi-Body Dyn. 225(1), 34–51 (2011)*
- `ChElementBeamANCF` renamed to `ChElementBeamANCF_3333`
- `ChElementShellANCF` renamed to `ChElementShellANCF_3423`
- `ChElementShellANCF_8` renamed to `ChElementShellANCF_3833`
- `ChElementBrick` renamed to `ChElementHexaANCF_3813`
- `ChElementBrick_9` renamed to `ChElementHexaANCF_3813_9`

The following elements were renamed to improve naming consistency:
- `ChElementHexa_8` renamed to `ChElementHexaCorot_8`
- `ChElementHexa_20` renamed to `ChElementHexaCorot_20`
- `ChElementTetra_4` renamed to `ChElementTetraCorot_4`
- `ChElementTetra_10` renamed to `ChElementTetraCorot_10`

**Added - New ANCF Elements:**

- `ChElementBeamANCF_3243` a fully parameterized 2-Node ANCF beam element
  - 24 DOF per element
  - Uses the enhanced continuum mechanics method to reduce locking (the same method as `ChElementBeamANCF_3333`)
  - Only rectangular cross sections are currently supported 
  - Only linear viscoelastic materials are supported at this time (single coefficient damping model)
- `ChElementShellANCF_3443` a fully parameterized 4-Node ANCF shell element
  - 48 DOF per element
  - Prone to locking, no modifications to reduce locking are included at this time
  - Supports multiple discrete layers in a single shell element like `ChElementShellANCF_3833`
  - Only linear viscoelastic materials are supported at this time (single coefficient damping model)
- `ChElementHexaANCF_3843` a fully parameterized 4-Node ANCF brick element
  - 96 DOF per element
  - No modifications to reduce locking are included at this time
  - Only linear viscoelastic materials are supported at this time (single coefficient damping model)


**Changed - Internal Force Calculation Method:** 

Applies to: `ChElementBeamANCF_3243`, `ChElementBeamANCF_3333`, `ChElementShellANCF_3443`, `ChElementShellANCF_3833`, and `ChElementHexaANCF_3843`

For these 5 elements, there is an option for two different generalized internal force calculation methods:
- `IntFrcMethod::ContInt` (**Default**): The "Continuous Integration" method efficiently integrates across the volume of the element every time the generalized internal force or its Jacobian is calculated.  This method is dependent on the number of Gauss quadrature points used for the integration, resulting in increased generalized internal force and Jacobian calculation times as additional layers are added to the shell elements.  Even so, this method will typically be faster, and it has a significantly lower memory storage overhead.  This method is a modification and extension to the method found in: *Gerstmayr, J., Shabana, A.A.: Efficient integration of the elastic forces and thin three-dimensional beam elements in the absolute nodal coordinate formulation. In: Proceedings of the Multibody Dynamics Eccomas thematic Conference, Madrid(2005).*
- `IntFrcMethod::PreInt`: The "Pre-Integration" method is designed so that integration across the volume of the element occurs only once prior to the start of the simulation.  This method is independent on the number of Gauss quadrature points used for the integration, resulting in no change in in-simulation generalized internal force and Jacobian calculation times as additional layers are added to the shell elements.  This method is generally slower and has a significantly higher memory storage overhead, especially as the degree of freedom count for the element increases.  This method is a modification and extension to the method found in: *Liu, Cheng, Qiang Tian, and Haiyan Hu. "Dynamics of a large scale rigid–flexible multibody system composed of composite laminated plates." Multibody System Dynamics 26, no. 3 (2011): 283-305.*

If possible, the calculation method should be set prior to the start of the simulation so that the precalculation phase is not called for both calculation methods resulting in unnecessary calculation and memory overhead.

A report covering the detailed mathematics and implementation both of these generalized internal force calculations and their Jacobians can be found in: *Taylor, M., Serban, R., and Negrut, D.: Technical Report TR-2020-09 Efficient CPU Based Calculations of the Generalized Internal Forces and Jacobian of the Generalized Internal Forces for ANCF Continuum Mechanics Elements with Linear Viscoelastic Materials, Simulation Based Engineering Lab, University of Wisconsin-Madison; 2021.*

These calculation methods make heavy use of the Eigen3 library.  For MSVC 2017 and to a lesser extent MSVC 2019, this can result in **significantly** longer compile times.  This is a known issue with Eigen3 and MSVC: https://gitlab.com/libeigen/eigen/-/issues/1725.


**Changed - Obtaining Stress and Strain:** 

Applies to: `ChElementBeamANCF_3243`, `ChElementBeamANCF_3333`, `ChElementShellANCF_3443`, `ChElementShellANCF_3833`, and `ChElementHexaANCF_3843`

For all 5 of these elements, the full 3x3 Green-Lagrange Strain Tensor can be obtained using the function below using normalized element coordinates with values between -1 and 1:
  ```cpp
  ChMatrix33<> GetGreenLagrangeStrain(double xi, double eta, double zeta)
  ```
  
The full 3x3 2nd Piola-Kirchhoff Stress Tensor can be obtained for the beam and brick elements using the function:
  ```cpp
  ChMatrix33<> GetPK2Stress(double xi, double eta, double zeta)
  ```
Since the shell elements support multiple discrete layers which can result in stress discontinuities, information about the layer of interest must be provided when obtaining the full 3x3 Green-Lagrange Strain Tensor.  For the shell elements, the layer index (0-index starting with the first layer defined for the element) is required as well as the normalized position through the thickness of the layer whose value is between -1 and 1.
  ```cpp
  ChMatrix33<> GetPK2Stress(double layer, double xi, double eta, double layer_zeta)
  ```
  
The Von Misses Stress can be obtained for the beam and brick elements using the function: 
  ```cpp
  double GetVonMissesStress(double xi, double eta, double zeta)
  ```
For the shell elements, the Von Misses Stress can be obtained using the function:   
  ```cpp
  double GetVonMissesStress(double layer, double xi, double eta, double layer_zeta)
  ```

**Changed - Application of Gravity:** 

Applies to: `ChElementCableANCF`, `ChElementBeamANCF_3243`, `ChElementBeamANCF_3333`, `ChElementShellANCF_3423`, `ChElementShellANCF_3443`, `ChElementShellANCF_3833`, `ChElementHexaANCF_3813`, `ChElementHexaANCF_3813_9`, and `ChElementHexaANCF_3843`

The ANCF has an efficient way to exactly calculate the generalized force due to gravity.  In the past this efficient gravity calculation method had to be explicitly enabled with the `SetGravityOn()` function which had to be coupled with a call to disable gravity at the mesh level `mesh->SetAutomaticGravity(false);`.  These elements have now been setup so that the default mesh level gravity calculation now automatically calls the efficient and exact ANCF gravity calculation.  With this change the `SetGravityOn()` function has been eliminated as it is no longer needed to enable the ANCF specific gravity calculation.


**Added - Ability to Apply Moments:** 

Applies to: `ChElementBeamANCF_3243`, `ChElementBeamANCF_3333`, `ChElementShellANCF_3423`, `ChElementShellANCF_3443`, `ChElementShellANCF_3833`, and `ChElementHexaANCF_3843`

Moments can be applied at any point within these elements just like forces.  For applied forces and moments, the first 3 entries in the force vector are assumed to be the applied force vector in global coordinates.  The second 3 entries are assumed to be the applied moment in global coordinates.  Any entries beyond the first 6 are ignored.  With this change, the returned Jacobians for potential use with numeric integration were updated to reflect the actual current configuration line/area/volume ratio rather than the reference configuration line/area/volume ratio.

**Added - Contacts:** 

- For `ChElementBeamANCF_3243` and `ChElementBeamANCF_3333`, the contacts are calculated using a capsule shape between the two end nodes whose radius is equal to the diagonal length of the reference cross-section shape.  This is the same approach as `ChElementBeamEuler`.

- For `ChElementShellANCF_3443`, a skin at the midsurface is used just like `ChElementShellANCF_3423` and `ChElementShellANCF_3443`.

- For `ChElementHexaANCF_3813` and `ChElementHexaANCF_3843`, a linear quadrilateral face is added to the free faces just like `ChElementHexaANCF_3813_9`.


### [Added] New Chrono::Vehicle features

1. A mechanism was added to allow replacing selected kinematic joints with bushings in various Chrono::Vehicle templates.  Several wheeled vehicle suspension templates, the `ChBalancer` subchassis template, as well as the tracked vehicle suspension and track shoe templates were updated to include this option.  

   A particular joint connection with this new option will be modeled as a bushing if bushing data is provided and as a kinematic joint otherwise. For example, the connections of the upper control arms to the chassis in the double wishbone suspension will be modeled as revolute joints (as before) if the virtual method `getUCABushingData` return `nullptr` and as bushings otherwise.  Bushing information is passed as a structure which provides stiffness and damping in the "constrained" linear and rotational directions and stiffness and damping in the DOF directions of the corresponding kinematic joint (see `ChVehicleBushingData`).  When instantiating a vehicle subsystem template through a JSON specification file, a joint with this capability will be modeled as a bushing if a JSON key "Bushing Data" is included.

2. All wheeled vehicle suspension templates that used to model their tierods using distance constraints were updated to optionally use rigid bodies for the tierods (these include the double and single wishbone, MacPherson, multi-link). A derived class specifies the type of tierod model by overriding the virtual function `UseTierodBodies`.  In a JSON file specification for such a suspension, the tierods will be modeled as bodies if the "Tierod" object includes the keys "Mass", "Inertia", and "Radius" and as distance constraints otherwise.

   When tierods are modeled as rigid bodies they will be connected using a spherical and universal joint or using bushings, depending on whether or not bushing data is provided.

3. JSON-based specification of a wheeled vehicle was enhanced to allow specification of rear chassis and associated chassis connectors, as well as subchassis subsystems.  An example set of JSON specification files for modelling an MTV truck with rear walking beam suspensions is available under the `data/vehicle/mtv/` directory.

4. The interface between a Chrono::Vehicle and a powertrain was modified to completely decouple the two systems and use a force-displacement co-simulation approach for all possible combinations of powertrain and driveline templates. In particular, this now allows using a shafts-based powertrain to drive one of the “simple” drivelines.

   Note also that a drivetrain's `GetDriveshaftSpeed` function now always reports a positive angular speed for a vehicle moving forward and a negative value for reverse (although internally these signs must be reversed due to the particular implementation of the shafts-body constraint in Chrono).

5. The contact manager for tracked vehicles was extended to also allow use with a track test rig. Furthermore, new public methods on `ChTrackedVehicle` and `ChTrackTestRig` allow controlling rendering of contact information (normals and/or contact forces) for all monitored subsystems.

6. Support was added for specifying and applying user-defined external forces on a vehicle's chassis.  Such forces are defined in a class derived from `ChChassis::ExternalForce` which should override the `Update` method (to calculate new values for the force and/or its application point at each synchronization of the chassis state).  An arbitrary number of such forces can be defined and added using `ChChassis::AddExternalForce`.

7. A demonstration program (`demo_VEH_RenderJSON`) was created to illustrate visualization of a Chrono::Vehicle model based on JSON specificatin files.  Using the Chrono::OpenGL run-time visualization module, this demo program allows re-creating the vehicle model after a potential change to one or more JSON specification files (use key `U` to trigger).

### [Added] New robot models

Two new models were added to the collection Chrono robot models:

- The **Curiosity** Mars Rover is a six-wheel rover model. The model can simulate the Curiosity-class Mars rover which includes a passive Rocker-Bogie suspension system. The operation and the usage of the Curiosity Rover is similar to the Viper Lunar Rover. The steering function of the Curiosity Rover needs to be explicitly controlled by calling
  ```cpp
  SetSteerSpeed(double speed, WheelID id)
  ```
  This independent steering control allows the rover model to conduct many types of steering maneuvers. The linear DC motor model in Curiosity is similar to the DC motor in Viper (see below).

  `demo_ROBOT_Curiosity_SCM` illustrates the rover crossiung symmetric obstacles on SCM deformable terrain and `demo_ROBOT_Curioisty_Rigid` shows the rover being operated on rigid terrain while climbing a stair-shaped obstacle. Both demos show the initialization process of the Curiosity rover model and the simulated Rocker-Bogie suspension system when crossing obstacles.

- The **Turtlebot** is a common basic robot used as demonstration in various robot siomulation packages (e.g., Gazebo/ROS). This robot consists of two drive wheels and one passive wheel. The steering function can be controlled by calling 
  ```cpp
  SetMotorSpeed(float rad_speed, WheelID id)
  ```
  on both wheels and using the speed difference between left and right wheels to turn. This is a model skeleton and in the future more functionalities can be added as necessary, such as adding sensors for autonomous driving simulation. 

  `demo_ROBOT_Turtlebot_Rigid` shows a turtlebot model operated on rigid terrain and the turning operation.

In addition, new capabilities and functionality were added to the **Viper** Lunar Rover model. These include steering controls, linear DC motor models, and an active-controlled suspension system. The steering function is achieved by four rotational motors in the Z directions (vertical direction of the rover, perpendicular to the drive motor). The steering of the rover can be accessed using the function
```cpp
SetTurn(TurnSig id, double turn_speed)
```
to specify the turn signal (left/right/hold) and the speed of the turn. The active suspension control is achieved through eight lifting motors located on the connection points between upper/lower suspension and the rover chassis. This suspension replaces the current passive suspension which only had two springs. These two springs were maintained in the new suspension system in order to include damping. Control of the active suspension can be achieved through
```cpp
SetLiftMotorSpeed(double rad_speed, WheelID id)
```
The linear DC motor is a new option which can be used to replace the constant angular velocity motor. The function 
```cpp
SetDCControl(bool dc_control)
```
must be called before the initialization of the rover. This new function can simulate a simple DC motor with a linear torque-angular speed characteristic. The linear torque-speed map can be set using
```cpp
SetMotorNoLoadSpeed(double rad_speed, WheelID id)
```
and
```cpp
SetMotorStallTorque(double torque, WheelID id)
```

`demo_ROBOT_Viper_Rigid` and `demo_ROBOT_Viper_SCM` were modified to reflect changes in the initialization and controls. 


### [Added] New multicore collision detection system

The collision detection system previously embedded in Chrono::Multicore was updated and also made available to the usual Chrono systems (ChSystemNSC and ChSystemSMC) as an alternative to the Bullet-based collision detection system.  The new collision detection system (`ChCollisionSystemChrono`) uses a single-level adaptive grid for broadphase; for the narrowphase, it uses analytical intersection functions for certain pairs of known primitive shapes with fallback to an MPR (Minkovski Portal Refinement) alhgorithm.  In addition to the features previously available in Chrono::Multicore, the new stand-alone collision detection system includes additional analytical collision functions (e.g., for box-box interaction), as well as support for ray intersection.

The new collision system requires the `Thrust` library which is included in the CUDA toolkit or stand-alone (https://github.com/NVIDIA/thrust). If Thrust is available at configuration time, the `ChConfig.h` header defines a macro `CHRONO_COLLISION` (which can be used in user code to check availability of the new collision detection system).

The collision system type is specified through the enum `chrono::collision::ChCollisionSystemType` with valid values `BULLET` or `CHRONO`.  
By default, both ChSystemNSC and ChSystemSMC use the Bullet-based collision detection system.  Use of the new collision detection system can be enabled either by calling `ChSystem::SetCollisionSystemType` or else by constructing an object of type `ChCollisionSystemChrono` and then calling `ChSystem::SetCollisionSystem`. The latter method allows for changing various parameters controlling the broadphase and narrowphase algorithms from their default values.  For example:
```cpp
chrono::ChSystemSMC sys;
//...
sys.SetCollisionSystemType(chrono::collision::ChCollisionSystemType::CHRONO);
```
or
```cpp
#include "chrono/ChConfig.h"
#ifdef CHRONO_COLLISION
#include "chrono/collision/ChCollisionSystemChrono.h"
#endif
// ...
chrono::ChSystemSMC sys;
// ...
#ifdef CHRONO_COLLISION
auto collsys = chrono_types::make_shared<chrono::collision::ChCollisionSystemChrono>();
collsys->SetBroadphaseGridResolution(ChVector<int>(2, 2, 1));
sys.SetCollisionSystem(collsys);
#endif
```
On the other hand, a Chrono::Multicore system (`ChSystemMulticoreNSC` or `ChSystemMulticoreSMC`) defaults to using the new multicore collision detection system.

See the documentation of `ChCollisionSystemChrono` for details on the various parameters controlling the behavior of the underlying algorithms.  Note that, for backward compatibility, the existing mechanism for setting algorithmic parameters in Chrono::Multicore was preserved.  In other words, one can still use code such as:
```cpp
chrono::ChSystemMulticoreNSC sys;
// ...
sys.GetSettings()->collision.collision_envelope = 0.01;
sys.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
```

Because of the different underlying data structures, the Chrono multicore collision detection system requires collision models of the new type `ChCollisionModelChrono`. As such, the `ChBody` constructor was modified to take the collision system type as an argument (default `BULLET`). Constructors for the various `ChBodyEasy***` classes that take the collision system type as an argument are also provided. The user must ensure that objects with **compatible** collision models are added to a system! For example:
```cpp
auto collision_type = chrono::collision::ChCollisionSystemType::CHRONO;
chrono::ChSystemNSC sys;
sys.SetCollisionSystemType(collision_type);
// ...
auto body = chrono_types::make_shared<chrono::ChBody>(collision_type);
sys.AddBody(body);
// ...
```

Alternatively, for a more flexible code, one can use the `ChSystem::NewBody` and `ChSystem::NewBodyAuxRef` to construct a ChBody or ChBodyAuxRef, respectively, with a collision model consistent with the current collision system.  This assumes that the underlying collision system in the Chrono system was already set with one of the methods mentioned above:
```cpp
auto collision_type = chrono::collision::ChCollisionSystemType::CHRONO;
chrono::ChSystemNSC sys;
sys.SetCollisionSystemType(collision_type);
// ...
auto body = std::shared_ptr<chrono::ChBody>(sys.NewBody());
```

A few of the Chrono demos were modified to illustrate the use of the new collision detection system (e.g., demo_IRR_collisionSMC, demo_IRR_collisionNSC, demo_IRR_motors), while a new demo_IRR_raycast_test demostrates the ray intersection capabilities.

**Features.**
Some of the salient features of the new multicore collision detection system are:
- analytical collision functions for several pairs of shapes (dynamic dispatching based on shape type if using the default `ChNarrowphase::Algorithm::HYBRID` narrowphase strategy):

 |                |     _sphere_       |        _box_       |       _rbox_       |      _capsule_     |      _cylinder_    |       _rcyl_       |     _trimesh_      | 
 | -------------- |     :------:       |        :---:       |       :----:       |      :-------:     |      :--------:    |       :----:       |     :-------:      |
 | _**sphere**_   | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
 | _**box**_      |                    | :heavy_check_mark: |         :x:        | :heavy_check_mark: |          :x:       |         :x:        |        :x:         |
 | _**rbox**_     |                    |                    |         :x:        |         :x:        |          :x:       |         :x:        |        :x:         |
 | _**capsule**_  |                    |                    |                    | :heavy_check_mark: |          :x:       |         :x:        |        :x:         |
 | _**cylinder**_ |                    |                    |                    |                    |          :x:       |         :x:        |        :x:         |
 | _**rcyl**_     |                    |                    |                    |                    |                    |         :x:        |        :x:         |
 | _**trimesh**_  |                    |                    |                    |                    |                    |                    |        :x:         |

- analytical collision functions for non-strictly convex shapes produce multiple collision points, as appropriate (e.g., up to 8 for box-box).
- support for efficient intersection tests of mono-disperse spherical 3-D particle systems.
- calculations done in double precision.
- multicore parallel broadphase and narrowphase 
- definition of the broadphase grid with fixed number of cells, fixed cell dimensions, or fixed shape density.
- support for an optional "active" axis-aligned bounding box (objects leaving this area are automatically disabled).
- ray casting is thread safe (i.e., multiple ray intersectino tests can be done concurrently, for example in a parallel OpenMP for loop).

**Limitations.**
The main limitation of the new multicore collision detection system is that removal of collision models from the collision system is currently not supported.  As such, bodies with collision enabled cannot be removed from the system once added.

**Work in progress.**
The following enhancements are currenty under development:
- ray intersection with generic convex shapes
- support for collision of flexible bodies


### [Added] Miscellaneous additions to Chrono::Gpu

**Added - Specification of the compuational domain**

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

**Added - calculation of total kinetic energy**

A new function, `ChSystemGpu::GetParticlesKineticEnergy` was added to calculate and return the total kinetic energy of the granular particles.

**Added - Contact material properties**

For contact force calculation that uses material-based parameters, such as Young's Modulus, Poisson ratio and coefficient of restitution, the following function has to be called (set `val` to `true`), 

```cpp
void UseMaterialBasedModel(bool val);
```
Note that the default setting is using user-defined stiffness and damping ratio for contact forces, so no need to set `val` to `false`. The corresponding material properties associated with particles, boundary and mesh can be set using the following functions,
````cpp
void SetYoungModulus_SPH(double val);
void SetYoungModulus_WALL(double val);
void SetYoungModulus_MESH(double val);

void SetPoissonRatio_SPH(double val);
void SetPoissonRatio_WALL(double val);
void SetPoissonRatio_MESH(double val);

void SetRestitution_SPH(double val);
void SetRestitution_WALL(double val);
void SetRestitution_MESH(double val);
````

**Changed - Spherical boundary condition**

Boundary condition type `Sphere` is now defined as a numerical boundary with mass assigned. Simple dynamics among `BCSphere` and granular particles can be performed, see example `demo_GPU_balldrop.cpp`. The spherical boundary is created with:
````cpp
size_t CreateBCSphere(const ChVector<float>& center, float radius, bool outward_normal, bool track_forces, float mass);

````
where `outward_normal` is set to true if granular particles are outside the sphere. Some get and set methods are available during the simulation stage:
````cpp
ChVector<float> GetBCSpherePosition(size_t sphere_id);
void SetBCSpherePosition(size_t sphere_bc_id, const ChVector<float>& pos);
ChVector<float> GetBCSphereVelocity(size_t sphere_id);
SetBCSphereVelocity(size_t sphere_bc_id, const ChVector<float>& velo);
````

**Added - Rotating plane boundary condition** 

A `BCPlane` type boundary condition of id `plane_id` can be set to rotate with respect to point `center` at a constant angular velocity `omega`
````cpp
void SetBCPlaneRotation(size_t plane_id, ChVector<double> center, ChVector<double> omega);
````


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

### [Added] Support for the Emscripten compiler targeting WebAssembly

Chrono now provides limited support for compiling to WebAssembly and running in browser or Node.js. The core library is supported along with Chrono::OpenGL and Chrono::Vehicle. It is recommended to use the `emcmake` wrapper and Ninja generator when targeting WASM to ensure that all of the configuration options are set correctly. 

```sh
cd build
emcmake ccmake -G Ninja ..
ninja
``` 

**Changed - Shaders embedded using embedfile.cpp are now generated in pure CMake**

This allows for cross-compilation which is necessary for WASM. 

**Changed - OpenGL components now target OpenGL ES 3.0**

WebAssembly platforms typically use WebGL, which maintains a feature set roughly on par with OpenGL ES. WebGL 2.0 is able to emulate almost all of OpenGL ES 3.0, which is similar to the capabilities of the previously supported target of OpenGL 3.3. This modification should also improve overall Chrono::OpenGL performance on low-power rendering hardware such as ultra-portable laptops or mobile devices. 


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
