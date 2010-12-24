
  CHRONO::ENGINE  
  
  History of API changes, bug fixes, new features


Release 1.2.0
x-xx-xxxx

- New API 

- Fixed a bug in the collision detection engine, that
  caused some collisions to be missed once in a while.
  This was related to different ways of managing thresholds
  in Bullet and Chrono::Engine.

- The CH_UNIT_CUDA flag has been renamed CH_UNIT_GPU.


Release 1.1.0
3-11-2010

- The 64 bit versions of dll and lib are included
  in the Windows distribution.
  
- New feature: cohesion in contacts. This can be
  activated using the ChCustomCollisionPointCallback
  as shown in demo_cohesion.cpp 
  
- New demo 'demo_cohesion', showing the simplified
  cohesion model, a new feature that allows to model
  granular material with some sticking (ex. soil).
  
- Collision engine updated to Bullet 2.77.
  Now also the collision shapes of rigid bodies can be
  marshalled using the C::E serialization.

- New unit: ChronoEngine_MPI.dll.
  This will be used to host code for cluster computing.
  It contains features for exploiting the MPI interface
  for high performance parallel computing. Compatibility
  is tested under MPICH-2. 
  Under development / please do not use it.

- All CUDA code, previously spread in many directories
  and mixed with normal code, now is grouped in a
  single directory: unit_GPU. 

- ChLcpIterativeCuda renamed ChLcpIterativeSolverGPUsimple.
  For this reason, and for the previous one, some
  includes have been changed.
  
- ChModelBullet::SetFamilyMaskNoCollisionWithFamily was
  broken in last release, since the Bullet collision code
  was updated with different functionalities. Bug fixed.

- The Javascript code is now completely separated from the
  main ChronoEngine.dll. This means that Javascript API is
  no longer needed if one wants to recompile C::E on 
  platforms where JS is not easily available. 
  The Javascript features are in ChronoEngine_JS.dll .
  
- The Cuda/GPU code is now completely separated from the
  main ChronoEngine.dll. This means that Cuda API is
  no longer needed if one wants to recompile C::E on 
  platforms where Cuda is not easily available. 
  The Cuda/GPU features are in ChronoEngine_GPU.dll .
  
- The CH_NOCUDA flag has been renamed CH_UNIT_CUDA, and
  its behaviour is reversed (TRUE enables CUDA).
  
- The GLOBAL_Vars global variable is deprecated. Use
  CHGLOBALS() function instead. Also, implementation of
  static global data have been improved. 
   
- If the user forgets the DLL_CreateGlobals() and 
  DLL_DeleteGlobals pair at the beginning/end of the
  program, a static default ChGlobal is used anyway.
  
- New classes ChStreamOstreamWrapper, ChStreamIstreamWrapper,
  ChStreamOutBinaryStream, ChStreamInBinaryStream.
  Class ChStreamWrapper renamed ChStreamOstreamWrapper

- In demos using Irrlicht realtime 3d view, now the AABB
  bounding boxes of rigid bodies can be drawn (press 'i'
  to show debugging GUI).


Release 1.0.0
01-09-2010

- Elastic restitution is added. Use mybody->SetImpactC()
  to set the restitution coefficient. The Newton model
  is used (coefficient = ratio between pre/post impact 
  normal speeds).
 
- Added SetMinBounceSpeed() in ChSystem class, to set
  a clamping that avoids high frequency rebounces with 
  nonzero restitution coefficients.
  
- New ChConveyor class for creating conveyor belts.
 
- New demo_conveyor.exe to show an example of usage of 
  the new ChConveyor class.

- Major feature: new ChMatterMeshless object to simulate 
  point-based plastic deformable materials, using the 
  meshless FEM approach. Useful for simulating mud, etc.
  
- Major feature: new ChMatterSPH object to simulate 
  fluids. 
  
- Improved ChVoightTensor class (computation of invariants
  etc.). 
  
- New classes for continumm material properties: density,
  viscosity etc. Basic hookean material properties and 
  tensor transformations are added. 
  
- New classes for elastoplastic materials, with plastic
  flow. 
  
- Experimental classes for different models of plastic
  flow: the strain-yeld-based Von Mises, and the 
  Drucker-Prager criterion (the latter is under 
  development)

- Improved ChSharedPtr and ChSmartPtr classes (added 
  functions IsNull, boolean type casting, safer casting, etc.)
  
- Gyroscopic torque calculation contained a bug

- In case Chrono::Engine was installed in a path with spaces
  (ex. C:\blabla\Doucuments and Settings\blabla...) the make-chrono_lib
  did not work properly. Fixed.

- StreamOUTsparseMatlabFormat() for ChSparseMatrix, to dump sparse 
  matrices on disk, for example, in Matlab sparse format.
 
- ComputeFeasabilityViolation, FromVariablesToVector, FromVectorToVariables
  moved from ChLcpSolver to ChLcpSystemDescriptor.
  Also, new method BuildMatrices() in ChLcpSystemDescriptor, for 
  debugging, testing, etc., allowing to dump full mass and jacobian matrices.
  

Release 0.9.0
23-2-2010

- The convex decomposition code has been updated and improved.
  Now there is a class collision/ChCConvexDecomposition.h that
  manages the convex decomposition for concave meshes, with 
  settings.
  
- New demo 'demo_decomposition.exe', that shows interactively the
  effects of the parameters of the convex decompostion. It can
  save the decomposed hulls as files, with '.chulls' suffix, that
  can be loaded directly in the ChCollisionSystem.
  
- Improved ChIrrAppInterface.h  Now the realtime visualization
  system of the demos allows also to change other settings 
  such as the stabilization speed clamping and the lambda 
  complementarity smoothing using mouse & sliders (when the 
  info panel is displayed, pressing 'i').
  
- Added support for using a 'smoothing factor' lambda in the 
  iterative complementarity solvers. This can help convergency
  and precision.
  
- New example 'demo_forklift.exe', showing a forklift vehicle that can
  be driven using the keyboard.
  
- New ChShaft physical object: it represents a 1-degree-of-freedom
  object (i.e. a rotating shaft, with no 3D shape) that can be
  used to create schematic power trains, when assembled with ChShaftsGear,
  ChShaftsClutch, etc.
  
- New ChShaftsGear: connects two ChShafts in 1D models of
  powertrains. It can be also used to simulate rigid torque joints
  between two ChShafts. Remember: this is a 1D model, useful
  whan you do not need full 3D representation of shafts, gears,
  bearings, etc (in that case, you would use Chbody, ChLinkGear, etc.,
  but they are often more complicate than necessary).
  
- New ChShaftsClutch: it simulates a clutch between two
  ChShaft, for making 1D models of powertrains. Note that
  the clutch can be used also to simulate a brake, if one of the
  two ChShaft objects is set as 'fixed', to represent the truss.
  
- The ChShaftsClutch can have two different values for forward/backward 
  max transmissible torque, to simulate freewheels and overrunning clutches.

- New ChShaftsPlanetary: it simulates a planetary gear, that is a 
  constraint between three ChShaft entities. For instance, it can be
  used to make differentials in 1D powertrains of cars, or 1D models
  of epicycloidal reducers, or summation devices. 
  
- New classes for constraints between three entities, in LCP
  solver libary. Those classes are used, for example, to make
  differentials in 1D models of powertrains.
  These are ChLcpConstraintThree, ChLcpConstraintThreeBBShaft,
  ChLcpConstraintThreeGeneric.
  
- New ChLcpConstraintTwoGenericBoxed constraint.

- The ChLinkLinActuator, in long simulations, added some numerical 
  accumulation of errors (drifting of imposed positions). Fixed.
  
- Collision detection system updated to Bullet release 2.75.

- ChIrrCamera namespaces modified to avoid name pollution.

- The LCP_ITERATIVE_JACOBI solver had a bug that gave bad results
  in scenarios with lot of friction. (this is the normal CPU version
  of the Jacobi solver, the GPU version was correct). Fixed.
   
- For API developers: the ChCollisionSystem class does not have
  a SetContactContainer() function anymore, because we do not
  assume that there could be only _one_ contact container. Now, instead,
  during the Run() command, now no contact is added to any container - only
  the basic CD is performed. Then, after the Run(), the new function 
  ReportContacts(ChContactContainerBase* mcontactcontainer) can be called: 
  this will fill the 'mcontactcontainer' object by calling its usual BeginAddContacts(),
  AddContact(), EndAddContacts() functions. 
  The ChSystem manager will call automatically the ReportContacts() for the 
  default contact container of the physical system after the collision detection, 
  but it will also scan the list of ChPhysicsItems to see if there's some other 
  container to be filled. Note: the same happens with ChProximityContainer 
  physical items, that are filled ChContactContainer::ReportProximities() at 
  each collision detection; for instance this is currently used to manage the SPH. 
  
- API change! Class "ChParticles" renamed "ChParticlesClones". This makes more sense (it
  recalls that the particles share the same mass and the same collision shape). In 
  fact now it is inherited from a new class ChIndexedParticles, for generic types of 
  clusters of 6-DOF particles. Similarly, we added a new base class ChIndexedNodes for clusters
  of 3-DOF nodes, for instance in SPH and meshless point clouds. 
  
- New ChMatterSPH class. It allows the creation of plasto-elastic material with large
  nonlinear deformations, based on a particle approach. Note, still experimental!
  
- Demos updated to newest Irrlicht-1.7.1. Older versions of Irrlich not supported.


Release 0.8
10-7-2009
  
- New feature: rolling friction, and spinning (drilling) friction.
  The ChBody items can have coefficients for these two additional
  types of friction. Note, if using zero values for rolling and spinning
  friction, as by default, the simulation will be faster than with
  nonzero values.
  
- New demo for rolling friction 'demo_friction'.

- New function ChIrrTools::drawAllContactLabels(), to print infos
  near the contact points in 3D view (reaction forces, etc.)

- New ChMatlabEngine class, for starting a Matlab engine (if
  installed) and executing Matlab commands, or copying data
  from/to Matlab within a Chrono::Engine program. This is 
  very useful because, if you have Matlab on your system, you can
  exploit the very powerful plotting capabilities of Matlab.
  See the new example demo_matlab.exe . Note: configure the demo makefile
  for your Matlab SDK directories, if any, otherwise do not compile 
  the demo.

- Important note for developers: major changes in the class hierarchy 
  and interfaces of the collision detection and contact management!
  In detail, contacts aren't added to the ChLink list anymore, but
  they are handled by a new object of ChContactContainerBase type.
  Inherited from ChContactContainerBase there is a default ChContactContainer
  class specialization that stores contacts in simple linked lists, for a
  typical CPU implementation; while a future ChContactContainerGPU specialization
  will handle contacts directly as buffers on the GPU.
  Read the points above for important changes that stem from this new design.

- The demo_hopper and demo_reactor (optional examples) have been updated
  to support the new way of handling contacts. Also, they do not use
  anymore ChBody items as spheres, but rather use the new ChParticles container,
  that save about 20% of RAM.

- ChSystem::SetCustomComputeCollisionCallback() interface has been 
  changed. Its ChCustomComputeCollisionCallback::PerformCustomCollision()
  callback function now uses only (ChSystem* msys) as parameter. 
  Some users used it to execute custom addition of further contacts to
  the system. Now this can be done in the callback by using the new concept
      msys->GetContactContainer()->AddContact(...)

- ChSystem::SetCustomCollisionPointCallback() interface has been 
  changed. Its ChCustomCollisionPointCallback::ContactCallback()
  callback function now uses different parameters. See demo_suspension.cpp
  to see the modified example.

- New classes ChCollisionInfo, ChContactContainerBase, ChContactContainer,
  ChModelBulletBody, ChModelBulletParticle, ChPhysicsItem, ChParticles,
  ChAparticle, ChSharedMass, etc.
  
- Important change! ChBody and ChLink objects now share a parent
  class ChPhysicsItem. The ChPhysicsItem class is responsible of 
  communicating ChVariables and ChConstraints to the LCP solver.
  Not only this unified interface simplifies things in the ChSystem 
  methods, but also makes easier to develop custom new types of physical
  actors from the user-side. For instance, the new ChParticles object 
  is inherited from ChPhysicsItem. 
  The ChSystem supports whatever custom ChPhysicsItem object by 
  new functions AddOtherPhysicsItem() and RemoveOtherPhysicsItem().
  
- Simplified Add() / Remove().
  To add/remove items from the ChSystem, either you use AddBody(),
  AddLink() and AddOtherPhysicsItem(), or you can use a simplier unified
  function  ' Add(ChPhysicsItem*) ' , that work in all cases (since also
  ChBody and ChLink objects are inherited form ChPhysicsItem anyway).
  The same for the new ' Remove(ChPhysicsItem*) ' function.

- To scan through all the contacts, now you should use a callback.
  Use ChContactContainerBase::ReportAllContacts(ChReportContactCallback* mcallback)
  for this purpose. (Previously, one iterated through the ChLink list until
  he found a ChLink of contact type, but the new design based on contact containers
  made this obsolete). See inside ChIrrDisplayTools.h for an example.
  
- The ChIrrWizard::add_typical_Camera() created a custom Irrlicht 3d camera
  that did not work properly in debug mode. Bug in ChIrrCamera.h code fixed.
  
- The VisualStudio wizard (that is installed automatically when an API user
  uses the InnoSetup installer to install Chrono::Engine on his system) has
  been updated to support VC 9.0, with some improvements. 

- New function SetFriction() in ChBody and ChParticles. It sets
  both static and kinetic friction at once, with the same value.
  [Note: current version of the library does not yet support different
  values of static and kinetic friction coefficients, so this is
  suggested over the use of SetSfriction and SetKfriction, for the moment]
  

Release 0.7
10-5-2009

- New feature: object picking (in demos inherited from the 
  ChIrrAppInterface class, for example demo_collision.cpp).
  Use the middle mouse button to pick an object and move
  it in the interactive 3D view.
  Only objects that collide can be picked.
  
- For ChSystem object, you can use the method
  my_system.SetLcpSolverType(ChSystem::LCP_SIMPLEX);
  to enable a direct solver for calculating speeds. 
  This is much slower than all iterative solvers (LC_ITERATIVE_xxx 
  family) and _cannot_ be used for systems with contacts, but
  it has the advantage that it is very precise. Hence, it can
  be used for simple mechanisms where precision is an issue (ex. robots
  or articulated mechanisms as in the demo_fourbar.exe).

- The GPU solver did not support the application of external forces
  (ex. springs). Now these forces are supported.

- Bug with ChLinkDistance: the clamping on speed recovery
  worked only in one direction.
  
- New ChSystem::SetOmega and ChSystem::GetOmega to set
  overrelaxation parameter of the iterative solver, if any,
  saving lot of typing respect to previous versions.
  
- New slider in Irrlicht setting panel, to set omega
  overrelaxation

- Collision margins for cylinder objects were a bit displaced 
  (causing cylinders to 'sink' a bit into other shapes). Fixed.  


Release 0.6
06-03-2009

- The GPU complementarity solver has been optimized so that
  it skips unneeded compuations when thousands of objects 
  touch a fixed object (this happens often, for example 
  when many objects like pebbles are placed on the ground).
  
- Previous version of the GPU solver didn't work with the 
  INT_TASORA timestepper. Now it works also with this stepper.
  
- A bug in the GPU solver has been fixed, about the loss of
  precision when using friction coefficients larger than 0.1
  
- Default omega relaxation value for Jacobi CCP solver has 
  been reduced to 0.2 instead of 1.0, otherwise it was 
  too likely a divergent value.
  
- New Length2() and LenghtInf() in the ChVector class, to
  improve the speed of the norm of a vector in some optimised
  circumstances. 
  Same Length2() and LenghtInf() also for ChQuaternion class.
  
- New feature: rigid body sleeping. Use my_system.SetUseSleeping(true)
  When this feature is activated, bodies that remain in a resting
  position for enough time will be 'frozen' until a collision will
  set them awake again. In this way, simulating large scenarios with
  many objects resting on the ground can be much faster. However
  note that this feature aims at realtime simulations hence it 
  may affect the precision of the results.
  
- Body sleeping can be turned on/off selectively for each body,
  when turning on global sleeping with my_system.SetUseSleeping(true).
  
- Support for Irrlicht 1.5. Few compatibility bugs are fixed. 
  The old Irrlicht 1.4.2 isn't supported anymore.
  
- New custom camera for Irrlicht 3D viewing (the old Maya-like
  default camera in Irrlicht doesn't work anymore as desired).
  The new camera rotation/pan is controlled by mouse left and right buttons,
  the zoom is controlled by mouse wheel or rmb+lmb+mouse, the position can 
  be changed also with keyboard up/down/left/right arrows, the height can be 
  changed with keyboard 'PgUp' and 'PgDn' keys.
			
- Improved function ChIrrWizard functions (add_typical_Lights etc)

- New class ChIrrAppInterface to ease the creation of applications
  based on Irrlicht and Chrono::Engine.
  Applications based on this class will use Irrlicht for 3d rendering
  and, when the 'i' key is pressed on the keyboard, a small interface
  pops up and shows some profiling informations (num. of bodies, CPU
  time, etc.) and also allows the user to change the settings of the 
  integrator.
  
- Most demos have been modified to use the ChIrrAppInterface
  base class. Their source code is simplier, yet the visualization
  has more features (press the 'i' key to show interfaces for settings).
  
  


Release 0.5
25-12-2008

- Parallel solver for NVIDIA GPU architecture has been improved.
  A bug happened because single-precision in CUDA hardware caused
  normalization problems during time integration. Fixed.
  The performance of the GPU<->CPU memory communication has been
  improved.

- Porting to Linux platform completed (Thank to Jean-Luc Charles
  for hints and help). Only static linking is supported at the
  moment. The code has been succesfully tested in Ubuntu.
  There are two new directories in lib/ directory: Linux32_gcc
  and Linux64_gcc. Similar directories are in the bin/ directory
  (precompiled examples are not provided for Linux, to save space).
  To compile demos in Linux, cd into the ChronoEngine/demos/ directory
  and enter "make".

- New code for POSIX multithreading support in Linux/Unix platform.
  This allows the use of the multithreaded SOR parallel solver also
  in the Linux version. Note that multithreading support in Linux
  vesrion is still experimental.

- IMPORTANT: the recent ChMatrixBase class (introduced with
  release 0.4 as a base for the specialized ChMatrix33, 
  ChMatrixNM and ChMatrixDynamic) has been renamed as 
  ChMatrix for readability reasons, and also to be more
  coherent with releases previous than the v.0.4.

- Updated 'wizard' for the Visual C++ 2008 Express IDE: it is installed
  automatically in your VC editor during the installation of the 
  chrono::engine SDK. This wizard replaces the old wizard for 
  Visual C++ 2005 IDE, that is not supported anymore.

- New collision detection feature: contact families. Each body can 
  be assigned to a family (in the interval 0..15) and each body 
  can define a mask of families to collide with. This allows 
  situations where two bodies are constrained near each other
  (as in the shoes of a track, for instance) but there is no
  need that they generate contacts. See ChCollisionModel class, 
  for the functions SetFamily(int mfamily);  GetFamily();
  SetFamilyMaskNoCollisionWithFamily(int mfamily); etc.
  
- New demo 'demo_tracks', showing a simplfied bulldozer with 
  tracks. This is an example of how to use the 'clone' objects
  that share the same collision geometry (each shoe-track collision
  model is described by many shapes, coming from the convex decomposition
  of a concave triangle mesh from an .obj file, so 'cloning' saves 
  some memory and avoids that the cpu wastes time to repeat the 
  convex decomposition for each shoe.)

- Serialization of ChLink inherited class has been completed and
  improved (StreamOUT, StreamIN).

  
Release 0.4
2-11-2008

- Merged latest Chrono::Engine into the Chrono::R3D development
  trunk (for Realsoft OY). The Chrono::R3D plugin will use the
  new features.

- New function AddBarrel() in the ChCollisionModel class.
  This can be used to create convex collision geometries 
  that are defined by lathing an arc of an ellipse around
  an axis. It can be used, for instance, to create the 
  rollers of omnidirectional wheels of mobile robots, 
  simplified rigid wheels for motorbikes or bycicles, etc.

- Support of latest Irrlicht library for 3D realtime viewing
  in demos. Please download Irrlicht 1.5 if you are using 
  it for viewing. Also update your config.bak accordingly.

- New demo_mecanum.exe demo, showing a small mobile
  robot with unidirictional wheels.

- The entire ChVector class has been templated. 

- new function addChBodySceneNode_easyClone() for the
  Irrlicht realtime viewing tools. This can be used to 
  replicate a shape (with its mesh, colliding shape, mass,
  inertia, etc.) without the need of repeating the creation
  of distinct shapes, In fact, clones will use less memory 
  because they can share the collision shapes.
  
- new function AddCopyOfAnotherModel() in ChCCollisionModel
  class. This can be used to have multiple object that share
  the same collision shapes. This is memory-friendly, especially
  if the cloned collision shapes are very complicated.
  
- Previous version had a bug in sphere-box collision
  algorithm causing spheres to sink a bit into boxes: since
  the amount of the penetration was small, it was not noticed.
  Now it has been fixed.
  
- Speed improvements. Many small structures, formerly allocated 
  on heap, now are allocated on stack. This allows a
  general speedup of the simulation, up to x1.5 in many situations.
  
- Data structures optimized for smaller memory footprint. 
  On average, large simulations require 30% less RAM.
  Also, the NCP complementarity solver uses jacobians
  with elements of 'float' type instead of 'double' type,
  saving memory (with minimum impact on precision)
  
- IMPORTANT:
  Major refactoring and evolution of the ChMatrix
  class. Now the ChMatrix class is only a base
  matrix class for the following specialized classes:
    - ChMatrixDynamic (works exactly as the former ChMatrix)
    - ChMatrixNM (faster performance since it uses stack,
        but cannot be resized)
    - ChMatrix33 (for coordinate transformations etc.Uses stack)
  Since the ChMatrix now is an abstract class,  now
  you cannot create matrices from it: you must use the
  three child classes above. For example, if you previously 
  created a 4x4 matrix using ChMatrix<> mm(4,4); now you must
  write ChMatrixDynamic<> mm(4,4);  if you created 3x3 matrices
  with ChMatrix<> mm; now either use ChMatrixDynamic<> mm; or, 
  better, the custom ChMatrix33<> mm. Note that you still can
  use ChMatrix as a pointer to whatever matrix. 

- Functions for linear algebra have been moved to 
  a new file with the class ChLinearAlgebra.
  
- Supporting of Microsoft Visual C++ Express 2008.
  From this version, support of VC 2005 edition is 
  dropped.
  
- Bug in file streaming with different little/big endian 
  convention has been fixed.
 
- Removed static data structures that made reentrancy difficult;
  the ChronoEngine dll now should be thread-safe. 
  



Release 0.3.3
21-4-2008

- The ChLcpIterativeCuda solver now supports also the 
  bilateral links (not only the frictional contacts).
  The feature is still experimental (only 1000 scalar
  constraints by default, see num_GPU_bilaterals.h).
  
- Added support for compilation on 64bit platforms, thank
  to the two new compiler types COMPILER_MSVC_X64 and
  COMPILER_GCC_X64. This is under testing. 
   
- The GPU solver also takes care of the time integration, which 
  has been parallelized.

- Major revisions to the makefile system. If you created your own
  config.mak, please switch to the new one.
  
- The NONCUDA flag has been renamed CH_NOCUDA

- The 'b' values (known terms of constraint residuals) in the LCP solver structures 
  were considered negative, now are considered positive, to match the
  same sign formalism in A.Tasora and M.Anitescu papers.
  
- Removed some auxiliary data structure in ChBody, so that each ChBody
  object is almost 30 bytes smaller. Also, ChBody updates of applied
  forces are faster.
  
  
  
Release 0.3.3
1-4-2008
  
- New class ChLcpIterativeCuda: it is a solver which uses the GPU
  via CUDA parallel programming API (still at experimental stage).
  
- The ChSystem class supports the GPU solver ChLcpIterativeCuda

- New classes ChLcpConstraintTwoGPUcontN and ChLcpConstraintTwoGPUcontT
  added in the HyperOctant solver. Meanwhile, some classes in the 
  solver have been refacored.

- ChLinkContact class inserted in ChLink hierarchy, between ChLink and
  ChLinkFastContact, so that the new ChLinkGPUcontact is its child too.
  Meanwhile, the entire ChLinkFast contact has been refactored.
  
- The ChDisplayTool functions have been updated to reflect the
  new functions of the ChLinkContact class. User code is not affected
  by these changes.
  
- Support for DigitalMars compiler temporarily removed, since no user
  is using it. Removed Win32_DigitalMars directories from bin/ and lib/.

- The 'nvcc' compiler had a compatibility problem with MS 'nmake': in fact
  nmake wasn't able to launch the nvcc compilation (strange enough, this
  does not happen if building with 'make' from MingW). Anyway, a workaround
  has been developed (nmake sets an env.variable and launches nvcc_compile.bat,
  which launches nvcc with the proper parameters).


  
Release 0.3.2
22-12-2007

- All demos use the new Irrlicht-1.4 rendering engine (please UPGRADE
  to Irrlicht-1.4 if you used previous releases, and if you want to 
  recompile the demos). 
  
- Because of new features in Irrlicht-1.4, the ChBodySceneNode has
  been completely rewritten. (This class is useful only if you want to
  use Irrlicht as a visualization, as in the examples).
  
- Collision engine updated to Bullet 2.66, which provides better
  pool allocators.

- New function in ChSystem class: SetParallelThreadNumber().
  This function can be used to modify the number of threads which run in 
  parallel when you are using a parallel solver (ex. LCP_ITERATIVE_SOR_MULTITHREAD)
  In fact, by default the threads are 2, but more threads can give high performnce
  if you have a N-core processor (4-core, 8-core, etc.). If you are not using
  a multithreaded solver, this setting has no effect.

- Bug in the last version (0.3.1) of the collision engine: the creation of
  a ChSystem implied the allocation of 40Mb for contact manifolds, but it
  were never released at ChSystem deletion, hence: memory leakage. Fixed.

- Improved basic parser of ASCII streams in Chstream.h.

- New static functions to set the default margin and envelope. Use them once,
  and later, all times you will add geometries to your ChCollisionModel(s), these
  margin and envelope values will be used by default(if not otherwise specified):
  ChCollisionModel::SetDefaultSuggestedEnvelope();
  ChCollisionModel::SetDefaultSuggestedMargin();

- New static string  irr::scene::irrlicht_default_obj_dir  in ChBodySceneNodeTools.h,
  to specify the directory where 3D .obj files with models for sphere, cylinder and
  cube can be found. Default is "../data/", but it can be changed. 
  
- New ChLinkLockHook joint, wihch can be used to create hinges similar to 
  spherical joints, but one of the three rotations is not allowed. That is, the objects
  are connected with a vertical hook (or a short chain) so that rotation on link's Y is
  not possible. 

- Light range of Irrlicht created by ChIrrWizard::add_typical_Lights() is higher, 
  because scenes were too dark in Irrlicht 1.4
  
- Bug: when zero contacts happened, the LCP_ITERATIVE_SOR_MULTITHREAD solver crashed.
  Fixed.
  
- API help for class documentation has been updated (previously was not complete because
  of a problem with the Doxygen automatic documentation).



--------------------------------------------------------
  
Release 0.3.1
28-11-2007

- New LCP_ITERATIVE_SOR_MULTITHREAD solver type, which exploits dual-core
  processors. The multithreading allows faster simulation.
  To activate it, use ChSystem::SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
  This feature is still experimental (optimal processor balancing and efficient
  thread spinlocks are not yet implemented) but enough to work.
  
- Iterative solver 'warm starting' feature can be used also for the Tasora
  stepper (previously it worked only for Anitescu stepper).
  
- The way that collision margin was used has changed. Now contacts are
  created also a bit before the shapes come into contact. This improves
  the stability of the complementarity solver.

- New ChHashTable class  for sublinear search of key-values records.

- New SetMaxPenetrationRecoverySpeed() and GetMaxPenetrationRecoverySpeed()
  methods in ChSystem class, for clamping correction speed in Anitescu stepper.
  
- New SetIterLCPwarmStarting() method in ChSystem for easily turn on/off the 
  warm starting feature of both iterative solvers (speed and pos.stabilization)
  if used.

- New SetTolSpeeds() and GetTolSpeeds() in ChSystem, to specify which is the
  tolerance threshold for satisfying speeds in constraints (i.e. the LCP solver
  will stop iterating if reaches this tolerance during the solution of the 
  speed-impulse problem)

- New SuggestSafeMargin() and SuggestEnvelope() functions in ChModelBullet class,
  replacing the old SuggestMargin() function. The SuggestSafeMargin() is almost 
  as the previous SuggestMargin(), while SuggestEnvelope() is used to get contact
  infos from collision detection even if distance>0. 
  
- Various bug fixes. Zero friction hanged the solver: fixed; Other minor fixes 
  here and there.



--------------------------------------------------------

Release 0.3.0
13-11-2007

- Collision engine updated to Bullet 2.64 rc2. 
  Better handling of collision envelopes, better concave-concave
  collision algorithm, etc.

- Support of convex decomposition of concave triangle meshes! (just
  remember: the triangle mesh must have a correct topology, ie. without
  holes, cracks, degenerate points, etc. Also, the convex decomposition 
  can take some CPU time, so -for complex projects like videogames- it
  would be better if your art department performs convex decomposition 
  off-line with some in-house asset tools, then you quickly load compounds 
  of convex shapes on the fly, when the exe starts.)
  
- Improved demo_racing example: the car can cross some bumps which
  are imported using convex decomposition of generic concave meshes.

- Improved ChLinkGear constraint. The previous version had a bug in 
  the SetCheckphase() function - now it works.
 
- New ChLinkPulley constraint. Can be used to constraint two 
  pulleys as it they were connected by synchro belts.
  
- New example demo_gears. It explains how to simulate spur gears,
  bevel gears, inner-teeth gears and pulleys. All these topics are
  covered within a single mechanism, similar to an epicycloidal reducer.

- New RayTest() function in the ChCollisionSystem class. Example: it can be used
  for 'raycasting' type of simulations (ex. fast moving cars, where a simple
  radius/ground test can be sufficient to approximate the wheel/ground
  contact). Also, it can be used to simulate optical barrier sensors, and so on.
  

--------------------------------------------------------

Release 0.2.2
31-10-2007

- New installer: it automatically sets up the config.mak with 
  the correct path to the chrono::engine SDK (and to the Irrlicht
  SDK, if needed). The same for the Visual C++ 2005 ChronoEngineWizard.
  So the user will need little or no efforts at all in configuring
  the SDK after the installation - no more manual typing of settings
  in the config.mak is needed.

- New 'wizard' for the Visual C++ 2005 Express IDE: it is installed
  automatically in your VC editor during the installation of the 
  chrono::engine SDK. This means that you can use the menu 
  File/New/Project.. of the VC editor, than choose ChronoEngineWizard
  from the Visual C++ group, and it automatically set up a C++ template
  project for you, already configured to use Chrono::Engine libs and
  headers (the same for Irrlicht).

- New directory demos/msvc/ containing examples based on Microsoft
  Visual C++ 2005 Express projects. These demos can be compiled
  only by users with that IDE. 
  
- New ChIrrTools::drawChFunction() simple function to draw ChFunction
  objects as x-y plots in the Irrlicht 3D view. Test it with the
  new ChFunction_oscilloscope as in the 'racing' demo. Useful for 
  testing, debugging, easy visualization of data, etc.
  
- Improved 'racing' demo, showing a simplified car model with
  real suspensions.

- Improved Initialize() methods for links. Can make easier the
  creation of constraints.
  
- New directory ChronoEngine/msvc_config. It contains three property
  sheets to be used to quickly configure a VisualC++ 2005
  project - so that it can link and use ChronoEngine and Irrlicht.
  For example, create a project of 'Empty' type, go to the 'Property
  Manager' tab (usually at the left of the interface) and add the
  chrono_proj_config.vsprops property sheet to all configurations,
  then add chrono_proj_debug_libs.vsprops to your debug config
  only, then add chrono_proj_release_libs.vsprops to your release
  configuration only. Select the chrono_proj_config.vsprops, use
  the popup menu to see its properties, go to Macros tab, edit
  the two paths of ChronoEngine and Irrlicht so they match your
  installations. Note: all this task is not needed if you use the
  ChronoEngineWizard available in the Visual C++ Express, thank to the
  new SDK installer.
  
- New 'oscillator' project, showing how to use a Windows.Forms
  interface plus ChronoEngine plus an Irrlicht 3D view inside
  the Windows.Forms window. Note that the demo is not yet
  complete. 
  
- Integration with the Irrklang library for sound effects has
  been tested.

- New tutorial 'Visual C++ Wizard' explaining how to use the
  new wizard, step by step.
  
- New 'install' documentation, with more detailed info, and easier to 
  understand also for novice users.


--------------------------------------------------------

Release 0.2.1   
9-10-2007
  
- A very large refactoring of all classes inherited from
  ChLink has been performed. New ChLinkMasked, ChLinkMarkers
  and other classes has been added as 'layers' between the
  ChLinkLock and the base ChLink. 
  
- Thank to the refactoring of the ChLink class hierarchy, 
  the ChLinkFastContact class inherits as few data as possible
  from the base ChLink (which is really lightweight when compared
  to the old implementation). This means that simulating systems
  with more than 100'000 objects (with contacts) is possible 
  with systems with 2Gb of RAM, thank to this memory optimization.

- 'Warm starting' of iterative solver has been implemented. 
  This means that, optionally, chrono::engine can cache the
  values of reaction forces in constraints (or contacts) so
  that they can be reused at the next integration step as
  a 'guess' to start the iterative complementarity solver.
  Using warm starting, the solver requires few iterations to
  get appreciable precision, regardless of the complexity of
  the system (while without warm starting, the more the
  objects, the more iterations could be needed to converge).
  The drawback is that in some cases large stacks might 
  behave too 'bouncy'. 
  
- New ChLinkDistance link, if you need a simple and computationally-
  efficient constraint imposing distance between two points on two
  bodies. Unlike the ChLinkLockLinact joint, this has less features
  but it is simplier and faster to compute (it does not need the two
  markers in bodies, it just takes two relative positions on the two
  bodies representing the end points of the imposed distance).
  
- New example: demo_suspension.exe . This shows how to use the new
  ChLinkDistance link to model rod structures such as those in racing
  car suspensions, which can be considered massless. (This saves you
  from creating specific bodies for all rods, with spherical links at
  the end, which could be a waste of cpu resources if the effect of
  their masses is negligible anyway). 
  Also, it shows how to use the ChLinkSpring class to model a 
  spring-damper.
  
- New example: racing.exe This is a project for the Microsoft Visual C++
  2005 Express, which shows how to use only the IDE to compile a program 
  without dealing with makefiles and command line tools. 
 
- Both 'release' and 'debug' libraries are available now, either for
  MingW GCC and MSVC. Previously, only the release liblaries were distributed,
  so creating programs in debug mode was difficult (because mixing 
  debug and release libraries can give troubles if one uses 'new' for
  c++ objects shared across libraries). Now this is fixed, because
  ChronoEngineDBG.dll is distributed too.
  
- New example 'oscillator.exe' showing how to use chrono::engine and
  a Irrlicht 3D view within the Windows.Forms environment. Here, the 
  user interface (buttons, etc.) are created with the .NET toolset
  of Windows.Forms, but a 3D OpenGL view is managed by Irrlicht.
  
- New ChFunction_Oscilloscope class. Similar to ChFunction_Recorder, 
  but simplier and more targeted to recording with a circular buffer,
  for infinite storage queue of 'fifo' type.
  
  
--------------------------------------------------------

Release 0.1.9    
12-7-2007
  
- This last minute release fixes a problem with the 
  previous 0.1.8, that is the installer did not install
  some header files which might be required in some projects.

- Previous releases could not be installed in directories
  containing spaces in path names (ex: C:\Program files\chronoengine)
  otherwise this caused problems with the 'nmake' command-line building.
  Problem fixed.
  
--------------------------------------------------------

Release 0.1.8    
11-7-2007
  
- The previous 0.1.7 release was compatible with Irrlicht 1.3
  only, and gave compilation errors with the new release of
  Irrlicht 1.3.1 (for demo executables). Problem fixed.
  
- New 'benchmark.exe' demo, for testing and profiling purposes.

- Major code refactoring on the ChLink classes and derivates,
  but the updated code is NOT included in this minor release 
  and will we postponed to the future major release. Anyway,
  be prepared to meet a new hierarchy of ChLink classes in 
  future.

- New installer and new 'readme.txt' file with the list
  of changes in Chrono::Engine API.




--------------------------------------------------------

Release 0.1.7    

- First public downloadable release of the API.
