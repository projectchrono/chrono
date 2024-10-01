<!-- For syntax, see  https://github.github.com/gfm/ -->


Change Log
==========

- [Unreleased (development branch)](#unreleased-development-branch)
  - [\[Changed\] Eigensolvers refactoring](#eigensolvers-refactoring)
- [Release 9.0.1 (2024-07-03)](#release-901-2024-07-03)
  - [\[Fixed\] Bug fixes in FSI solver](#fixed-bug-fixes-in-fsi-solver)
  - [\[Fixed\] Miscellaneous bug fixes](#fixed-miscellaneous-bug-fixes)
- [Release 9.0.0 (2024-05-20)](#release-900-2024-05-20)
  - [\[Changed\] Default number of threads](#changed-default-number-of-threads)
  - [\[Changed\] Refactoring of class and function names](#changed-refactoring-of-class-and-function-names)
  - [\[Added\] Functional Mock-Up Interface (FMI) support](#added-functional-mock-up-interface-fmi-support)
  - [\[Added\] Chrono::Sensor features and updates](#added-chronosensor-features-and-updates)
  - [\[Added\] Chrono::ROS module](#added-chronoros-module)
  - [\[Changed\] Updated Chrono::VSG module](#changed-updated-chronovsg-module)
  - [\[Added\] New motion functions and filters](#added-new-motion-functions-and-filters)
  - [\[Changed\] Updated ChBlender exporter to Blender4.0](#changed-updated-chblender-exporter-to-blender4.0)
  - [\[Added\] Unilateral distance constraint](#added-unilateral-distance-constraint)
  - [\[Changed\] Collision detection refactoring](#changed-collision-detection-refactoring)
  - [\[Changed\] Application of terrain forces to vehicle systems](#changed-application-of-terrain-forces-to-vehicle-systems)
  - [\[Changed\] Modifications to the HHT integrator](#changed-modifications-to-the-hht-integrator)
  - [\[Added\] Modeling hydraulic circuit elements and hydraulic actuators](#added-modeling-hydraulic-circuit-elements-and-hydraulic-actuators)
  - [\[Added\] Support for modeling components with own dynamics](#added-support-for-modeling-components-with-own-dynamics)
  - [\[Changed\] Renamed SPHTerrain and RCCar vehicle classes](#changed-renamed-sphterrain-and-rccar-vehicle-classes)
  - [\[Changed\] Moved drive mode to automatic transmissions](#changed-moved-drive-mode-to-automatic-transmissions)
  - [\[Changed\] Transmission gear numbering](#changed-transmission-gear-numbering)
  - [\[Added\] Redundant constraints remover](#added-redundant-constraints-remover)
  - [\[Changed\] Serialization expanded and improved](#changed-serialization-expanded-and-improved)
  - [\[Changed\] Rewrite of Pac02 handling tire model](#changed-rewrite-of-pac02-handling-tire-model)
  - [\[Added\] New URDF parser](#added-new-urdf-parser)
  - [\[Added\] Support for STL 3D file format](#added-support-for-stl-3d-file-format)
  - [\[Changed\] Definition and use of primitive geometric shapes](#changed-definition-and-use-of-primitive-geometric-shapes)
  - [\[Changed\] Chrono::Vehicle engine and transmission templates](#changed-chronovehicle-engine-and-transmission-templates)
  - [\[Added\] New generic template for wheeled suspension subsystems](#added-new-generic-template-for-wheeled-suspension-subsystems)
  - [\[Changed\] CMake configuration and utility build scripts](#changed-cmake-configuration-and-utility-build-scripts)
  - [\[Added\] SPHTerrain - continuum representation method for deformable terrain](#added-sphterrain---continuum-representation-method-for-deformable-terrain)
  - [\[Added\] TMSimple tire model](#added-tmsimple-tire-model)
  - [\[Added\] Blender plug-in for post-process visualization](#added-blender-plug-in-for-post-process-visualization)
  - [\[Added\] VSG-based run-time visualization module](#added-vsg-based-run-time-visualization-module)
  - [\[Changed\] Support for linear and nonlinear vehicle force elements](#changed-support-for-linear-and-nonlinear-vehicle-force-elements)
- [Release 8.0.0 (2022-12-21)](#release-800-2022-12-21)
  - [\[Added\] Chrono::Sensor features and updates](#added-chronosensor-features-and-updates)
  - [\[Fixed\] Closed-loop vehicle paths](#fixed-closed-loop-vehicle-paths)
  - [\[Added\] Miscellaneous Chrono::Vehicle extensions](#added-miscellaneous-chronovehicle-extensions)
  - [\[Changed\] Chrono::FSI API changes](#changed-chronofsi-api-changes)
  - [\[Added\] User-defined SMC contact force calculation](#added-user-defined-smc-contact-force-calculation)
  - [\[Changed\] Redesigned run-time visualization system](#changed-redesigned-run-time-visualization-system)
  - [\[Changed\] Vehicle inertia properties](#changed-vehicle-inertia-properties)
  - [\[Changed\] CMake project configuration script](#changed-cmake-project-configuration-script)
  - [\[Changed\] Right-handed frames in Chrono::Irrlicht](#changed-right-handed-frames-in-chronoirrlicht)
  - [\[Added\] Modal analysis module](#added-modal-analysis-module)
  - [\[Added\] Callback mechanism for collision debug visualization](#added-callback-mechanism-for-collision-debug-visualization)
  - [\[Changed\] Translational and rotational spring-damper-actuators](#changed-translational-and-rotational-spring-damper-actuators)
  - [\[Changed\] Refactor Chrono::Vehicle suspension test rigs](#changed-refactor-chronovehicle-suspension-test-rigs)
- [Release 7.0.3 (2022-04-17)](#release-703-2022-04-17)
  - [\[Fixed\]](#fixed)
- [Release 7.0.2 (2022-04-03)](#release-702-2022-04-03)
  - [\[Fixed\]](#fixed-1)
- [Release 7.0.1 (2022-01-07)](#release-701-2022-01-07)
  - [\[Fixed\]](#fixed-2)
- [Release 7.0.0 (2021-11-15)](#release-700-2021-11-15)
  - [\[Added\] DDS communicator in Chrono::Synchrono module](#added-dds-communicator-in-chronosynchrono-module)
  - [\[Added\] New terramechanics co-simulation module](#added-new-terramechanics-co-simulation-module)
  - [\[Changed\] Chrono::Fsi API redesign](#changed-chronofsi-api-redesign)
  - [\[Changed\] Sensor to improve performance and added features](#changed-sensor-to-improve-performance-and-added-features)
  - [\[Changed\] ANCF element improvements and additions](#changed-ancf-element-improvements-and-additions)
  - [\[Added\] New Chrono::Vehicle features](#added-new-chronovehicle-features)
  - [\[Added\] New robot models](#added-new-robot-models)
  - [\[Added\] New multicore collision detection system](#added-new-multicore-collision-detection-system)
  - [\[Added\] Miscellaneous additions to Chrono::Gpu](#added-miscellaneous-additions-to-chronogpu)
  - [\[Added\] New loads for ChNodeFEAxyzrot](#added-new-loads-for-chnodefeaxyzrot)
  - [\[Added\] Analytical box box collision detection algorithm in Chrono::Multicore](#added-analytical-box-box-collision-detection-algorithm-in-chronomulticore)
  - [\[Added\] Checkpointing capabilities in Chrono::Gpu](#added-checkpointing-capabilities-in-chronogpu)
  - [\[Fixed\] Particle volume samplers and generators](#fixed-particle-volume-samplers-and-generators)
  - [\[Changed\] SCM deformable terrain improvements](#changed-scm-deformable-terrain-improvements)
  - [\[Changed\] Miscellaneous fixes to Chrono::Vehicle API](#changed-miscellaneous-fixes-to-chronovehicle-api)
  - [\[Added\] New tracked vehicle model](#added-new-tracked-vehicle-model)
  - [\[Changed\] Support for Z up camera in Chrono::Irrlicht](#changed-support-for-z-up-camera-in-chronoirrlicht)
  - [\[Changed\] Reading and writing collision meshes in Chrono::Gpu](#changed-reading-and-writing-collision-meshes-in-chronogpu)
  - [\[Added\] Support for the Emscripten compiler targeting WebAssembly](#added-support-for-the-emscripten-compiler-targeting-webassembly)
- [Release 6.0.0 (2021-02-10)](#release-600-2021-02-10)
  - [\[Added\] New Chrono::Csharp module](#added-new-chronocsharp-module)
  - [\[Added\] RoboSimian, Viper, and LittleHexy models](#added-robosimian-viper-and-littlehexy-models)
  - [\[Added\] Contact force reporting through user-provided callback](#added-contact-force-reporting-through-user-provided-callback)
  - [\[Changed\] Chrono::Gpu module rename](#changed-chronogpu-module-rename)
  - [\[Changed\] Chrono::Multicore module rename](#changed-chronomulticore-module-rename)
  - [\[Added\] Geometric stiffness for Euler beams](#added-geometric-stiffness-for-euler-beams)
  - [\[Added\] New Chrono::Synchrono module](#added-new-chronosynchrono-module)
  - [\[Changed\] Rename Intel MKL Pardiso interface module](#changed-rename-intel-mkl-pardiso-interface-module)
  - [\[Added\] Saving POV-Ray files from Irrlicht interactive view](#added-saving-pov-ray-files-from-irrlicht-interactive-view)
  - [\[Added\] Support for modelling wheeled trailers](#added-support-for-modelling-wheeled-trailers)
  - [\[Changed\] Enhancements to Chrono::FSI](#changed-enhancements-to-chronofsi)
  - [\[Added\] New Chrono::Sensor module](#added-new-chronosensor-module)
  - [\[Changed\] Setting OpenMP number of threads](#changed-setting-openmp-number-of-threads)
  - [\[Changed\] Redesigned SCM deformable terrain](#changed-redesigned-scm-deformable-terrain)
  - [\[Added\] Tracked vehicle support in PyChrono](#added-tracked-vehicle-support-in-pychrono)
  - [\[Changed\] Constitutive models for EULER beams](#changed-constitutive-models-for-euler-beams)
  - [\[Changed\] Constitutive models for IGA beams](#changed-constitutive-models-for-iga-beams)
  - [\[Added\] Obtaining body applied forces](#added-obtaining-body-applied-forces)
  - [\[Added\] Chrono::Vehicle simulation world frame](#added-chronovehicle-simulation-world-frame)
  - [\[Changed\] CASCADE module](#changed-cascade-module)
  - [\[Changed\] Collision shapes and contact materials](#changed-collision-shapes-and-contact-materials)
- [Release 5.0.1 (2020-02-29)](#release-501-2020-02-29)
  - [\[Fixed\]](#fixed-3)
- [Release 5.0.0 (2020-02-24)](#release-500-2020-02-24)
  - [\[Changed\] Refactoring of dense linear algebra](#changed-refactoring-of-dense-linear-algebra)
  - [\[Changed\] Eigen sparse matrices and updates to direct sparse linear solvers](#changed-eigen-sparse-matrices-and-updates-to-direct-sparse-linear-solvers)
- [Release 4.0.0 (2019-02-22)](#release-400-2019-02-22)

# Unreleased (development branch)

## [Changed] Eigensolvers refactoring

All Chrono eingesolvers share some common features:
- solve *generalized* eigenvalue problems i.e. `A*v = lambda*B*v`
- are iterative and based on shift-and-invert methods
- deal with real-valued matrices, but may have complex shifts and/or eigenpairs

but they differ depending if the `A` and `B` matrices are *symmetric* (`ChSym____`) or not (`ChUnsym____`).

The name pattern is now changed to `Ch[Sym|Unsym]GenEigenvalueSolver` to clarify this difference.

These eigenvalue solvers are meant to deal directly with *matrices*, not with Chrono `ChSystem`s nor with `ChAssembly`s. This is indeed a task for `ChModalSolver` classes.

The modal solvers are split into undamped and damped versions but, while the undamped case can generate either symmetric or unsymmetric problems, the damped case matrices are always unsymmetric. Because of this, different modal solvers may be equipped with different eigensolvers.

Further details are explained in the documentation.

# Release 9.0.1 (2024-07-03)

## [Fixed] Bug fixes in FSI solver 

- Use different formulas for computing BCE marker forces for the explicit and implicit SPH solvers (intermediate vector represents particle accelerations in the explicit solver, but represents particle forces in the implicit solvers).
- Fix GridSampler to generate points fully covering the specified sampling domain.
- Fix Chrono::FSI demos related to solid inertia properties, collision shapes, and BCE marker generation.
- Add missing velocity setting for FSI particle visualization.

## [Fixed] Miscellaneous bug fixes

- Add checks for CUDA and Thrust versions at configuration time (latest supported versions are CUDA 12.3.2 and Thrust 2.2.0). Disable Chrono::Multicore and GPU-based Chrono modules for newer versions.
- Fix calculation of relative position, velocity, and acceleration for `ChLinkMate`.
- Fix calculation of relative frames on connected bodies for `ChLinkDistance`.
- Implement tolerance-based stopping criteria for the Barzilai-Borwein solver.
- Fix bug in inflating the axis-aligned bounding box for a `ChGeometry` object.
- Fix bug in `ChTimer::GetTimeMilliseconds` which was previously reporting the elapsed time in microseconds.

# Release 9.0.0 (2024-05-20)

## [Changed] Default number of threads

The default values for the number of OpenMP threads used in various parts of Chrono were changed to be always 1.
The user is responsible to adjust these settings to values that are appropriate to their problem and hardware.

This affects OpenMP settings for the Chrono system and the PardisoMKL direct sparse linear solver.  
- the function `ChSystem::SetNumThreads` controls the following values:
  + *num_threads_chrono*: number of OpenMP threads used in FEA (for parallel evaluation of internal forces and their Jacobians) and in the SCM deformable terrain (for parallel ray casting).
  + *num_threads_collision*: number of OpenMP threads used by the collision detection algorithms (Bullet or Multicore).
  + *num_threads_eigen*: number of threads used in the Eigen sparse direct solvers (SparseLU and SparseQR) and a few linear algebra operations. Note that Eigen enables multi-threaded execution only under certain conditions (consult the Eigen documentation).

- the constructor for `ChSolverPardisoMKL` takes as an argument the number of OpenMP threads passed to MKL (default: 1) and used during the setup and solve phases.

## [Changed] Refactoring of class and function names

For consistency and uniformity, many classes and functions were renamed, modified, or even obsoleted.
Note that this represents a major public API change and we expect most user code will need to be updated to reflect these changes.

The main changes are collected in the following tables. These can serve as a "dictionary" for converting existing user codes to the new Chrono API.
Please make sure to also read the notes at the end of this log entry where we provide more details on changes that went beyond simple class or function renaming.

**Header files**

| File                                | Action                                    |
| :---------------------------------- | :---------------------------------------- |
| ChBitmaskEnums.h                    | remove                                    |
| ChController.h                      | rename: ChControllers.h                   |
| ChDistribution.h                    | rename: ChRandom.h                        |
| ChException.h                       | remove                                    |
| ChFunction_Const.h                  | rename: ChFunctionConst.h                 |
| ChFunction_BSpline.h                | rename: ChFunctionBSpline.h               |
| ChFunction_ConstAcc.h               | rename: ChFunctionConstAcc.h              |
| ChFunction_DoubleS.h                | rename: ChFunctionConstJerk.h             |
| ChFunction_Cycloidal.h              | rename: ChFunctionCycloidal.h             |
| ChFunction_Derive.h                 | rename: ChFunctionDerivative.h            |
| ChFunction_Fillet3.h                | rename: ChFunctionFillet3.h               |
| ChFunction_Integrate.h              | rename: ChFunctionIntegral.h              |
| ChFunction_Lambda.h                 | rename: ChFunctionLambda.h                |
| ChFunction_Mirror.h                 | rename: ChFunctionMirror.h                |
| ChFunction_Mocap.h                  | remove                                    |
| ChFunction_Noise.h                  | remove                                    |
| ChFunction_Operation.h              | rename: ChFunctionOperator.h              |
| ChFunction_Oscilloscope.h           | remove                                    |
| ChFunction_Poly345.h                | rename: ChFunctionPoly345.h               |
| ChFunction_Poly.h                   | rename: ChFunctionPoly.h                  |
| ChFunction_Ramp.h                   | rename: ChFunctionRamp.h                  |
| ChFunction_Recorder.h               | rename: ChFunctionInterp.h                |
| ChFunction_Repeat.h                 | rename: ChFunctionRepeat.h                |
| ChFunction_Sequence.h               | rename: ChFunctionSequence.h              |
| ChFunction_Sigma.h                  | rename: ChFunctionPoly23.h                |
| ChFunction_Sine.h                   | rename: ChFunctionSine.h                  |
| ChFunction_Setpoint.h               | rename: ChFunctionSetpoint.h              |
| ChFunctionPosition_line.h           | rename: ChFunctionPositionLine.h          |
| ChFunctionPosition_setpoint.h       | rename: ChFunctionPositionSetpoint.h      |
| ChFunctionPosition_XYZfunctions.h   | rename: ChFunctionPositionXYZFunctions.h  |
| ChFunctionRotation_ABCfunctions.h   | rename: ChFunctionRotationABCFunctions.h  |
| ChFunctionRotation_axis.h           | rename: ChFunctionRotationAxis.h          |
| ChFunctionRotation_setpoint.h       | rename: ChFunctionRotationSetpoint.h      |
| ChFunctionRotation_spline.h         | rename: ChFunctionRotationBSpline.h       |
| ChFunctionRotation_SQUAD.h          | rename: ChFunctionRotationSQUAD.h         |
| ChFx.h                              | remove                                    |
| ChKblock.h                          | remove                                    |
| ChKblockGeneric.h                   | rename: ChKRMBlock.h                      |
| ChLimit.h                           | rename: ChLinkLimit.h                     |
| ChLineBspline.h                     | rename: ChLineBSpline.h                   |
| ChLinkBeamIGAslider.h               | rename: ChLinkBeamIGAFrame.h              |
| ChLinkBrake.h                       | rename: ChLinkLockBrake.h                 |
| ChLinkClearance.h                   | rename: ChLinkLockClearance.h             |
| ChLinkDirFrame.h                    | rename: ChLinkNodeSlopeFrame.h            |
| ChLinkGear.h                        | rename: ChLinkLockGear.h                  |
| ChLinkLinActuator.h                 | rename: ChLinkLockLinActuator.h           |
| ChLinkPointFrame.h                  | rename: ChLinkNodeFrame.h                 |
| ChLinkPointPoint.h                  | rename: ChLinkNodeNode.h                  |
| ChLinkPointSpline.h                 | rename: ChLinkLockPointSpline.h           |
| ChLinkPointTriface.h                | rename: ChLinkNodeFace.h                  |
| ChLinkPulley.h                      | rename: ChLinkLockPulley.h                |
| ChLinkRackpinion.h                  | remove: merged into ChLinkMate.h          |
| ChLinkScrew.h                       | rename: ChLinkLockScrew.h                 |
| ChLinkTrajectory.h                  | rename: ChLinkLockTrajectory.h            |
| ChLists.h                           | remove                                    |
| ChLoadsXYZnode.h                    | rename: ChLoadsNodeXYZ.h                  |
| ChLoadsZYZROTnode.h                 | rename: ChLoadsNodeXYZRot.h               |
| ChLog.h                             | remove                                    |
| ChMaterialSurface.h                 | rename: ChContactMaterial.h               |
| ChMaterialSurfaceNSC.h              | rename: ChContactMaterialNSC.h            |
| ChMaterialSurfaceSMC.h              | rename: ChContactMaterialSMC.h            |
| ChMath.h                            | remove                                    |
| ChMathematics.h                     | remove and replace with:                  |
|                                     |         ChConstants.h                     |
|                                     |         ChUtils.h                         |
| ChShaftsBody.h                      | rename: ChShaftBodyConstraint.h           |
| ChShaftsMotorAngle.h                | rename: ChShaftsMotorPosition.h           |
| ChShaftsMotorTorque.h               | rename: ChShaftsMotorLoad.h               |
| ChSolvmin.h                         | remove                                    |
| ChStream.h                          | remove                                    |
| ChUpdateFlags.h                     | remove                                    |
| ChVector.h                          | rename: ChVector3.h                       |


**Classes and functions**

The following table summarizes all changes in class and member function names.
The table is sorted in alphabetical order of the old class name (in the 1st column) and then in alphabetical order of the old member function name (in the 2nd column).
The 3rd column specifies the action taken on that class or function: remove (indicating obsoleted functionality) or renamed (in which case the new class or function name is provided).
In some instances, the reader is directed to the "Notes" section for more details.

| Class                             | Function                      | Action                                           |
| :-------------------------------- | :--------------------------   | :----------------------------------------------- |
| -                                 | make_ChFunctionLambda         | remove                                           |
| -                                 | make_shared_ChFunctionLambda  | remove                                           |
| -                                 | ChAtan2                       | remove                                           |
| -                                 | ChClamp                       | relocate to utils/ChUtils.h                      |
| -                                 | ChClampValue                  | relocate to utils/ChUtils.h                      |
| -                                 | ChGetRandomSeed               | remove                                           |
| -                                 | ChMax                         | remove                                           |
| -                                 | ChMin                         | remove                                           |
| -                                 | ChPeriodicPar                 | remove                                           |
|                                   | ChRandom                      | replace with: ChRandom::Get                      |
| -                                 | ChSetRandomSeed               | replace with: ChRandom::SetSeed                  |
| -                                 | ChSignum                      | relocate to utils/ChUtils.h                      |
| -                                 | ChSineStep                    | implemented in ChFunctionSineStep                |
| -                                 | GetUniqueIntID                | remove                                           |
| -                                 | GetLog                        | remove                                           |
| -                                 | SetFirstIntID                 | remove                                           |
| -                                 | SetLog                        | remove                                           |
| -                                 | SetLogDefault                 | remove                                           |
| -                                 | StreamInDenseMAtlabFormat     | remove                                           |
| -                                 | StreamOutDenseMatlabFormat    | rename: ChStreamOut                              |
| -                                 | StreamOutSparseMatlabFormat   | rename: ChStreamOut (0- or 1- indexed option)    |
| -                                 | XdirToDxDyDz                  | remove                                           |
| ChAbsorbed_Power_Vertical         |                               | rename: ChAbsorbedPowerVertical                  |
| ChAparticle                       |                               | rename: ChParticle                               |
| ChArchiveFmu                      |                               | rename: ChOutputFMU                              |
| ChArchiveAsciiDump                |                               | rename: ChOutputASCII                            |
| ChArchiveExplorer                 |                               | rename: ChObjectExplorer                         |
| ChAssembly                        |                               |                                                  |
|                                   | Get_bodylist                  | rename: GetBodies                                |
|                                   | Get_linklist                  | rename: GetLinks                                 |
|                                   | Get_meshlist                  | rename: GetMeshes                                |
|                                   | Get_otherphysicslist          | rename: GetShafts                                |
|                                   | Get_shaftslist                | rename: GetOtherPhysicsItems                     |
|                                   | GetForceList                  | rename: GetForces                                |
|                                   | GetMarkerList                 | rename: GetMarkers                               |
|                                   | GetNbodies                    | rename: GetNumBodiesActive                       |
|                                   | GetNbodiesFixed               | rename: GetNumBodiesFixed                        |
|                                   | GetNbodiesSleeping            | rename: GetNumBodiesSleeping                     |
|                                   | GetNbodiesTotal               | rename: GetNumBodies                             |
|                                   | GetNcontacts                  | rename: GetNumContacts                           |
|                                   | GetNcoords                    | rename: GetNumCoordsPosLevel                     |
|                                   | GetNcoords_w                  | rename: GetNumCoordsVelLevel                     |
|                                   | GetNdoc                       | remove: no constraints at position level         |
|                                   | GetNdoc_w                     | rename: GetNumConstraints                        |
|                                   | GetNdoc_w_C                   | rename: GetNumConstraintsBilateral               |
|                                   | GetNdoc_w_D                   | rename: GetNumConstraintsUnilateral              |
|                                   | GetNdof                       | remove: it was a rough estimate                  |
|                                   | GetNlinks                     | rename: GetNumLinksActive                        |
|                                   | GetNmeshes                    | rename: GetNumMeshes                             |
|                                   | GetNphysicsItems              | rename: GetNumOtherPhysicsItemsActive            |
|                                   | GetNshafts                    | rename: GetNumShafts                             |
|                                   | GetNshaftsFixed               | rename: GetNumShaftsFixed                        |
|                                   | GetNshaftsSleeping            | rename: GetNumShaftsSleeping                     |
|                                   | GetNshaftsTotal               | rename: GetNumShaftsTotal                        |
|                                   | GetNsysvars                   | remove                                           |
|                                   | GetNsysvars_w                 | remove                                           |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChAssemblyAnalysis                |                               |                                                  |
|                                   | get_L                         | rename: GetLagrangeMultipliers                   |
|                                   | get_X                         | rename: GetStatePos                              |
|                                   | get_V                         | rename: GetStateVel                              |
|                                   | get_A                         | rename: GetStateAcc                              |
| ChBasisToolsBspline               |                               | rename: ChBasisToolsBSpline                      |
| ChBasisToolsBsplineSurfaces       |                               | rename: ChBasisToolsBSplineSurfaces              |
| ChBeamSectionCable                |                               |                                                  |
|                                   | GetBeamRayleighDamping        | rename: GetRayleighDamping                       |
|                                   | GetI                          | rename: GetInertia                               |
|                                   | SetBeamRayleighDamping        | rename: SetRayleighDamping                       |
|                                   | SetI                          | rename: SetInertia                               |
| ChBeamSectionEuler                |                               |                                                  |
|                                   | GetBeamRayleighDampingAlpha   | rename: GetRayleighDampingAlpha                  |
|                                   | GetBeamRayleighDampingBeta    | rename: GetRayleighDampingBeta                   |
|                                   | SetBeamRayleighDampingAlpha   | rename: SetRayleighDampingAlpha                  |
|                                   | SetBeamRayleighDampingBeta    | rename: SetRayleighDampingBeta                   |
| ChBeamSectionEulerAdvancedGeneric |                               |                                                  |
|                                   | SetXtorsionRigidity           | rename: SetTorsionRigidityX                      |
|                                   | SetYbendingRigidity           | rename: SetBendingRigidityY                      |
|                                   | SetZbendingRigidity           | rename: SetBendingRigidityZ                      |
| ChBeamSectionEulerSimple          |                               |                                                  |
|                                   | GetGshearModulus              | rename: GetShearModulus                          |
|                                   | GetXtorsionRigidity           | rename: GetTorsionRigidityX                      |
|                                   | GetYbendingRigidity           | rename: GetBendingRigidityY                      |
|                                   | GetZbendingRigidity           | rename: GetBendingRigidityZ                      |
|                                   | SetGshearModulus              | rename: SetShearModulus                          |
|                                   | SetGwithPoissonRatio          | rename: SetShearModulusFromPoisson               |
| ChBeamSectionShape                |                               |                                                  |
|                                   | GetNofLines                   | rename: GetNumLines                              |
|                                   | GetNofPoints                  | rename: GetNumPoints                             |
| ChBeamSectionTimoshenkoAdvancedGeneric  |                         |                                                  |
|                                   | GetYshearRigidity             | rename: GetShearRigidityY                        |
|                                   | GetZshearRigidity             | rename: GetShearRigidityZ                        |
|                                   | SetYshearRigidity             | rename: SetShearRigidityY                        |
|                                   | SetZshearRigidity             | rename: SetShearRigidityZ                        |
| ChBezierCurve                     |                               |                                                  |
|                                   | calcClosestPoint              | rename: CalcClosestPoint                         |
|                                   | eval                          | rename: Eval                                     |
|                                   | evalD                         | rename: EvalDer                                  |
|                                   | evalDD                        | rename: EvalDer2                                 |
|                                   | getNumPoints                  | rename: GetNumPoints                             |
|                                   | getNumSegments                | rename: GetNumSegments                           |
|                                   | getPoint                      | rename: GetPoint                                 |
|                                   | getPoints                     | rename: GetPoints                                |
|                                   | read                          | rename: Read                                     |
|                                   | reset                         | rename: Reset                                    |
|                                   | write                         | rename: Write                                    |
| ChBinaryArchive                   |                               | remove                                           |
| ChBody                            |                               |                                                  |
|                                   | Accumulate_force              | rename: AccumulateForce                          |
|                                   | Accumulate_torque             | rename: AccumulateTorque                         |
|                                   | Dir_Body2World                | remove                                           |
|                                   | Dir_World2Body                | remove                                           |
|                                   | Empty_forces_accumulators     | rename: EmptyAccumulators                        |
|                                   | Get_accumulated_force         | rename: GetAccumulatedForce                      |
|                                   | Get_accumulated_torque        | rename: GetAccumulatedTorque                     |
|                                   | GetBodyFixed                  | rename: IsFixed                                  |
|                                   | GetCollide                    | rename: IsCollisionEnabled                       |
|                                   | GetFrame_COG_to_abs           | rename: GetFrameCOMToAbs                         |
|                                   | GetGid                        | remove                                           |
|                                   | GetId                         | rename: GetIndex (internal use only)             |
|                                   | GetMaxSpeed                   | rename: GetMaxLinVel                             |
|                                   | GetMaxWvel                    | rename: GetMaxAngVel                             |
|                                   | GetSleeping                   | rename: IsSleeping                               |
|                                   | GetSleepMinSpeed              | rename: GetSleepMinLinVel                        |
|                                   | GetSleepMinWvel               | rename: GetSleepMinAngVel                        |
|                                   | GetUseSleeping                | rename: IsSleepingAllowed                        |
|                                   | Point_Body2World              | remove                                           |
|                                   | Point_World2Body              | remove                                           |
|                                   | RelPoint_AbsAcc               | remove                                           |
|                                   | RelPoint_AbsSpeed             | remove                                           |
|                                   | SetBodyFixed                  | rename: SetFixed                                 |
|                                   | SetCollide                    | rename: EnableCollision                          |
|                                   | SetGid                        | remove                                           |
|                                   | SetId                         | remove                                           |
|                                   | SetMaxSpeed                   | rename: SetMaxLinVel                             |
|                                   | SetMaxWvel                    | rename: SetMaxAngVel                             |
|                                   | SetNoGyroTorque               | rename: SetUseGyroTorque                         |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
|                                   | SetSleepMinSpeed              | rename: SetSleepMinLinVel                        |
|                                   | SetSleepMinWvel               | rename: SetSleepMinAngVel                        |
|                                   | SetUseSleeping                | rename: SetSleepingAllowed                       |
| ChBodyAuxRef                      |                               |                                                  |
|                                   | GetFrame_COG_to_REF           | rename: GetFrameCOMToRef                         |
|                                   | GetFrame_REF_to_abs           | rename: GetFrameRefToAbs                         |
|                                   | GetFrame_REF_to_COG           | rename: GetFrameRefToCOM                         |
|                                   | SetFrame_COG_to_REF           | rename: SetFrameCOMToRef                         |
|                                   | SetFrame_REF_to_abs           | rename: SetFrameRefToAbs                         |
|                                   | SetFrame_REF_to_COG           | rename: SetFrameRefToCOM                         |
| ChBodyFrame                       |                               |                                                  |
|                                   | To_abs_forcetorque            | remove                                           |
|                                   |                               | added: AppliedForceLocalToWrenchParent           |
|                                   |                               | added: AppliedForceParentToWrenchParent          |
| ChButterworth_Highpass            |                               | rename: ChButterworthHighpass                    |
| ChButterworth_Lowpass             |                               | rename: ChButterworthLowpass                     |
| ChCollisionModel                  |                               |                                                  |
|                                   | GetFamilyMaskDoesCollisionWithFamily | rename: CollidesWith                      |
|                                   | GetShape                      | rename: GetShapeInstance                         |
|                                   | GetShapes                     | rename: GetShapeInstances                        |
|                                   | SetFamilyMaskDoCollisionWithFamily   | rename: AllowCollisionsWith               | 
|                                   | SetFamilyMaskNoCollisionWithFamily   | rename: DisallowCollisionsWith            |
| ChConstraint                      |                               |                                                  |
|                                   | Build_Cq                      | rename: PasteJacobianInto                        |
|                                   | Build_CqT                     | rename: PasteJacobianTransposedInto              |
|                                   | Compute_c_i                   | rename: ComputeResidual                          |
|                                   | Compute_Cq_q                  | rename: ComputeJacobianTimesState                |
|                                   | Get_b_i                       | rename: GetRightHandSide                         |
|                                   | Get_c_i                       | rename: GetResidual                              |
|                                   | Get_cfm_i                     | rename: GetComplianceTerm                        |
|                                   | Get_g_i                       | rename: GetSchurComplement                       |
|                                   | Get_l_i                       | rename: GetLagrangeMultiplier                    |
|                                   | Set_b_i                       | rename: SetRightHandSide                         |
|                                   | Set_c_i                       | rename: SetResidual                              |
|                                   | Set_cfm_i                     | rename: SetComplianceTerm                        |
|                                   | Set_g_i                       | rename: SetSchurComplement                       |
|                                   | Set_l_i                       | rename: SetLagrangeMultiplier                    |
|                                   | Increment_q                   | rename: IncrementState                           |
|                                   | MultiplyAndAdd                | rename: AddJacobianTimesVectorInto               |
|                                   | MultiplyTandAdd               | rename: AddJacobianTransposedTimesScalarInto     |
| ChContactable                     |                               |                                                  |
|                                   | ContactableGet_ndof_x         | rename: GetContactableNumCoordsPosLevel          |
|                                   | ContactableGet_ndof_w         | rename: GetContactableNumCoordsVelLevel          |
|                                   | ContactableGetStateBlock_x    | rename: ContactableGetStateBlockPosLevel         |
|                                   | ContactableGetStateBlock_w    | rename: ContactableGetStateBlockVelLevel         |
|                                   | GetCsysForCollisionModel      | rename: GetCollisionModelFrame                   |
| ChContactNodeXYZROT               |                               | rename: ChContactNodeXYZRot                      |
| ChContactNodeXYZROTsphere         |                               | rename: ChContactNodeXYZRotSphere                |
| ChContactSurfaceMesh              |                               |                                                  |
|                                   | GetTriangleList               | rename: GetTrianglesXYZ                          |
|                                   | GetTriangleListRot            | rename: GetTrianglesXYZROT                       |
| ChContactSurfaceNodeCloud         |                               |                                                  |
|                                   | GetNnodesRot                  | rename: GetNumNodesRot                           |
|                                   | GetNodeList                   | rename: GetNodes                                 |
|                                   | GetNodeListRot                | rename: GetNodesRot                              |
| ChContactTriangleXYZROT           |                               | rename: ChContactTriangleXYZRot                  |
|                                   | GetTrianglesXYZROT            | rename: GetTrianglesXYZRot                       |
| ChContinuumElastic                |                               |                                                  |
|                                   | Get_BulkModulus               | rename: GetBulkModulus                           |
|                                   | Get_density                   | rename: GetDensity                               |
|                                   | Get_E                         | rename: GetYoungModulus                          |
|                                   | Get_G                         | rename: GetShearModulus                          |
|                                   | Get_l                         | rename: GetLameFirstParam                        |
|                                   | Get_StressStrainMatrix        | rename: GetStressStrainMatrix                    |
|                                   | Get_RayleighDampingK          | rename: GetRayleighDampingBeta                   |
|                                   | Get_RayleighDampingM          | rename: GetRayleighDampingAlpha                  |
|                                   | Get_WaveModulus               | rename: GetPWaveModulus                          |
|                                   | Get_v                         | rename: GetPoissonRatio                          |
|                                   | Set_density                   | rename: SetDensity                               |
|                                   | Set_E                         | rename: SetYoungModulus                          |
|                                   | Set_G                         | rename: SetShearModulus                          |
|                                   | Set_RayleighDampingK          | rename: SetRayleighDampingBeta                   |
|                                   | Set_RayleighDampingM          | rename: SetRayleighDampingAlpha                  |
|                                   | Set_v                         | rename: SetPoissonRatio                          |
| ChContinuumDruckerPrager          |                               |                                                  |
|                                   | Get_alpha                     | rename: GetInternalFriction                      |
|                                   | Get_dilatancy                 | rename: GetDilatancy                             |
|                                   | Get_hardening_limit           | rename: GetHardeningLimit                        |
|                                   | Get_hardening_speed           | rename: GetHardeningSpeed                        |
|                                   | Set_alpha                     | rename: SetInternalFriction                      |
|                                   | Set_dilatancy                 | rename: SetDilatancy                             |
|                                   | Set_from_MohrCoulomb          | rename: SetFromMohrCoulomb                       |
|                                   | Set_hardening_limit           | rename: SetHardeningLimit                        |
|                                   | Set_hardening_speed           | rename: SetHardeningSpeed                        |
| ChContinuumElastoplastic          |                               |                                                  |
|                                   | ComputeYeldFunction           | rename: ComputeYieldFunction                     |
|                                   | Get_flow_rate                 | rename: GetPlasticFlowRate                       |
|                                   | Set_flow_rate                 | rename: SetPlasticFlowRate                       |
| ChContinuumElectrostatics         |                               |                                                  |
|                                   | Get_PermittivityEmatrix       | rename: GetPermittivityMatrix                    |
| ChContinuumMaterial               |                               |                                                  |
|                                   | Get_density                   | rename: GetDensity                               |
|                                   | Set_density                   | rename: SetDensity                               |
| ChContinuumPlasticVonMises        |                               |                                                  |
|                                   | Get_elastic_yeld              | rename: GetElasticYield                          |
|                                   | Get_plastic_yeld              | rename: GetPlasticYield                          |
|                                   | Set_elastic_yeld              | rename: SetElasticYield                          |
|                                   | Set_plastic_yeld              | rename: SetPlasticYield                          |
| ChContinuumPoisson3D              |                               |                                                  |
|                                   | Get_ConstitutiveMatrix        | rename: GetConstitutiveMatrix                    |
| ChContinuumThermal                |                               |                                                  |
|                                   | GetThermalConductivityK       | rename: GetThermalConductivity                   |
|                                   | GetMassSpecificHeatCapacity   | rename: GetSpecificHeatCapacity                  |
|                                   | Get_ThermalKmatrix            | rename: GetConductivityMatrix                    |
|                                   | SetThermalConductivityK       | rename: SetThermalConductivity                   |
|                                   | SetMassSpecificHeatCapacity   | rename: SetSpecificHeatCapacity                  |
| ChControllerPID                   |                               |                                                  |
|                                   | Get_Out                       | rename: GetOutput                                |
|                                   | Get_Pcomp                     | remove                                           |
|                                   | Get_Icomp                     | remove                                           |
|                                   | Get_Dcomp                     | remove                                           |
|                                   | Get_In_int                    | remove                                           |
|                                   | Get_In_dt                     | remove                                           |
|                                   | Get_In                        | remove                                           |
| ChElasticityCosseratAdvancedGenericFPM |                          |                                                  |
|                                   | GetEMatrix                    | rename: GetStiffnessMatrix                       |
|                                   | SetEMatrix                    | rename: SetStiffnessMatrix                       |
|                                   | UpdateEMatrix                 | rename: UpdateStiffnessMatrix                    |
| ChElasticityKirchhoffIsothropic   |                               |                                                  |
|                                   | Get_nu                        | rename: GetPoissonRatio                          |
| ChElasticityKirchhoffOrthotropic  |                               |                                                  |
|                                   | Get_E_x                       | rename: GetYoungModulusX                         |
|                                   | Get_E_y                       | rename: GetYoungModulusY                         |
|                                   | Get_G_xy                      | rename: GetShearModulusXY                        |
|                                   | Get_nu_xy                     | rename: GetPoissonRatioXY                        |
|                                   | Get_nu_yx                     | rename: GetPoissonRatioYX                        |
| ChElasticityReissnerIsothropic    |                               |                                                  |
|                                   | Get_alpha                     | rename: GetShearFactor                           |
|                                   | Get_beta                      | rename: GetTorqueFactor                          |
| ChElasticityReissnerOrthotropic   |                               |                                                  |
|                                   | Get_G_xz                      | rename: GetShearModulusXZ                        |
|                                   | Get_G_yz                      | rename: GetShearModulusYZ                        |
| ChElementBar                      |                               |                                                  |
|                                   | GetBarArea                    | rename: GetArea                                  |
|                                   | GetBarDensity                 | rename: GetDensity                               |
|                                   | GetBarRayleighDamping         | rename: GetRayleighDamping                       |
|                                   | GetBarYoungModulus            | rename: GetYoungModulus                          |
|                                   | SetBarArea                    | rename: SetArea                                  |
|                                   | SetBarDensity                 | rename: SetDensity                               |
|                                   | SetBarRayleighDamping         | rename: SetRayleighDamping                       |
|                                   | SetBarYoungModulus            | rename: SetYoungModulus                          |
| ChElementBase                     |                               |                                                  |
|                                   | GetNdofs                      | rename: GetNumCoordsPosLevel                     |
|                                   | GetNdofs_active               | rename: GetNumCoordsPosLevelActive               |
|                                   | GetNodeN                      | rename: GetNode                                  |
|                                   | GetNodeNdofs                  | rename: GetNodeNumCoordsPosLevel                 |
|                                   | GetNodeNdofs_active           | rename: GetNodeNumCoordsPosLevelActive           |
|                                   | GetNnodes                     | rename: GetNumNodes                              |
|                                   | InjectKRMmatrices             | rename: InjectKRMMatrices                        |
|                                   | KRMmatricesLoad               | rename: LoadKRMMatrices                          |
| ChElementBeamEuler                |                               |                                                  |
|                                   | GetField_dt                   | rename: GetFieldDt                               |
|                                   | GetField_dtdt                 | rename: GetFieldDt2                              |
| ChElementShellANCF                |                               |                                                  |
|                                   | Get_rho                       | rename: GetDensity                               |
| ChElementShellBST                 |                               |                                                  |
|                                   | Get_theta                     | rename: GetFiberAngle                            |
|                                   | Get_thickness                 | rename: GetThickness                             |
|                                   | GetNodeNeighbourN             | rename: GetNodeNeighbour                         |
|                                   | GetNodeTriangleN              | rename: GetNodeMainTriangle                      |
| ChElementSpring                   |                               |                                                  |
|                                   | GetDamperR                    | rename: GetDampingCoefficient                    |
|                                   | GetSpringK                    | rename: GetSpringCoefficient                     |
|                                   | SetDamperR                    | rename: SetDampingCoefficient                    |
|                                   | SetSpringK                    | rename: SetSpringCoefficient                     |
| ChException                       |                               | remove                                           |
| ChFrame                           |                               |                                                  |
|                                   | GetA                          | rename: GetRotMat                                |
|                                   | GetCoord                      | rename: GetCoordsys                              |
|                                   | operator%=                    | remove                                           |
|                                   | SetCoord                      | rename: SetCoordsys                              |
|                                   | TransformLocalToParent        | see Notes                                        |
|                                   | TransformParentToLocal        | see Notes                                        |
| ChFrameMoving                     |                               |                                                  |
|                                   | Compute_Adt                   | rename: ComputeRotMatDt                          |
|                                   | Compute_Adtdt                 | rename: ComputeRotMatDt2                         |
|                                   | GetA_dt                       | rename: GetRotMatDt                              |
|                                   | GetA_dtdt                     | rename: GetRotMatDt2                             |
|                                   | GetCoord_dt                   | rename: GetCoordsysDt                            |
|                                   | GetCoord_dtdt                 | rename: GetCoordsysDt2                           |
|                                   | GetPos_dt                     | rename: GetPosDt                                 |
|                                   |                               | added: GetLinVel                                 |
|                                   | GetPos_dtdt                   | rename: GetPosDt2                                |
|                                   |                               | added: GetLinAcc                                 |
|                                   | GetRot_dt                     | rename: GetRotDt                                 |
|                                   | GetRot_dtdt                   | rename: GetRotDt2                                |
|                                   | GetWacc_loc                   | rename: GetAngAccLocal                           |
|                                   | GetWacc_par                   | rename: GetAngAccParent                          |
|                                   | GetWvel_loc                   | rename: GetAngVelLocal                           |
|                                   | GetWvel_par                   | rename: GetAngVelParent                          |
|                                   | operator%=                    | remove                                           |
|                                   | SetCoord_dt                   | rename: SetCoordsysDt                            |
|                                   | SetCoord_dtdt                 | rename: SetCoordsysDt2                           |
|                                   | SetPos_dt                     | rename: SetPosDt                                 |
|                                   |                               | added: SetLinVel                                 |
|                                   | SetPos_dtdt                   | rename: SetPosDt2                                |
|                                   |                               | added: SetLinAcc                                 |
|                                   | SetRot_dt                     | rename: SetRotDt                                 |
|                                   | SetRot_dtdt                   | rename: SetRotDt2                                |
|                                   | SetWacc_loc                   | rename: SetAngAccLocal                           |
|                                   | SetWacc_par                   | rename: SetAngAccParent                          |
|                                   | SetWvel_loc                   | rename: SetAngVelLocal                           |
|                                   | SetWvel_par                   | rename: SetAngVelParent                          |
| ChFunction                        |                               |                                                  |
|                                   | Compute_int                   | rename: GetIntegral                              |
|                                   | Compute_max                   | rename: GetMax                                   |
|                                   | Compute_mean                  | rename: GetMean                                  |
|                                   | Compute_min                   | rename: GetMin                                   |
|                                   | Compute_sqrmean               | rename: GetSquaredMean                           |
|                                   | Estimate_x_range              | remove                                           |
|                                   | Estimate_y_range              | remove: use GetMin, GetMax                       |
|                                   | EvaluateIntervaldN            | rename: SampleUpToDerN                           |
|                                   | Get_Ca_neg                    | rename: GetNegativeAccelerationCoeff             |
|                                   | Get_Ca_pos                    | rename: GetPositiveAccelerationCoeff             |
|                                   | Get_Cv                        | rename: GetVelocityCoeff                         |
|                                   | Get_Type                      | rename: GetType                                  |
|                                   | Get_weight                    | rename: GetWeight                                |
|                                   | Get_y                         | rename: GetVal                                   |
|                                   | Get_y_dN                      | rename: GetDerN                                  |
|                                   | Get_y_dx                      | rename: GetDer                                   |
|                                   | Get_y_dxdx                    | rename: GetDer2                                  |
|                                   | Get_y_dxdxdx                  | rename: GetDer3                                  |
|                                   | FileAsciiPairsSave            | rename: OutputToASCIIFile                        |
|                                   | HandleAccess                  | remove                                           |
|                                   | HandleNumber                  | remove                                           |
|                                   | FilePostscriptPlot            | remove                                           |
| ChFunction_Const                  |                               | rename: ChFunctionConst                          |
|                                   | Get_yconst                    | rename: GetConstant                              |
|                                   | Set_yconst                    | rename: SetConstant                              |
| ChFunction_BSpline                |                               | rename: ChFunctionBSpline                        |
|                                   | Get_Basis_Tool                | rename: GetBasisTool                             |
|                                   | Get_Control_Points            | rename: GetControlPoints                         |
|                                   | Get_Control_Points_Abscissae  | rename: GetControlPointsAbscissae                |
|                                   | Get_Knots                     | rename: GetKnots                                 |
|                                   | Get_Order                     | rename: GetOrder                                 |
|                                   | Recompute_Constrained         | rename: ApplyInterpolationConstraints            |
|                                   | Setup_Data                    | rename: Setup                                    |
| ChFunction_ConstAcc               |                               | rename: ChFunctionConstAcc                       |
|                                   | Get_end                       | rename: GetDuration                              |
|                                   | Get_h                         | rename: GetDisplacement                          |
|                                   | Get_av                        | rename: GetFirstAccelerationEnd                  |
|                                   | Get_aw                        | rename: GetSecondAccelerationStart               |
|                                   | Set_av                        | rename: SetFirstAccelerationEnd                  |
|                                   | Set_avw                       | rename: SetAccelerationReferencePoints           |
|                                   | Set_aw                        | rename: SetSecondAccelerationStart               |
|                                   | Set_end                       | rename: SetDuration                              |
|                                   | Set_h                         | rename: SetDisplacement                          |
| ChFunction_Derive                 |                               | rename: ChFunctionDerivative                     |
|                                   | Get_order                     | rename: GetOrder                                 |
|                                   | Set_fa                        | rename: SetOperandFunction                       |
|                                   | Get_fa                        | rename: GetOperandFunction                       |
|                                   | Set_order                     | rename: SetOrder                                 |
| ChFunction_DoubleS                |                               | rename: ChFunctionConstJerk                      |
|                                   | Get_Bounds                    | rename: GetBoundaryConditions                    |
|                                   | Get_Constraints               | rename: GetImposedLimits                         |
|                                   | Get_Limits                    | rename: GetReachedLimits                         |
|                                   | Get_Times                     | rename: GetTimes                                 |
| ChFunction_Cycloidal              |                               | rename: ChFunctionCycloidal                      |
|                                   | Set_end                       | rename: SetWidth                                 |
|                                   | Set_h                         | rename: SetHeight                                |
|                                   | Get_end                       | rename: GetWidth                                 |
|                                   | Get_h                         | rename: GetHeight                                |
| ChFunction_Fillet3                |                               | rename: ChFunctionFillet3                        |
|                                   | Get_dy1                       | rename: GetStartDer                              |
|                                   | Get_dy2                       | rename: GetEndDer                                |
|                                   | Get_y1                        | rename: GetStartVal                              |
|                                   | Get_y2                        | rename: GetEndVal                                |
|                                   | Set_dy1                       | rename: SetStartDer                              |
|                                   | Set_dy2                       | rename: SetEndDer                                |
|                                   | Set_y1                        | rename: SetStartVal                              |
|                                   | Set_y2                        | rename: SetEndVal                                |
|                                   | SetupCoefficients             | rename: Setup                                    |
| ChFunction_Integrate              |                               | rename: ChFunctionIntegral                       |
|                                   | ComputeIntegral               | rename: Setup                                    |
|                                   | Get_C_start                   | rename: GetOffsetVal                             |
|                                   | Get_num_samples               | rename: GetNumSamples                            |
|                                   | Get_x_end                     | rename: GetStart                                 |
|                                   | Get_x_start                   | rename: GetEnd                                   |
|                                   | Set_C_start                   | rename: SetOffsetVal                             |
|                                   | Set_num_samples               | rename: SetNumSamples                            |
|                                   | Set_x_end                     | rename: SetEndArg                                |
|                                   | Set_x_start                   | rename: SetStartArg                              |
| ChFunction_Lambda                 |                               | rename: ChFunctionLambda                         |
| ChFunction_Mirror                 |                               | rename: ChFunctionMirror                         |
|                                   | Get_mirror_axis               | rename: GetMirrorAxis                            |  
|                                   | Set_mirror_axis               | rename: SetMirrorAxis                            |  
| ChFunction_Mocap                  |                               | remove                                           |
| ChFunction_Noise                  |                               | remove                                           |
| ChFunction_Operation              |                               | rename: ChFunctionOperator                       |
|                                   | Get_fa                        | rename: GetFirstOperandFunction                  |
|                                   | Get_fb                        | rename: GetSecondOperandFunction                 |
|                                   | Get_optype                    | rename: GetOperationType                         |
|                                   | Set_fa                        | rename: SetFirstOperandFunction                  |
|                                   | Set_fb                        | rename: SetSecondOperandFunction                 |
|                                   | Set_optype                    | rename: SetOperationType                         |
| ChFunction_Oscilloscope           |                               | remove                                           |
| ChFunction_Poly                   |                               | rename: ChFunctionPoly                           |
|                                   | Get_coeff                     | rename: GetCoefficients                          |
|                                   | Set_coeff                     | rename: SetCoefficients                          |
| ChFunction_Poly345                |                               | rename: ChFunctionPoly345                        |
|                                   | Set_end                       | rename: SetWidth                                 |
|                                   | Set_h                         | rename: SetHeight                                |
|                                   | Get_end                       | rename: GetWidth                                 |
|                                   | Get_h                         | rename: GetHeight                                |
| ChFunction_Ramp                   |                               | rename: ChFunctionRamp                           |
|                                   | Set_ang                       | rename: SetAngularCoeff                          |
|                                   | Set_y0                        | rename: SetStartVal                              |
|                                   | Get_ang                       | rename: GetAngularCoeff                          |
|                                   | Get_y0                        | rename: GetStartVal                              |
| ChFunction_Recorder               |                               | rename: ChFunctionInterp                         |
|                                   | GetPoints                     | rename: GetTable                                 |
| ChFunction_Repeat                 |                               | rename: ChFunctionRepeat                         |
|                                   | Get_window_length             | rename: GetSliceLength                           |
|                                   | Get_window_phase              | rename: GetSliceShift                            |
|                                   | Get_window_start              | rename: GetSliceStart                            |
|                                   | Set_window_length             | rename: SetSliceWidth                            |
|                                   | Set_window_phase              | rename: SetSliceShift                            |
|                                   | Set_window_start              | rename: SetSliceStart                            |
| ChFunction_Sequence               |                               | rename: ChFunctionSequence                       |
|                                   | Get_list                      | rename: GetFunctions                             |
|                                   | GetNthDuration                | rename: GetDuration                              |
|                                   | GetNthFunction                | rename: GetFunction                              |
|                                   | GetNthNode                    | rename: GetNode                                  |
|                                   | KillFunct                     | rename: RemoveFunct                              |
| ChFunction_Sigma                  |                               | rename: ChFunctionPoly23                         |
|                                   | Get_amp                       | rename: GetAmplitude                             |
|                                   | Set_amp                       | rename: SetAmplitude                             |
| ChFunction_Sine                   |                               | rename: ChFunctionSine                           |
|                                   | (constructor)                 | changed args order to ampl, freq, phase          |
|                                   | Get_phase                     | rename: GetPhase                                 |
|                                   | Set_phase                     | rename: SetPhase                                 |
|                                   | SetFreq                       | rename: SetFrequency                             |
|                                   | Get_w                         | rename: GetAngularRate                           |
|                                   | Set_freq                      | rename: SetFrequency                             |
|                                   | Set_w                         | rename: SetAngularRate                           |
| ChFunction_SineStep               | SetP1                         | rename: SetFirstPoint                            |
|                                   | SetP2                         | rename: SetSecondPoint                           |
| ChFunction_Setpoint               |                               | rename: ChFunctionSetpoint                       |
| ChFunctionPosition                |                               |                                                  |
|                                   | Get_p                         | rename: GetPos                                   |
|                                   | Get_p_ds                      | rename: GetLinVel                                |
|                                   | Get_p_dsds                    | rename: GetLinAcc                                |
| ChFunctionPosition                |                               |                                                  |
|                                   | Get_q                         | rename: GetQuat                                  |
|                                   | Get_w_loc                     | rename: GetAngVel                                |
|                                   | Get_a_loc                     | rename: GetAngAcc                                |
| ChFunctionPosition_line           |                               | rename: ChFunctionPositionLine                   |
| ChFunctionPosition_setpoint       |                               | rename: ChFunctionPositionSetpoint               |
| ChFunctionPosition_XYZfunctions   |                               | rename: ChFunctionPositionXYZFunctions           |
| ChFunctionRotation_ABCfunctions   |                               | rename: ChFunctionRotationABCFunctions           |
|                                   | SetAngleset                   | rename: SetRotationRepresentation                |
|                                   | GetAngleset                   | rename: GetRotationRepresentation                |
| ChFunctionRotation_axis           |                               | rename: ChFunctionRotationAxis                   |
| ChFunctionRotation_setpoint       |                               | rename: ChFunctionRotationSetpoint               |
| ChFunctionRotation_spline         |                               | rename: ChFunctionRotationBSpline                |
| ChFunctionRotation_SQUAD          |                               | rename: ChFunctionRotationSQUAD                  |
| ChImplicitTimestepper             |                               |                                                  |
|                                   | GetMaxiters                   | rename: GetMaxIters                              |
|                                   | SetMaxiters                   | rename: SetMaxIters                              |
| ChIndexedParticles                |                               |                                                  |
|                                   | GetNparticles                 | rename: GetNumParticles                          |
|                                   | GetParticle                   | rename: Particle                                 |
| ChIntegrable                      |                               |                                                  |
|                                   | GetNconstr                    | rename: GetNumConstraints                        |
|                                   | GetNcoords_dy                 | remove: split in GetNumCoordsVelLevel/Acc        |
|                                   | GetNcoords_x                  | rename: GetNumCoordsPosLevel                     |
|                                   | GetNcoords_v                  | rename: GetNumCoordsVelLevel                     |
|                                   | GetNcoords_y                  | remove: split in GetNumCoordsPosLevel/Vel        |
| ChIntegrable1D                    |                               | rename: ChIntegrand1D                            |
| ChIntegrable2D                    |                               | rename: ChIntegrand2D                            |
| ChIntegrable3D                    |                               | rename: ChIntegrand3D                            |
| ChIterativeSolver                 |                               |                                                  |
|                                   | SaveMatrix                    | rename: WriteMatrices                            |
| ChKblock                          |                               | remove                                           |
| ChKblockGeneric                   |                               | rename: ChKRMBlock                               |
|                                   | Build_K                       | rename: PasteMatrixInto (see Notes)              |
|                                   | Get_K                         | rename: GetMatrix                                |
|                                   | GetVariableN                  | rename: GetVariable                              |
|                                   | MultiplyAndAdd                | rename: AddMatrixTimesVectorInto                 |
| ChLine                            |                               |                                                  |
|                                   | Get_closed                    | rename: IsClosed                                 |
|                                   | Get_complexity                | rename: GetComplexity                            |
|                                   | Set_closed                    | rename: SetClosed                                |
|                                   | Set_complexity                | rename: SetComplexity                            |
| ChLineBspline                     |                               | rename: ChLineBSpline                            |
| ChLineCam                         |                               |                                                  |
|                                   | Get_b0                        | rename: GetFollowerInitPhase                     |
|                                   | Get_center                    | rename: GetCenter                                |
|                                   | Get_d                         | rename: GetFollowerDistance                      |
|                                   | Get_e                         | remove                                           |
|                                   | Get_internal                  | rename: IsInternal                               |
|                                   | Get_motion_law                | rename: GetMotionLaw                             |
|                                   | Get_negative                  | rename: IsNegative                               |
|                                   | Get_Phase                     | rename: GetPhase                                 |
|                                   | Get_Rb                        | rename: GetCamRadius                             |
|                                   | Get_Rr                        | rename: GetWheelRadius                           |
|                                   | Get_s                         | remove                                           |
|                                   | Get_type                      | rename: GetCamType                               |
|                                   | Set_center                    | rename: SetCenter                                |
|                                   | Set_flat_oscillate            | rename: SetFlatOscillate                         |
|                                   | Set_internal                  | rename: SetInternal                              |
|                                   | Set_motion_law                | rename: SetMotionLaw                             |
|                                   | Set_negative                  | rename: SetNegative                              |
|                                   | Set_Phase                     | rename: SetPhase                                 |
|                                   | Set_Rb                        | rename: SetCamRadius                             |
|                                   | Set_Rr                        | rename: SetWheelRadius                           |
|                                   | Set_rotating_follower         | rename: SetRotatingFollower                      |
|                                   | Set_sliding_eccentrical       | rename: SetSlidingEccentrical                    |
|                                   | Set_type                      | rename: SetCamType                               |
| ChLinePoly                        |                               |                                                  |
|                                   | Get_degree                    | rename: GetDegree                                |
|                                   | Get_numpoints                 | rename: GetNumPoints                             |
|                                   | Set_point                     | rename: SetPoint                                 |
| ChLink                            |                               |                                                  |
|                                   | GetLeftDOF                    | remove                                           |
|                                   | GetLinkRelativeCoords         | remove (see Notes)                               |
|                                   |                               | add: GetFrame1Rel                                |
|                                   |                               | add: GetFrame2Rel                                |
|                                   | ResetRedundant                | remove                                           |
| ChLinkBase                        |                               |                                                  |
|                                   | GetLinkAbsoluteCoords         | remove                                           |
|                                   |                               | add: ChFrameAbs1                                 |
|                                   |                               | add: ChFrameAbs2                                 |
|                                   | GetNumCoords                  | rename: GetNumAffectedCoords                     |
|                                   | Get_react_force               | remove                                           |
|                                   | Get_react_torque              | remove                                           |
|                                   |                               | add: GetReaction1 (see Notes)                    |
|                                   |                               | add: GetReaction2 (see Notes)                    |
| ChLinkBeamIGAslider               |                               | rename: ChLinkBeamIGAFrame                       |
| ChLinkBrake                       |                               | rename: ChLinkLockBrake                          |
| ChLinkClearance                   |                               | rename: ChLinkLockClearance                      |
| ChLinkDirFrame                    |                               | rename: ChLinkNodeSlopeFrame                     |
| ChLinkForce                       |                               |                                                  |
|                                   | GetF                          | rename: GetActuatorForceTorque                   |
|                                   | GetFcurrent                   | rename: GetCurrentActuatorForceTorque            |
|                                   | GetForce                      | rename: GetForceTorque                           |
|                                   | GetK                          | rename: GetSpringCoefficient                     |
|                                   | GetKcurrent                   | rename: GetCurrentSpringCoefficient              |
|                                   | GetModulationActuator         | rename: GetActuatorModulation                    |
|                                   | GetModulationK                | rename: GetSpringModulation                      |
|                                   | GetModulationR                | rename: GetDamperModulation                      |
|                                   | GetR                          | rename: GetDampingCoefficient                    |
|                                   | GetRcurrent                   | rename: GetCurrentDampingCoefficient             |
|                                   | SetF                          | rename: SetActuatorForceTorque                   |
|                                   | SetModulationF                | rename: SetActuatorModulation                    |
|                                   | SetModulationK                | rename: SetSpringModulation                      |
|                                   | SetModulationR                | rename: SetDamperModulation                      |
| ChLinkGear                        |                               | rename: ChLinkLockGear                           |
| ChLinkLimit                       |                               |                                                  |
|                                   | GetKmax                       | rename: GetSpringCoefficientMax                  |
|                                   | GetKmin                       | rename: GetSpringCoefficientMin                  |
|                                   | GetMaxPolarAngle              | rename: GetPolarAngleMax                         |
|                                   | GetModulationKmax             | rename: GetSpringModulationMax                   |
|                                   | GetModulationKmin             | rename: GetSpringModulationMin                   |
|                                   | GetModulationRmax             | rename: GetDamperModulationMax                   |
|                                   | GetModulationRmin             | rename: GetDamperModulationMin                   |
|                                   | GetPolarMax                   | rename: GetPolarAngleModulationMax               |
|                                   | GetRmax                       | rename: GetDampingCoefficientMax                 |
|                                   | GetRmin                       | rename: GetDampingCoefficientMin                 |
|                                   | SetKmax                       | rename: SetSpringCoefficientMax                  |
|                                   | SetKmin                       | rename: SetSpringCoefficientMin                  |
|                                   | SetModulationKmax             | rename: SetSpringModulationMax                   |
|                                   | SetModulationKmin             | rename: SetSpringModulationMin                   |
|                                   | SetModulationRmax             | rename: SetDamperModulationMax                   |
|                                   | SetModulationRmin             | rename: SetDamperModulationMin                   |
|                                   | SetPolarMax                   | rename: SetPolarAngleModulationMax               |
|                                   | SetRmax                       | rename: SetDampingCoefficientMax                 |
|                                   | SetRmin                       | rename: SetDampingCoefficientMin                 |
| ChLinkLinActuator                 |                               | rename: ChLinkLockLinActuator                    |
| ChLinkLock                        |                               |                                                  |
|                                   | ChangeLinkType                | rename: ChangeType                               |
|                                   | GetConstraintViolation_dt     | rename: GetConstraintViolationDt                 |
|                                   | GetConstraintViolation_dtdt   | rename: GetConstraintViolationDt2                |
|                                   | GetForce_D                    | rename: ForceD                                   |
|                                   | GetForce_R                    | rename: ForceRp                                  |
|                                   | GetForce_Rx                   | rename: ForceRx                                  |
|                                   | GetForce_Ry                   | rename: ForceRy                                  |
|                                   | GetForce_Rz                   | rename: ForceRz                                  |
|                                   | GetForce_X                    | rename: ForceX                                   |
|                                   | GetForce_Y                    | rename: ForceY                                   |
|                                   | GetForce_Z                    | rename: ForceZ                                   |
|                                   | GetLimit_D                    | rename: LimitD                                   |
|                                   | GetLimit_Rp                   | rename: LimitRp                                  |
|                                   | GetLimit_Rx                   | rename: LimitRx                                  |
|                                   | GetLimit_Ry                   | rename: LimitRy                                  |
|                                   | GetLimit_Rz                   | rename: LimitRz                                  |
|                                   | GetLimit_X                    | rename: LimitX                                   |
|                                   | GetLimit_Y                    | rename: LimitY                                   |
|                                   | GetLimit_Z                    | rename: LimitZ                                   |
|                                   | GetRelM                       | rename: GetRelCoordsys                           |
|                                   | GetRelM_dt                    | rename: GetRelCoordsysDt                         |
|                                   | GetRelM_dtdt                  | rename: GetRelCoordsysDt2                        |
|                                   | SetUpMarkers                  | rename: SetupMarkers                             |
| ChLinkLockBrake                   |                               |                                                  |
|                                   | Get_brake_mode                | rename: GetBrakeMode                             |
|                                   | Get_brake_torque              | rename: GetBrakeTorque                           |
|                                   | Get_stick_ratio               | rename: GetStickingCoeff                         |
|                                   | Set_brake_mode                | rename: SetBrakeMode                             |
|                                   | Set_brake_torque              | rename: SetBrakeTorque                           |
|                                   | Set_stick_ratio               | rename: SetStickingCoeff                         |
| ChLinkLockClearance               |                               |                                                  |
|                                   | Get_axis_eccentricity         | rename: GetEccentricity                          |
|                                   | Get_axis_phase                | rename: GetAxisAngularLocation                   |
|                                   | Get_c_friction                | rename: GetFriction                              |
|                                   | Get_c_restitution             | rename: GetRestitution                           |
|                                   | Get_c_tang_restitution        | remove                                           |
|                                   | Get_c_viscous                 | remove                                           |
|                                   | Get_clearance                 | rename: GetClearance                             |
|                                   | Get_contact_F_abs             | rename: GetContactForceAbs                       |
|                                   | Get_contact_F_n               | rename: GetContactForceNormal                    |
|                                   | Get_contact_F_t               | rename: GetContactForceTangential                |
|                                   | Get_contact_N_abs             | rename: GetContactNormalAbs                      |
|                                   | Get_contact_P_abs             | rename: GetContactPosAbs                         |
|                                   | Get_contact_V_t               | rename: GetContactSpeedTangential                |
|                                   | Get_diameter                  | rename: GetDiameter                              |
|                                   | Get_is_in_contact             | remove                                           |
|                                   | Get_rotation_angle            | rename: GetRotationAngle                         |
|                                   | GetC_force                    | rename: GetAccumulatedForce                      |
|                                   | GetC_torque                   | rename: GetAccumulatedTorque                     |
|                                   | GetRelRotaxis                 | rename: GetRelAngleAxis                          |
|                                   | Set_c_friction                | rename: SetFriction                              |
|                                   | Set_c_restitution             | rename: SetRestitution                           |
|                                   | Set_c_tang_restitution        | remove                                           |
|                                   | Set_c_viscous                 | remove                                           |
|                                   | Set_clearance                 | rename: SetClearance                             |
|                                   | Set_diameter                  | rename: SetDiameter                              |
| ChLinkLockLock                    |                               |                                                  |
|                                   | GetMotion_ang                 | rename: GetMotionAng1                            |
|                                   | GetMotion_ang2                | rename: GetMotionAng2                            |
|                                   | GetMotion_ang3                | rename: GetMotionAng3                            |
|                                   | GetMotion_axis                | rename: GetMotionAxis                            |
|                                   | GetMotion_X                   | rename: GetMotionX                               |
|                                   | GetMotion_Y                   | rename: GetMotionY                               |
|                                   | GetMotion_Z                   | rename: GetMotionZ                               |
|                                   | GetRelC                       | rename: GetRelCoordsysViolation                  |
|                                   | GetRelC_dt                    | rename: GetRelCoordsysViolationDt                |
|                                   | GetRelC_dtdt                  | rename: GetRelCoordsysViolationDt2               |
|                                   | SetMotion_ang                 | rename: SetMotionAng1                            |
|                                   | SetMotion_ang2                | rename: SetMotionAng2                            |
|                                   | SetMotion_ang3                | rename: SetMotionAng3                            |
|                                   | SetMotion_axis                | rename: SetMotionAxis                            |
|                                   | SetMotion_X                   | rename: SetMotionX                               |
|                                   | SetMotion_Y                   | rename: SetMotionY                               |
|                                   | SetMotion_Z                   | rename: SetMotionZ                               |
| ChLinkLockGear                    |                               |                                                  |
|                                   | Get_a1                        | rename: GetRotation1                             |
|                                   | Get_a2                        | rename: GetRotation2                             |
|                                   | Get_alpha                     | rename: GetPressureAngle                         |
|                                   | Get_beta                      | rename: GetPitchAngle                            |
|                                   | Get_checkphase                | rename: GetEnforcePhase                          |
|                                   | Get_epicyclic                 | rename: GetEpicyclic                             |
|                                   | Get_local_shaft1              | rename: GetFrameShaft1                           |
|                                   | Get_local_shaft2              | rename: GetFrameShaft2                           |
|                                   | Get_phase                     | rename: GetPhase                                 |
|                                   | Get_r1                        | rename: GetRadius1                               |
|                                   | Get_r2                        | rename: GetRadius2                               |
|                                   | Get_shaft_dir1                | rename: GetDirShaft1                             |
|                                   | Get_shaft_dir2                | rename: GetDirShaft2                             |
|                                   | Get_shaft_pos1                | rename: GetPosShaft1                             |
|                                   | Get_shaft_pos2                | rename: GetPosShaft2                             |
|                                   | Get_tau                       | rename: GetTransmissionRatio                     |
|                                   | Reset_a1a2                    | rename: ResetRotations                           |
|                                   | Set_alpha                     | rename: SetPressureAngle                         |
|                                   | Set_beta                      | rename: SetPitchAngle                            |
|                                   | Set_checkphase                | rename: SetEnforcePhase                          |
|                                   | Set_local_shaft1              | rename: SetFrameShaft1                           |
|                                   | Set_local_shaft2              | rename: SetFrameShaft2                           |
|                                   | Set_epicyclic                 | rename: SetEpicyclic                             |
|                                   | Set_tau                       | rename: SetTransmissionRatio                     |
| ChLinkLockPointSpline             |                               |                                                  |
|                                   | Get_trajectory_line           | rename: GetTrajectory                            |
|                                   | Set_trajectory_line           | rename: SetTrajectory                            |
| ChLinkLockPulley                  |                               |                                                  |
|                                   | Get_belt_up1                  | rename: GetBeltUpPos1                            |
|                                   | Get_belt_up2                  | rename: GetBeltUpPos2                            |
|                                   | Get_belt_low1                 | rename: GetBeltBottomPos1                        |
|                                   | Get_belt_low2                 | rename: GetBeltBottomPos2                        |
|                                   | Set_r1                        | rename: SetRadius1                               |
|                                   | Set_r2                        | rename: SetRadius2                               |
| ChLinkLockScrew                   |                               |                                                  |
|                                   | Get_thread                    | rename: GetThread                                |
|                                   | Set_thread                    | rename: SetThread                                |
| ChLinkLockTrajectory              |                               |                                                  |
|                                   | Get_space_fx                  | rename: GetTimeLaw                               |
|                                   | Set_space_fx                  | rename: SetTimeLaw                               |
|                                   | Set_modulo_one_fx             | rename: WrapTimeLaw                              |
| ChLinkMask                        |                               |                                                  |
|                                   | Constr_N                      | rename: GetConstraint                            |
|                                   | GetActiveConstrByNum          | rename: GetActiveConstraint                      |
|                                   | GetMaskDoc                    | rename: GetNumConstraintsActive                  |
|                                   | GetMaskDoc_c                  | rename: GetNumConstraintsBilateralActive         |
|                                   | GetMaskDoc_d                  | rename: GetNumConstraintsUnilateralActive        |
|                                   | nconstr                       | rename: GetNumConstraints                        |
|                                   | ResetNconstr                  | rename: SetNumConstraints                        |
|                                   | ResetRedundant                | remove                                           |
|                                   | SetActiveRedundantByArray     | remove                                           |
| ChLinkMarkers                     |                               |                                                  |
|                                   | GetDist                       | rename: GetDistance                              |
|                                   | GetDist_dt                    | rename: GetDistanceDt                            |
|                                   | GetRelWvel                    | rename: GetRelativeAngVel                        |
|                                   | GetRelWacc                    | rename: GetRelativeAngAcc                        |
| ChLinkMate                        |                               |                                                  |
|                                   | RestoreRedundant              | rename: ResetRedundant                           |
| ChLinkMateCoaxial                 |                               | rename: ChLinkMateCylindrical                    |
| ChLinkMateGeneric                 |                               |                                                  |
|                                   | GetSeparation                 | rename: GetDistance                              |
| ChLinkMatePlane                   |                               | rename: ChLinkMatePlanar                         |
|                                   | GetLagrangeMultiplier_f       | remove                                           |
|                                   | GetLagrangeMultiplier_m       | remove                                           |
| ChLinkMatePrismatic               |                               |                                                  |
|                                   | GetRelativePos_dt             | rename: GetRelativePosDt                         |
|                                   | GetRelativePos_dtdt           | rename: GetRelativePosDt2                        |
| ChLinkMateRevolute                |                               |                                                  |
|                                   | GetRelativeAngle_dt           | rename: GetRelativeAngleDt                       |
|                                   | GetRelativeAngle_dtdt         | rename: GetRelativeAngleDt2                      |
| ChLinkMateXdistance               |                               | rename: ChLinkMateDistanceZ                      |
| ChLinkMotorLinearDriveline        |                               |                                                  |
|                                   | GetInnerShaft1lin             | rename: GetInnerShaft1Lin                        |
|                                   | GetInnerShaft2lin             | rename: GetInnerShaft2Lin                        |
|                                   | GetInnerShaft2rot             | rename: GetInnerShaft2Rot                        |
| ChLinkMotorPosition               |                               |                                                  |
|                                   | GetMotorPos_dt                | rename: GetMotorPosDt                            |
|                                   | GetMotorPos_dtdt              | rename: GetMotorPosDt2                           |
| ChLinkMotorRotation               |                               |                                                  |
|                                   | GetMotorRot                   | rename: GetMotorAngle                            |
|                                   | GetMotorRot_dt                | rename: GetMotorAngleDt                          |
|                                   | GetMotorRot_dtdt              | rename: GetMotorAngleDt2                         |
|                                   | GetMotorRotPeriodic           | rename: GetMotorAngleWrapped                     |
|                                   | GetMotorRotTurns              | rename: GetMotorNumTurns                         |
| ChLinkMotorRotationSpeed          |                               |                                                  |
|                                   | GetAvoidAngleDrift            | remove                                           |
|                                   | SetAvoidAngleDrift            | rename: AvoidAngleDrift                          |
| ChLinkPointFrame                  |                               | rename: ChLinkNodeFrame                          |
| ChLinkPointPoint                  |                               | rename: ChLinkNodeNode                           |
|                                   | GetConstrainedNodeA           | rename: GetNode1                                 |
|                                   | GetConstrainedNodeB           | rename: GetNode2                                 |
| ChLinkPointTriface                |                               | rename: ChLinkNodeFace                           |
|                                   | GetConstrainedNodeA           | rename: GetNode                                  |
|                                   | GetConstrainedTriangle        | rename: GetTriangle                              |
| ChLinkPointTrifaceRot             |                               | rename: ChLinkNodeFaceRot                        |
|                                   | GetConstrainedNodeA           | rename: GetNode                                  |
|                                   | GetConstrainedTriangle        | rename: GetTriangle                              |
| ChLinkPointSpline                 |                               | rename: ChLinkLockPointSpline                    |
| ChLinkPulley                      |                               | rename: ChLinkLockPulley                         |
| ChLinkRackpinion                  |                               | rename: ChLinkMateRackPinion                     |
|                                   | GetAlpha                      | rename: GetPressureAngle                         |
|                                   | GetBeta                       | rename: GetPitchAngle                            |
|                                   | GetCheckphase                 | rename: GetEnforcePhase                          |
|                                   | SetAlpha                      | rename: SetPressureAngle                         |
|                                   | SetBeta                       | rename: SetPitchAngle                            |
|                                   | SetCheckphase                 | rename: SetEnforcePhase                          |
|                                   | Reset_a1                      | rename: ResetRotation1                           |
| ChLinkScrew                       |                               | rename: ChLinkLockScrew                          |
| ChLinkTrajectory                  |                               | rename: ChLinkLockTrajectory                     |
| ChList                            |                               | remove                                           |
| ChLoad                            |                               | replaced with non-templated class (see Notes)    |
|                                   | LoadGet_ndof_x                | rename: LoadGetNumCoordsPosLevel                 |
|                                   | LoadGet_ndof_w                | rename: LoadGetNumCoordsVelLevel                 |
| ChLoadable                        |                               |                                                  |
|                                   | Get_field_ncoords             | rename: GetNumFieldCoords                        |
|                                   | GetSubBlocks                  | rename: GetNumSubBlocks                          |
|                                   | LoadableGet_ndof_x            | rename: GetLoadableNumCoordsPosLevel             |
|                                   | LoadableGet_ndof_w            | rename: GetLoadableNumCoordsVelLevel             |
|                                   | LoadableGetStateBlock_x       | rename: LoadableGetStateBlockPosLevel            |
|                                   | LoadableGetStateBlock_w       | rename: LoadableGetStateBlockVelLevel            |
| ChLoadBase                        |                               |                                                  |
|                                   | LoadGet_field_ncoords         | rename: LoadGetNumFieldCoords                    |
| ChLoadBodyBodyBushingPlastic      |                               |                                                  |
|                                   | GetYeld                       | rename: GetYield                                 |
|                                   | SetYeld                       | rename: SetYield                                 |
| ChLoaderGravity                   |                               |                                                  |
|                                   | Get_G_acc                     | rename: GetGravitationalAcceleration             |
|                                   | Set_G_acc                     | rename: SetGravitationalAcceleration             |
| ChLoaderXYZnode                   |                               | rename: ChLoaderNodeXYZ                          |
| ChLoadXYZnode                     |                               | rename: ChLoadNodeXYZ                            |
| ChLoadXYZnodeBody                 |                               | rename: ChLoadNodeXYZBody                        |
|                                   | GetBodyB                      | rename: GetBody                                  |
|                                   | GetNodeA                      | rename: GetNode                                  |
| ChLoadXYZnodeBodyBushing          |                               | rename: ChLoadNodeXYZBodyBushing                 |
| ChLoadXYZnodeBodySpring           |                               | rename: ChLoadNodeXYZBodySpring                  |
| ChLoadXYZnodeForce                |                               | rename: ChLoadNodeXYZForce                       |
| ChLoadXYZnodeForceAbsolute        |                               | rename: ChLoadNodeXYZForceAbs                    |
| ChLoadXYZnodeXYZnode              |                               | rename: ChLoadNodeXYZNodeXYZ                     |
| ChLoadXYZnodeXYZnodeBushing       |                               | rename: ChLoadNodeXYZNodeXYZBushing              |
| ChLoadXYZnodeXYZnodeSpring        |                               | rename: ChLoadNodeXYZNodeXYZSpring               |
| ChLoadXYZROTnodeBody              |                               | rename: ChLoadNodeXYZRotBody                     |
|                                   | GetBodyB                      | rename: GetBody                                  |
|                                   | GetNodeA                      | rename: GetNode                                  |
| ChLoadXYZROTnodeBodyBushingGeneric   |                            | rename: ChLoadNodeXYZRotBodyBushingGeneric       |
| ChLoadXYZROTnodeBodyBushingMate      |                            | rename: ChLoadNodeXYZRotBodyBushingMate          |
| ChLoadXYZROTnodeBodyBushingPlastic   |                            | rename: ChLoadNodeXYZRotBodyBushingPlastic       |
| ChLoadXYZROTnodeBodyBushingSpherical |                            | rename: ChLoadNodeXYZRotBodyBushingSpherical     |
| ChLoadXYZROTnode                  |                               | rename: ChLoadNodeXYZRot                         |
| ChLoadXYZROTnodeForceAbsolute     |                               | rename: ChLoadNodeXYZRotForceAbs                 |
| ChLoadXYZROTnodeXYZROTnode        |                               | rename: ChLoadNodeXYZRotNodeXYZRot               |
| ChLoadXYZROTnodeXYZROTnodeBushingGeneric   |                      | rename: ChLoadNodeXYZRotNodeXYZRotBushingGeneric   |
| ChLoadXYZROTnodeXYZROTnodeBushingMate      |                      | rename: ChLoadNodeXYZRotNodeXYZRotBushingMate      |
| ChLoadXYZROTnodeXYZROTnodeBushingPlastic   |                      | rename: ChLoadNodeXYZRotNodeXYZRotBushingPlastic   |
| ChLoadXYZROTnodeXYZROTnodeBushingSpherical |                      | rename: ChLoadNodeXYZRotNodeXYZRotBushingSpherical |
| ChLog                             |                               | remove                                           |
| ChLogConsole                      |                               | remove                                           |
| ChMarker                          |                               |                                                  |
|                                   | Dir_Ref2World                 | remove                                           |
|                                   | Dir_World2Ref                 | remove                                           |
|                                   | GetAbsWvel                    | rename: GetAngVelAbs                             |
|                                   | GetAbsWacc                    | rename: GetAngAccAbs                             |
|                                   | GetAbsCoord                   | rename: GetAbsCoordsys                           |
|                                   | GetAbsCoord_dt                | rename: GetAbsCoordsysDt                         |
|                                   | GetAbsCoord_dtdt              | rename: GetAbsCoordsysDt2                        |
|                                   | GetMotionAng1                 | rename: GetMotionAngle                           |
|                                   | GetMotion_axis                | rename: GetMotionAxis                            |
|                                   | GetMotionX                    | rename: GetMotionX                               |
|                                   | GetMotionY                    | rename: GetMotionY                               |
|                                   | GetMotionZ                    | rename: GetMotionZ                               |
|                                   | GetRest_Coord                 | rename: GetRestCoordsys                          |
|                                   | Impose_Rel_Coord              | rename: ImposeRelativeTransform                  |
|                                   | Impose_Abs_Coord              | rename: ImposeAbsoluteTransform                  |
|                                   | Point_Ref2World               | remove                                           |
|                                   | Point_World2Ref               | remove                                           |
|                                   | SetAbsCoord                   | rename: SetAbsCoordsys                           |
|                                   | SetAbsCoord_dt                | rename: SetAbsCoordsysDt                         |
|                                   | SetAbsCoord_dtdt              | rename: SetAbsCoordsysDt2                        |
|                                   | SetMotion_ang                 | rename: SetMotionAngle                           |
|                                   | SetMotion_axis                | rename: SetMotionAxis                            |
|                                   | SetMotion_X                   | rename: SetMotionX                               |
|                                   | SetMotion_Y                   | rename: SetMotionY                               |
|                                   | SetMotion_Z                   | rename: SetMotionZ                               |
| ChMaterialSurface                 |                               | rename: ChContactMaterial                        |
|                                   | SetSfriction                  | rename: SetStaticFriction                        |
|                                   | GetSfriction                  | rename: GetStaticFriction                        |
|                                   | SetKfriction                  | rename: SetSlidingFriction                       |
|                                   | GetKfriction                  | rename: GetSlidingFriction                       |
| ChMaterialComposite               |                               | rename: ChContactMaterialComposite               |
| ChMaterialCompositeNSC            |                               | rename: ChContactMaterialCompositeNSC            |
| ChMaterialCompositeSMC            |                               | rename: ChContactMaterialCompositeSMC            |
| ChMaterialCompositionStrategy     |                               | rename: ChContactMaterialCompositionStrategy     |
| ChMatrix33                        |                               |                                                  |
|                                   | Get_A_Cardano                 | rename: GetFromCardanAnglesZXY                   |
|                                   | Get_A_Eulero                  | rename: GetFromEulerAnglesZXZ                    |
|                                   | Get_A_Hpb                     | rename: GetFromCardanAnglesZYX                   |
|                                   | Get_A_quaternion              | rename: GetFromQuaternion                        |
|                                   | Get_A_Rodriguez               | rename: GetFromRodriguesParameters               |
|                                   | Get_A_Rxyz                    | rename: GetFromCardanAnglesXYZ                   |
|                                   | Get_A_Xaxis()                 | rename: GetAxisX()                               |
|                                   | Get_A_Yaxis()                 | rename: GetAxisY()                               |
|                                   | Get_A_Zaxis()                 | rename: GetAxisZ()                               |
|                                   | GetAx()                       | remove                                           |
|                                   | Set_A_axis                    | rename: SetFromDirectionAxes                     |
|                                   | Set_A_Cardano                 | rename: SetFromCardanAnglesZXY                   |
|                                   | Set_A_Eulero                  | rename: SetFromEulerAnglesZXZ                    |
|                                   | Set_A_Hpb                     | rename: SetFromCardanAnglesZYX                   |
|                                   | Set_A_quaternion              | rename: SetFromQuaternion                        |
|                                   | Set_A_Rodriguez               | rename: SetFromRodriguesParameters               |
|                                   | Set_A_Rxyz                    | rename: SetFromCardanAnglesXYZ                   |
|                                   | Set_A_Xdir                    | rename: SetFromAxisX                             |
| ChMesh                            |                               |                                                  |
|                                   | GetNcontactSurfaces           | rename: GetNumContactSurfaces                    |
|                                   | GetNelements                  | rename: GetNumElements                           |
|                                   | GetNmeshSurfaces              | rename: GetNumMeshSurfaces                       |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChMeshSurface                     |                               |                                                  |
|                                   | GetFacesList                  | rename: GetFaces                                 |
| ChMinMaxDistribution              |                               | rename: ChUniformDistribution                    |
| ChModalAssembly                   |                               |                                                  |
|                                   | refer to ChAssembly           | like ChAssembly with boundary/internal suffixes  |
|                                   | ComputeModalKRMmatrix         | rename: ComputeModalKRMmatricesGlobal            |
|                                   | ComputeLocalFullKMCqMatrix    | rename: ComputeLocalFullKMCqMatrices             |
|                                   | ComputeInertialKRMmatrix      | remove                                           |
|                                   | ComputeStiffnessMatrix        | remove                                           |
|                                   | ComputeDampingMatrix          | remove                                           |
|                                   | DoModalReduction_CraigBamption | remove                                          |
|                                   | DoModalReduction_HERTING      | rename: ApplyModeAccelerationTransformation      |
|                                   | DumpSubassemblyMatrices       | rename: WriteSubassemblyMatrices                 |
|                                   | Get_full_assembly_x_old       | rename: GetDeformedState                         |
|                                   | Get_modal_K                   | rename: GetModalStiffnessMatrix                  |
|                                   | Get_modal_M                   | rename: GetModalMassMatrix                       |
|                                   | Get_modal_Psi                 | rename: GetModalReductionMatrix                  |
|                                   | Get_modal_q                   | rename: GetModalCoordinatesPosLevel              |
|                                   | Get_modal_q_dt                | rename: GetModalCoordinatesVelLevel              |
|                                   | Get_modal_q_dtdt              | rename: GetModalCoordinatesAccLevel              |
|                                   | Get_modal_R                   | rename: GetModalDampingMatrix                    |
|                                   | Get_modes_assembly_x0         | rename: GetInitialState                          |
|                                   | Get_modes_damping_ratios      | rename: GetDampingRatios                         |
|                                   | Get_modes_eig                 | rename: GetEigenValues                           |
|                                   | Get_modes_frequencies         | rename: GetUndampedFrequencies                   |
|                                   | Get_modes_V                   | rename: GetEigenVectors                          |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
|                                   | SwitchModalReductionON        | rename: DoModalReduction                         |
|                                   | SetFullStateWithModeOverlay   | rename: UpdateFullStateWithModeOverlay           |
|                                   | SetInternalStateWithModes     | rename: UpdateInternalStateWithModes             |
| ChMotionlawFilter                 |                               | rename: ChMotionFilter                           |
| ChMotionlawFilter_SecondOrder     |                               | rename: ChMotionFilterSecondOrder                |
| ChMotionlawFilter_ThirdOrder      |                               | rename: ChMotionFilterThirdOrder                 |
| ChNodeBase                        |                               |                                                  |
|                                   | GetNdofX                      | rename: GetNumCoordsPosLevel                     |
|                                   | GetNdofX_active               | rename: GetNumCoordsPosLevelActive               |
|                                   | GetNdofW                      | rename: GetNumCoordsVelLevel                     |
|                                   | GetNdofW_active               | rename: GetNumCoordsVelLevelActive               |
|                                   | NodeGetOffsetX                | rename: NodeGetOffsetPosLevel                    |
|                                   | NodeGetOffsetW                | rename: NodeGetOffsetVelLevel                    |
|                                   | NodeSetOffset_x               | rename: NodeSetOffsetPosLevel                    |
|                                   | NodeSetOffset_w               | rename: NodeSetOffsetVelLevel                    |
|                                   | UseFullDof                    | rename: IsAllCoordsActive                        |
| ChNodeFEAbase                     |                               |                                                  |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChNodeFEAcurv                     |                               |                                                  |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChNodeFEAxyz                      |                               |                                                  |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChNodeFEAxyzD                     |                               |                                                  |
|                                   | GetD                          | rename: GetSlope1                                |
|                                   | GetD_dt                       | rename: GetSlope1Dt                              |
|                                   | GetD_dtdt                     | rename: GetSlope1Dt2                             |
|                                   | IsFixedD                      | rename: IsSlope1Fixed                            |
|                                   | SetD                          | rename: SetSlope1                                |
|                                   | SetD_dt                       | rename: SetSlope1Dt                              |
|                                   | SetD_dtdt                     | rename: SetSlope1Dt2                             |
|                                   | SetFixedD                     | rename: SetSlope1Fixed                           |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
|                                   | Variable_D                    | rename: VariablesSlope1                          |
| ChNodeFEAxyzDD                    |                               |                                                  |
|                                   | GetDD                         | rename: GetSlope2                                |
|                                   | GetDD_dt                      | rename: GetSlope2Dt                              |
|                                   | GetDD_dtdt                    | rename: GetSlope2Dt2                             |
|                                   | IsFixedDD                     | rename: IsSlope2Fixed                            |
|                                   | SetDD                         | rename: SetSlope2                                |
|                                   | SetDD_dt                      | rename: SetSlope2Dt                              |
|                                   | SetDD_dtdt                    | rename: SetSlope2Dt2                             |
|                                   | SetFixedDD                    | rename: SetSlope2Fixed                           |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
|                                   | Variable_DD                   | rename: VariablesSlope2                          |
| ChNodeFEAxyzDDD                   |                               |                                                  |
|                                   | GetDDD                        | rename: GetSlope3                                |
|                                   | GetDDD_dt                     | rename: GetSlope3Dt                              |
|                                   | GetDDD_dtdt                   | rename: GetSlope3Dt2                             |
|                                   | IsFixedDDD                    | rename: IsSlope3Fixed                            |
|                                   | SetDDD                        | rename: SetSlope3                                |
|                                   | SetDDD_dt                     | rename: SetSlope3Dt                              |
|                                   | SetDDD_dtdt                   | rename: SetSlope3Dt2                             |
|                                   | SetFixedDDD                   | rename: SetSlope3Fixed                           |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
|                                   | Variable_DDD                  | rename: VariablesSlope3                          |
| ChNodeFEAxyzP                     |                               |                                                  |
|                                   | GetF                          | rename: GetLoad                                  |
|                                   | GetP                          | rename: GetFieldVal                              |
|                                   | GetP_dt                       | rename: GetFieldValDt                            |
|                                   | SetF                          | rename: SetLoad                                  |
|                                   | SetP                          | rename: SetFieldVal                              |
|                                   | SetP_dt                       | rename: SetFieldValDt                            |
| ChNodeFEAxyzrot                   |                               |                                                  |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChObj                             |                               |                                                  |
|                                   | SetIdentifier                 | remove (see Notes)                               |
|                                   | GetName                       | remove                                           |
|                                   | GetNameString                 | rename: GetName                                  |
|                                   | SetName                       | remove                                           |
|                                   | SetNameString                 | rename: SetName                                  |
| ChParticleCloud                   |                               |                                                  |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
| ChPhysicsItem                     |                               |                                                  |
|                                   | ConstraintsLoadJacobian       | rename: LoadConstraintJacobians                  |
|                                   | GetDOC                        | rename: GetNumConstraints                        |
|                                   | GetDOC_c                      | rename: GetNumConstraintsBilateral               |
|                                   | GetDOC_d                      | rename: GetNumConstraintsUnilateral              |
|                                   | GetDOF                        | rename: GetNumCoordsPosLevel                     |
|                                   | GetDOF_w                      | rename: GetNumCoordsVelLevel                     |
|                                   | InjectKRMmatrices             | rename: InjectKRMMatrices                        |
|                                   | KRMmatricesLoad               | rename: LoadKRMMatrices                          |
|                                   | SetNoSpeedNoAcceleration      | rename: SetZeroVelocityZeroAcceleration          |
| ChQuaternion                      |                               |                                                  |
|                                   | free functions                | rename and move to ChRotation.h (see Notes)      |
|                                   | GetXaxis                      | rename: GetAxisX                                 |
|                                   | GetYaxis                      | rename: GetAxisY                                 |
|                                   | GetZaxis                      | rename: GetAxisZ                                 |
|                                   | ImmQ_complete                 | remove                                           |
|                                   | ImmQ_dt_complete              | remove                                           |
|                                   | ImmQ_dtdtcomplete             | remove                                           |
|                                   | operator%                     | remove  use operator*()                          |
|                                   | Q_from_AngAxis                | rename: SetFromAngleAxis                         |
|                                   | Q_from_AngX                   | rename: SetFromAngleX                            |
|                                   | Q_from_AngY                   | rename: SetFromAngleY                            |
|                                   | Q_from_AngZ                   | rename: SetFromAngleZ                            |
|                                   | Q_from_Euler123               | rename: SetFromCardanAnglesXYZ                   |
|                                   | Q_from_NasaAngles             | rename: SetFromCardanAnglesZYX                   |
|                                   | Q_from_Rotv                   | rename: SetFromRotVec                            |
|                                   | Q_to_AngAxis                  | rename: GetAngleAxis                             |
|                                   | Q_to_Euler123                 | rename: GetCardanAnglesXYZ                       |
|                                   | Q_to_NasaAngles               | rename: GetCardanAnglesZYX                       |
|                                   | Q_to_Rotv                     | rename: GetRotVec                                |
|                                   | Qdt_from_AngAxis              | rename: SetDtFromAngleAxis                       |
|                                   | Qdt_from_Wabs                 | rename: SetDtFromAngVelAbs                       |
|                                   | Qdt_to_Wabs                   | rename: GetAngVelAbs                             |
|                                   | Qdt_to_Wrel                   | rename: GetAngVelRel                             |
|                                   | Qdt_from_Wrel                 | rename: SetDtFromAngVelRel                       |
|                                   | Qdtdt_from_Aabs               | rename: SetDt2FromAngAccAbs                      |
|                                   | Qdtdt_from_Arel               | rename: SetDt2FromAngAccRel                      |
|                                   | Qdtdt_from_AngAxis            | rename: SetDt2FromAngleAxis                      |
| ChShaft                           |                               |                                                  |
|                                   | GetAppliedTorque              | rename: GetAppliedLoad                           |
|                                   | GetId                         | rename: GetIndex (internal use only)             |
|                                   | GetGid                        | remove                                           |
|                                   | GetLimitSpeed                 | remove                                           |
|                                   | GetShaftFixed                 | rename: IsFixed                                  |
|                                   | GetSleeping                   | rename: IsSleeping                               |
|                                   | GetSleepMinSpeed              | remove                                           |
|                                   | GetSleepMinWvel               | remove                                           |
|                                   | GetSleepTime                  | remove                                           |
|                                   | GetUseSleeping                | rename: IsSleepingAllowed                        |
|                                   | SetAppliedTorque              | rename: SetAppliedLoad                           |
|                                   | SetNoSpeedNoAcceleration      | rename: ForceToRest                              |
|                                   | SetSleepMinWvel               | remove                                           |
|                                   | SetShaftFixed                 | rename: SetFixed                                 |
|                                   | SetUseSleeping                | rename: SetSleepingAllowed                       |
| ChShaftsBody                      |                               | rename: ChShaftBodyRotation                      |
| ChShaftsBodyTranslation           |                               | rename: ChShaftBodyTranslation                   |
| ChShaftsClutch                    |                               |                                                  |
|                                   | GetSlippage_dt                | rename: GetSlippageDt                            |
|                                   | GetSlippage_dtdt              | rename: GetSlippageDt2                           |
| ChShaftsCouple                    |                               |                                                  |
|                                   | GetRelativeRotation           | rename: GetRelativePos                           |
|                                   | GetRelativeRotation_dt        | rename: GetRelativePosDt                         |
|                                   | GetRelativeRotation_dtdt      | rename: GetRelativePosDt2                        |
|                                   | GetTorqueReactionOn1          | rename: GetReaction1                             |
|                                   | GetTorqueReactionOn2          | rename: GetReaction2                             |
| ChShaftsElasticGear               |                               |                                                  |
|                                   | GetGearRadiusA                | rename: GetGearRadius1                           |
|                                   | GetGearRadiusB                | rename: GetGearRadius2                           |
| ChShaftsGear                      |                               |                                                  |
|                                   | GetAvoidPhaseDrift            | remove                                           |
|                                   | SetAvoidPhaseDrift            | rename: AvoidPhaseDrift                          |
| ChShaftsLoad                      |                               |                                                  |
|                                   | GetShaftA                     | rename: GetShaft1                                |
|                                   | GetShaftB                     | rename: GetShaft2                                |
| ChShaftsMotor                     |                               | remove                                           |
| ChShaftsMotorBase                 |                               | rename: ChShaftsMotor                            |
|                                   | GetMotorRot                   | rename: GetMotorPos                              |
|                                   | GetMotorRot_dt                | rename: GetMotorPosDt                            |
|                                   | GetMotorRot_dtdt              | rename: GetMotorPosDt2                           |
|                                   | GetMotorRotPeriodic           | rename: GetMotorAngleWrapped                     |
|                                   | GetMotorRotTurns              | rename: GetMotorNumTurns                         |
|                                   | GetMotorTorque                | rename: GetMotorLoad                             |
|                                   | GetTorqueReactionOn1          | rename: GetReaction1                             |
|                                   | GetTorqueReactionOn2          | rename: GetReaction2                             |
| ChShaftsMotorAngle                |                               | rename: ChShaftsMotorPosition                    |
|                                   | GetAngleFunction              | rename: GetPositionFunction                      |
|                                   | GetAngleOffset                | rename: GetOffset                                |
|                                   | SetAngleFunction              | rename: SetPositionFunction                      |
|                                   | SetAngleOffset                | rename: SetOffset                                |
| ChShaftsMotorSpeed                |                               |                                                  |
|                                   | GetAvoidAngleDrift            | remove                                           |
|                                   | GetAngleOffset                | rename: GetOffset                                |
|                                   | SetAvoidAngleDrift            | rename: AvoidDrift                               |
|                                   | SetAngleOffset                | rename: SetOffset                                |
| ChShaftsMotorTorque               |                               | rename: ChShaftsMotorLoad                        |
|                                   | GetTorqueFunction             | rename: GetLoadFunction                          |
|                                   | SetTorqueFunction             | rename: SetLoadFunction                          |
| ChShaftsPlanetary                 |                               |                                                  |
|                                   | GetAvoidPhaseDrift            | remove                                           |
|                                   | SetAvoidPhaseDrift            | rename: AvoidPhaseDrift                          |
| ChShaftsTorque                    |                               | rename: ChShaftsAppliedTorque                    |
| ChShaftsTorqueBase                |                               | rename: ChShaftsTorque                           |
| ChSolverBB                        |                               |                                                  |
|                                   | GetMaxArmijoBacktrace         | rename: GetMaxStepsArmijoBacktrace               |
|                                   | GetNarmijo                    | rename: GetMaxStepsArmijoLineSearch              |
|                                   | SetMaxArmijoBacktrace         | rename: SetMaxStepsArmijoBacktrace               |
|                                   | SetNarmijo                    | rename: SetMaxStepsArmijoLineSearch              |
| ChStaticAnalysis                  |                               | remove                                           |
|                                   | GetL                          | rename: GetLagrangeMultipliers                   |
|                                   | GetX                          | rename: GetStatePos                              |
| ChStream                          |                               | remove                                           |
| ChStreamFile                      |                               | remove                                           |
| ChStreamIn                        |                               | remove                                           |
| ChStreamInAscii                   |                               | remove                                           |
| ChStreamInAsciiFile               |                               | remove                                           |
| ChStreamInBinary                  |                               | remove                                           |
| ChStreamInBinaryFile              |                               | remove                                           |
| ChStreamOut                       |                               | remove                                           |
| ChStreamOutAscii                  |                               | remove                                           |
| ChStreamOutAsciiFile              |                               | remove                                           |
| ChStreamOutBinary                 |                               | remove                                           |
| ChStreamOutBinaryFile             |                               | remove                                           |
| ChSurface                         |                               |                                                  |
|                                   | Get_closed_U                  | rename: IsClosedU                                |
|                                   | Get_closed_V                  | rename: IsClosedV                                |
| ChSystem                          |                               |                                                  |
|                                   | ConstraintsLoadJacobian       | rename: LoadConstraintJacobians                  |
|                                   | DoEntireDynamics              | remove                                           |
|                                   | DoEntireKinematics            | remove                                           |
|                                   | DoEntireUniformDynamics       | remove                                           |
|                                   | DoFullAssembly                | remove  use DoAssembly(AssemblyLevel::FULL)      |
|                                   | DumpSystemMatrices            | rename: WriteSystemMatrices                      |
|                                   | Get_bodylist                  | rename: GetBodies                                |
|                                   | Get_G_acc                     | rename: GetGravitationalAcceleration             |
|                                   | Get_linklist                  | rename: GetLinks                                 |
|                                   | Get_meshlist                  | rename: GetMeshes                                |
|                                   | Get_otherphysicslist          | rename: GetOtherPhysicsItems                     |
|                                   | Get_shaftslist                | rename: GetShafts                                |
|                                   | GetDOC                        | remove                                           |
|                                   | GetDOC_c                      | remove                                           |
|                                   | GetDOC_d                      | remove                                           |
|                                   | GetDOF                        | remove                                           |
|                                   | GetDOF_w                      | remove                                           |
|                                   | GetMaxiter                    | remove                                           |
|                                   | GetMinBounceSpeed             | remove                                           |
|                                   | GetNbodies                    | rename: GetNumBodiesActive                       |
|                                   | GetNbodiesFixed               | rename: GetNumBodiesFixed                        |
|                                   | GetNbodiesSleeping            | rename: GetNumBodiesSleeping                     |
|                                   | GetNbodiesTotal               | rename: GetNumBodies                             |
|                                   | GetNconstr                    | rename: GetNumConstraints                        |
|                                   | GetNcontacts                  | rename: GetNumContacts                           |
|                                   | GetNcoords                    | rename: GetNumCoordsPosLevel                     |
|                                   | GetNcoords_w                  | rename: GetNumCoordsVelLevel                     |
|                                   | GetNdoc                       | remove: no constraints at position level         |
|                                   | GetNdoc_w                     | rename: GetNumConstraints                        |
|                                   | GetNdoc_w_C                   | rename: GetNumConstraintsBilateral               |
|                                   | GetNdoc_w_D                   | rename: GetNumConstraintsUnilateral              |
|                                   | GetNdof                       | remove: it was a rough estimate                  |
|                                   | GetNlinks                     | rename: GetNumLinksActive                        |
|                                   | GetNmeshes                    | rename: GetNumMeshes                             |
|                                   | GetNphysicsItems              | rename: GetNumOtherPhysicsItemsActive            |
|                                   | GetNshafts                    | rename: GetNumShafts                             |
|                                   | GetNshaftsFixed               | rename: GetNumShaftsFixed                        |
|                                   | GetNshaftsSleeping            | rename: GetNumShaftsSleeping                     |
|                                   | GetNshaftsTotal               | rename: GetNumShaftsTotal                        |
|                                   | GetNsysvars                   | remove                                           |
|                                   | GetNsysvars_w                 | remove                                           |
|                                   | GetSolverCallsCount           | rename: GetSolverSolveCount                      |
|                                   | GetSolverForceTolerance       | remove                                           |
|                                   | GetSolverMaxIterations        | remove                                           |
|                                   | GetSolverTolerance            | remove                                           |
|                                   | GetStepcount                  | rename: GetNumSteps                              |
|                                   | GetUseSleeping                | rename: IsSleepingAllowed                        |
|                                   | InjectKRMmatrices             | rename: InjectKRMMatrices                        |
|                                   | Integrate_Y                   | rename: AdvanceDynamics                          |
|                                   | KRMmatricesLoad               | rename: LoadKRMMatrices                          |
|                                   | ResetStepcount                | rename: ResetNumSteps                            |
|                                   | Set_G_acc                     | rename: SetGravitationalAcceleration             |
|                                   | SetMaxiter                    | remove                                           |
|                                   | SetMinBounceSpeed             | remove and move to ChSystemNSC                   |
|                                   | SetSolverForceTolerance       | remove (see Notes)                               |
|                                   | SetSolverMaxIterations        | remove (see Notes)                               |
|                                   | SetSolverTolerance            | remove (see Notes)                               |
|                                   | SetStep                       | remove                                           |
|                                   | SetUseSleeping                | rename: SetSleepingAllowed                       |
| ChSystemSMC                       |                               |                                                  |
|                                   | GetStiffContact               | rename: IsContactStiff                           |
|                                   | SetStiffContact               | rename: SetContactStiff                          |
| ChSystemDescriptor                |                               |                                                  |
|                                   | ConvertToMatrixForm           | rename: BuildSystemMatrix                        |
|                                   | GetConstraintsList            | rename: GetConstraints                           |
|                                   | GetKblocksList                | rename: GetKRMBlocks                             |
|                                   | GetVariablesList              | rename: GetVariables                             |
|                                   | InsertKblock                  | rename: InsertKRMBlock                           |
| ChSystemFsi                       |                               |                                                  |
|                                   | Get_G_acc                     | rename: GetGravitationalAcceleration             |
|                                   | Set_G_acc                     | rename: SetGravitationalAcceleration             |
| ChSystemMulticore                 |                               |                                                  |
|                                   | GetNumBilaterals              | rename: GetNumConstraintsBilateral               |
|                                   | Integrate_Y                   | rename: AdvanceDynamics                          |
| ChTimestepper                     |                               |                                                  |
|                                   | get_L                         | rename: GetLagrangeMultipliers                   |
| ChTimestepperIorder               |                               |                                                  |
|                                   | get_Y                         | rename: GetState                                 |
|                                   | get_dYdt                      | rename: GetStateDt                               |
| ChTimestepperIIorder              |                               |                                                  |
|                                   | get_X                         | rename: GetStatePos                              |
|                                   | get_V                         | rename: GetStateVel                              |
|                                   | get_A                         | rename: GetStateAcc                              |
| ChTriangleMesh                    |                               |                                                  |
|                                   | addTriangle                   | rename: AddTriangle                              |
|                                   | getNumTriangles               | rename: GetNumTriangles                          |
|                                   | getTriangle                   | rename: GetTriangle                              |
| ChTimer                           |                               | see Notes                                        |
|                                   | GetTimeMicrosecondsIntermediate | remove                                         |
|                                   | GetTimeMillisecondsIntermediate | remove                                         |
|                                   | GetTimeSecondsIntermediate      | remove                                         |
| ChTriangleMeshConnected           |                               |                                                  |
|                                   | addTriangle                   | rename: AddTriangle                              |
|                                   | getCoordsVertices             | rename: GetCoordsVertices                        |
|                                   | getCoordsNormals              | rename: GetCoordsNormals                         |
|                                   | getCoordsUV                   | rename: GetCoordsUV                              |
|                                   | getCoordsColors               | rename: GetCoordsColors                          |
|                                   | getIndicesVertexes            | rename: GetIndicesVertexes                       |
|                                   | getIndicesNormals             | rename: GetIndicesNormals                        |
|                                   | getIndicesUV                  | rename: GetIndicesUV                             |
|                                   | getIndicesColors              | rename: GetIndicesColors                         |
|                                   | getIndicesMaterials           | rename: GetIndicesMaterials                      |
|                                   | getNumNormals                 | rename: GetNumNormals                            |
|                                   | getNumTriangles               | rename: GetNumTriangles                          |
|                                   | getNumVertices                | rename: GetNumVertices                           |
|                                   | getPropertiesPerVertex        | rename: GetPropertiesPerVertex                   |
|                                   | getPropertiesPerFace          | rename: GetPropertiesPerFace                     |
|                                   | getTriangle                   | rename: GetTriangle                              |
| ChTriangleMeshSoup                |                               |                                                  |
|                                   | addTriangle                   | rename: AddTriangle                              |
|                                   | getNumTriangles               | rename: GetNumTriangles                          |
|                                   | getTriangle                   | rename: GetTriangle                              |
| ChTriangleOfXYZnodes              |                               | rename: ChTriangleNodesXYZ                       |
| ChTriangleOfXYZROTnodes           |                               | rename: ChTriangleNodesXYZrot                    |
| ChVariables                       |                               |                                                  |
|                                   | Build_M                       | rename: PasteMassInto  (see Notes)               |
|                                   | Compute_fb                    | remove                                           |
|                                   | Compute_invMb_v               | rename: ComputeMassInverseTimesVector            |
|                                   | Compute_inc_invMb_v           | remove                                           |
|                                   | Compute_inc_Mb_v              | rename: AddMassTimesVector                       |
|                                   | DiagonalAdd                   | rename: AddMassDiagonalInto                      |
|                                   | Get_ndof                      | rename: GetDOF                                   |
|                                   | Get_qb                        | rename: State                                    |
|                                   | Get_fb                        | rename: Force                                    |
|                                   | MultiplyAndAdd                | rename: AddMassTimesVectorInto                   |
| ChVector                          |                               | rename: ChVector3                                |
|                                   | DirToDxDyDz                   | rename: GetDirectionAxesAsX                      |
| ChVisualModel                     |                               |                                                  |
|                                   | GetShapes                     | rename: GetShapeInstances                        |
| ChVolume                          |                               |                                                  |
|                                   | Get_closed_U                  | rename: IsClosedU                                |
|                                   | Get_closed_V                  | rename: IsClosedV                                |
|                                   | Get_closed_W                  | rename: IsClosedW                                |
| CSV_writer                        |                               | rename: ChWriterCSV                              |
|                                   | delim                         | rename: GetDelimiter                             |
|                                   | set_delim                     | rename: SetDelimiter                             |
|                                   | stream                        | rename: Stream                                   |
|                                   | write_to_file                 | rename: WriteToFile                              |
| Generator                         |                               | rename: ChGenerator                              |
|                                   | getBodyIdentifier             | rename: GetBodyIdentifier                        |
|                                   | getTotalMass                  | rename: GetTotalMass                             |
|                                   | getTotalNumBodies             | rename: GetTotalNumBodies                        |
|                                   | getTotalVolume                | rename: GetTotalVolume                           |
|                                   | setBodyIdentifier             | rename: SetStartTag                              |
| GridSampler                       |                               | rename: ChGridSampler                            |
| HCPSampler                        |                               | rename: ChHCPSampler                             |
| MixtureIngredient                 |                               | rename: ChMixtureIngredient                      |
|                                   | setDefaultDensity             | rename: SetDefaultDensity                        |
|                                   | setDefaultMaterial            | rename: SetDefaultMaterial                       |
|                                   | setDefaultSize                | rename: SetDefaultSize                           |
|                                   | setDistributionCohesion       | rename: SetDistributionCohesion                  |
|                                   | setDistributionDensity        | rename: SetDistributionDensity                   |
|                                   | setDistributionFriction       | rename: SetDistributionFriction                  |
|                                   | setDistributionPoisson        | rename: SetDistributionPoisson                   |
|                                   | setDistributionRestitution    | rename: SetDistributionRestitution               |
|                                   | setDistributionSize           | rename: SetDistributionSize                      |
|                                   | setDistributionYoung          | rename: SetDistributionYoung                     |
| PDSampler                         |                               | rename: ChPDSampler                              |
| Sampler                           |                               | rename: ChSampler                                |


**Types**

| Class                             | Type                | Action                                           |
| :-------------------------------- | :------------------ | :----------------------------------------------- |
| ChCoordsys                        | Coordsys            | rename: ChCoordsysd                              |
|                                   | CoordsysF           | rename: ChCoordsysf                              |
| ChQuaternion                      | Quaternion          | rename: ChQuaterniond                            |
|                                   | QuaternionF         | rename: ChQuaternionf                            |
| ChVector                          | Vector              | rename: ChVector3d                               |
|                                   | VectorF             | rename: ChVector3f                               |

**Constants**

| Name                              | Action                                           |
| :-------------------------------- | :----------------------------------------------- |
| CH_C_1_PI                         | remove                                           |
| CH_C_2PI                          | rename: CH_2PI                                   |
| CH_C_DEG_TO_RAD                   | rename: CH_DEG_TO_RAD                            |
| CH_C_E                            | remove                                           |
| CH_C_LN10                         | remove                                           |
| CH_C_LN2                          | remove                                           |
| CH_C_LOG10E                       | remove                                           |
| CH_C_LOG2E                        | remove                                           |
| CH_C_PI                           | rename: CH_PI                                    |
| CH_C_PI_2                         | rename: CH_PI_2                                  |
| CH_C_PI_4                         | rename: CH_PI_4                                  |
| CH_C_RAD_TO_DEG                   | rename: CH_RAD_TO_DEG                            |
| CH_C_RPM_TO_RPS                   | rename: CH_RPM_TO_RAD_S                          |
| CH_C_SQRT_1_2                     | remove                                           |
| CH_C_SQRT_2                       | rename: CH_SQRT_2                                |


**Notes**

+ The code for `ChFunction` classes was moved from `src/chrono/motion_functions/` to `src/chrono/functions/`. As such, include headers should be changed to something like:
  ```cpp
  #include "chrono/functions/ChFunctionSine.h"
  ```

+ The `chrono::geometry` namespace was removed. All geometry classes are now in the `chrono` namespace.

+ Chrono object identifiers were made read-only so that uniqueness can be guaranteed. 
  - These integer identifiers are read-only and can be cached by the user (e.g., for searching in a ChAssembly).
    Identifiers are generated automatically in incremental order based on the order in which objects are created.
    As transient quantities, object identifiers are not serialized.
  - Chrono objects can now be tagged (using newly introduced functions `SetTag`/`GetTag`).
    Unlike object identifiers, object tags are completely under user control and not used anywhere else in Chrono.
    Tags are serialized and de-serialized.

+ Functions that duplicated C++ Standard Library functions were removed and replaced with the corresponding C++ function (e.g., `ChMin` was obsoleted in favor of `std::min`).

+ The enum `AngleSet` (previously defined in ChQuaternion.h) was renamed to `RotRepresentation` and moved to a new header named ChRotation.h.

+ All free functions for converting from one rotation representation to another are now located in ChRotation.h.
  These functions have consistent names of the form *XxxFromYyy*.

  | Old (in ChQuaternion.h)  | New (in ChRotation.h)         |
  | :----------------------- | :-----------------------------|
  | -                        | AngleSetFromAngleSet          |
  | -                        | AngleSetFromRodriguez         |
  | -                        | RodriguezFromAngleSet         |
  | -                        | QuatFromRotVec                |
  | -                        | RotVecFromQuat                |
  | Angle_to_Quat            | QuatFromRodriguez             |
  | Angle_to_Quat            | QuatFromAngleSet              |
  | AngleDT_to_QuatDT        | QuatDtFromAngleSet            |
  | AngleDT_to_QuatDT        | QuatDtFromRodriguez           |
  | AngleDTDT_to_QuatDTDT    | QuatDt2FromAngleSet           |
  | AngleDTDT_to_QuatDTDT    | QuatDt2FromRodriguez          |
  | ImmQ_complete            | QuatFromImaginary             |
  | ImmQ_dt_complete         | QuatDtFromImaginary           |
  | ImmQ_dtdt_complete       | QuatDt2FromImaginary          |
  | Q_from_AngAxis           | QuatFromAngleAxis             |
  | Q_from_AngX              | QuatFromAngleX                |
  | Q_from_AngY              | QuatFromAngleY                |
  | Q_from_AngZ              | QuatFromAngleZ                |
  | Q_from_Euler123          | QuatFromAngleSet              |
  | Q_from_NasaAngles        | QuatFromAngleSet              |
  | Q_from_Vect_to_Vect      | QuatFromVec2Vec               |
  | Q_to_AngAxis             | AngleAxisFromQuat             |
  | Q_to_Euler123            | AngleSetFromQuat              |
  | Q_to_NasaAngles          | AngleSetFromQuat              |
  | Qdt_from_AngAxis         | QuatDtFromAngleAxis           |
  | Qdtdt_from_AngAxis       | QuatDt2FromAngleAxis          |
  | Quat_to_Angle            | RodriguezFromQuat             |
  | Quat_to_Angle            | AngleSetFromQuat              |

+ For consistency, functions for transforming a `ChFrame` or a `ChCoordsys` from one reference to another (`TransformFromLocalToParent` and `TransformFromParentToLocal`) were modified so that they return the transformed object (previously, the returned object was pased as an argument).
  For clarity, we removed functions such as `ChFrame::TransformLocalToParent(const ChVector<>& local)` to express a 3D vector given in local coordinates to the parent frame. Use instead `ChFrame::TransformPointLocalToParent`.

+ `ChLinkMate` and derived classes have been rewritten so that:
  - for links with a single DOF, the relevant axis is Z, in line with `ChLinkLock` formulation (e.g., a `ChLinkMatePrismatic` allows translation along the Z axis of the link frame, a ChLinkMateRevolute allows rotation about the Z axis of the link frame, etc.).
  - for links with 2 DOFs, the relevant axes are X and Y. The exception to this is `ChLinkMateRackPinion` which uses the Z axis of the link frame as axis of rotation for the pinion and the X as direction of translation for the rack.
  - the 'flipped==true' state now refers to axes that are counter-aligned.

+ Link objects used to consider just one frame as 'principal' (usually 'frame 2'), thus returning reaction forces, frame position, as well as any other information with respect to this frame only.
  - For consistency and to remove ambiguity, all links (connections between two physical items) now report two frames, one on each connected object. These frames, expressed in the absolute coordinate frame can be obtained through the functions `GetFrame1Abs` and `GetFrame2Abs`.
    Certain derived classes (notably those connecting two `ChBody` objects, in particular all classes representing kinematic joints) also provide functions to return the link frames expressed in the frame of the corresponding connected body (`GetFrame1Rel` and `GetFrame2Rel`).
  - Similarly, all link objects now provide functions to return the reaction force and torque at the location of the link frame on the connected object. These reactions are expressed in the corresponding link frame and can be obtained, as a wrench, through calls to `GetReaction[1|2]`.

+ The signature of all ChLink `Initialize()` functions were changed to consistently use `ChFrame` objects to specify link position and alignment (where previously some of them used `ChCoordsys`).
  A corresponding change was done for the constructor of `vehicle::ChVehicleJoint`.

+ The `ChLoad` class (loads on physics items via `ChLoader` objects) was changed to a non-templated class.  Instead of specifying the ChLoader as a template parameter, a shared pointer to a ChLoader is now passed as a constructor argument. 
  This streamlines the code and allows proper SWIG wrapping for use in Python or C# codes. 
  All pre-defined ChLoader classes (e.g., `ChLoaderPressure`, `ChLoaderGravity`, `ChLoaderBeamWrench`, etc.) were updated accordingly.
  An example of a user-provided loader class (derived from some `ChLoader`) is as follows:
  ```cpp
  class MyLoader : public ChLoaderUatomic {
    public:
      MyLoader(std::shared_ptr<ChLoadableU> loadable) : ChLoaderUatomic(loadable) {}

      virtual void ComputeF(const double U,              // normalized line position
                            ChVectorDynamic<>& F,        // load at U (set to zero on entry)
                            ChVectorDynamic<>* state_x,  // if non-null, first update position to this
                            ChVectorDynamic<>* state_w   // if non-null, first update velocities to this
                            ) override {
          // Compute F = F(U)...
      }
  };

  auto loader = chrono_types::make_shared<MyLoader>(object);
  auto load = chrono_types::make_shared<ChLoad>(loader);
  load_container->Add(load);
  ```

+ `ChStream` classes were simple wrappers around C++ output streams. The entire code has been now refactored to operate on STL streams directly, in order to simplify the API and to allow the user a more familiar interaction with streams in Chrono.
  - the `operator<<` associated to `ChStreamOutAscii` has been removed: this means that it is not possible to stream custom classes through this operator;
  this option has been replaced by appropriate overloads of STL streams (clearly limited to data members publicly accessible) or by the direct usage of `ChOutputASCII`.
  - `ChStream[In/Out]Binary` features to operate on streams through the `read|write` methods have been incorporated into the one class that was using it, namely `ChArchiveBinary`. 

+ `ChArchive` classes have been refactored to adapt to the direct use of STL streams and have been renamed for consistency and clarity.
  - proper serialization/de-serialization classes kept the same name:
    + `ChArchiveJSON[In/Out]`
    + `ChArchiveXML[In/Out]`
    + `ChArchiveBinary[In/Out]`
  - auxiliary classes which, while still leveraging the `ChArchive` features, allow only export but not loading back objects were renamed:
    + `ChArchiveAsciiDump` &#8594; `ChOutputASCII`
    + `ChArchiveExplorer` &#8594; `ChObjectExplorer`

+ Solver access through `ChSystem` was removed. The functions `SetSolverMaxIterations`, `SetSolverTolerance`, and `SetSolverForceTolerance` were controlling only a subset of solver parameters and only for iterative solvers.
  Instead, the user must set solver parameters directly on the solver object. This is straightforward for the case where the user explicitly creates a solver object and attaches it to the system with `ChSystem::SetSolver`.
  For the case where the default solver is used or where the solver is set through `ChSystem::SetSolverType`, we implemented a mechanism on `ChSolver` and derived classes which permits identifying the solver type
  and setting solver parameters without the need for dynamic casting and testing. This mechanism is provided through `ChSolver` functions and can be used as illustrated below:
  ```cpp
  if (my_system.GetSolver()->IsIterative()) {
     my_system.GetSolver()->AsIterative()->SetMaxIterations(100);
     my_system.GetSolver()->AsIterative()->SetTolerance(1e-6);
  }
  ```
  or
  ```cpp
  my_system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
  my_system.GetSolver()->AsIterative()->SetMaxIterations(20);
  ```
+ We removed the option of setting an integration step size for the types of analyses that require it. Instead, the desired step size value is always explicitly passed as an argument to the `ChSystem` function that initiates that analysis (e.g., `DoStepDynamics`).
  The current value of the step size (which may be adjusted internally in certain situations) is cached and can still be queried with `ChSystem::GetStep`. This is typically needed only internally but can also be used in user code that requires it.

  Similarly, we removed the function `ChSystem::SetMaxiter` which allowed setting the maximum number of iterations for the system assembly analysis. This quantity can now be passed through an optional argument to `ChSystem::DoAssembly` (default value: 6).

+ `ChKblock::Build_K` (now `ChKRMBlock::PasteInto`) reversed the meaning of the last argument (was `add`, now `overwrite`) in accordance to the signature of `PasteMatrix`;
  Also `ChVariable::Build_M` (now `PasteMassInto`) is not taking the position in which the mass should be placed but the offset with respect to the `ChVariable::offset`

+ The `ChTimer` class was refactored so that `GetTimeMicroseconds`, `GetTimeMilliseconds`, and `GetTimeSeconds` return either an intermediate time (if the timer was started but not stopped; in this case the timer keeps running) or the timer value when the timer was stopped.
  The old functions `GetTimeMicrosecondsIntermediate`, `GetTimeMillisecondsIntermediate`, and `GetTimeSecondsIntermediate` were obsoleted.

## [Added] Functional Mock-Up Interface (FMI) support

The Functional Mock-up Interface is an open (tool-independent) standard for exchanging dynamical simulation models between different tools in a standardized format.  These models are encapsulated in so-called Functional Mock-Up Units (FMU) which contain a description of the model (as an XML file) and binaries (as shared libraries).

FMI support in Chrono is provided via (1) `fmu-tools`, a general-purpose, stand-alone library for exporting and importing FMUs and (2) `Chrono::FMI`, a module with Chrono-specific extensions to facilitate working with FMU variables wrapping Chrono types.

At this time, only the FMI 2.0 standard is supported (with FMI 3.0 support coming later). The stand-alone `fmu_tools` library provides support for exporting and importing both *Co-Simulation* and *Model Exchange* FMUs.  Currently, `Chrono:FMI` focuses only on Co-Simulation FMUs. 

## [Added] Chrono::Sensor features and updates

**Updates and Bug Fixes**
  - Upgraded to Optix 7.7, requiring NVIDIA drive 530.41 or newer
  - Fixed issues with CUDA NVRTC runtime compilation of Optix shaders

**New Features**
  - Added supprt for Area Lights (rectangular shaped). Can be added to a scene using `ChScene::AddAreaLights(ChVector3f position, ChColor color, float max_range, ChVector3f du, ChVector3f dv)`.
  - Added support for rendering emissive surfaces. When defining a visual material, set `ChVisualMaterial::SetEmissiveColor(ChColor color)` and set `ChVisualMaterial:SetEmissivePower(float power)`.
  - Added a Depth Camera sensor (`ChDepthCamera`). The Depth Camera is initialized in the same manner as `ChCameraSensor` , with each pixel containing depth information of the scene in meters (m).
  - Added support for the Hapke BRDF model to render celestial bodies (ex: Lunar regolith). To enable, when defining a material set `ChVisualMaterial:SetUseHapke(bool enableHapke)` and set the model parameters using `ChVisualMaterial:SetHapkeParameters(float w,...)`. More information regarding the Hapke model and its parametrization can be found in <https://doi.org/10.1002/2013JE004580>.
  - Added a Random Walk based noise model for GPS sensor. To enable this noise model, when adding a `ChGPSSensor`, set a pointer to a `ChNoiseRandomWalks` object as the noise model parameter.

## [Added] Chrono::ROS module

A new module (`Chrono::ROS`) has been introduced to support direct integration of ROS 2 with Chrono. `Chrono::ROS` provides a bridge between the Chrono simulation and the ROS 2 middleware, allowing for communication of sensor data, vehicle and transformation data, and more. Compatibility with specific Chrono modules, including `Chrono::Vehicle` and `Chrono::Sensor`, is provided. In addition to providing a few default publishers and subscribers, the module exposes an API for creating custom ROS 2 logic within the Chrono simulation.

The `ChROSManager` class is the main interface for the `Chrono::ROS` module. It maintains a single `ChROSInterface` object which wraps a ROS 2 node and provides the ability to create publishers and subscribers. The `ChROSHandler` class is the base class which all ROS 2 logic (i.e. publishers, subscribers, etc.) inherit from. During initialization, the `ChROSInterface` in the `ChROSManager` is passed to the `ChROSHandler` object through the `ChROSHandler::Initialize` function. This allows the `ChROSHandler` object to create ROS 2 objects. Over the course of the simulation, the `ChROSManager` calls `ChROSHandler::Tick` on each handler at a fixed rate to update each handler _relative to the simulation time_. As Chrono may run faster or slower than wall time, it's recommended to set ROS 2 `/use_sim_time` to `true` in all ROS nodes. Multiple nodes may be created by setting the node name in the `ChROSManager` constructor.

Python bindings are also provided for the `Chrono::ROS` module. It can be imported via the `pychrono.ros` module. ROS 2 doesn't directly support wrapping of it's C++ API for use in python, and so a special `ChROSPythonManager` class was created for situations where you need direct access to the ROS 2 API (such as creating a custom handler). When the ChROSPythonManager is used, two nodes are created: the original C++ `rclcpp` node in `ChROSManager`/`ChROSInterface` and the new `rclpy`-based node. The node name for the `rclpy` node (settable via the constructor) is always the `rclcpp` node name with `_py` appended. This allows for the creation of ROS 2 publishers and subscribers in python (as two nodes can't have the same name). All python-based handlers (i.e. when a custom handler inherits from `ChROSHandler` in python) must be added to the `ChROSPythonManager` object, not the `ChROSManager` object. All C++-based handlers can be added to either the `ChROSManager` or `ChROSPythonManager` objects, as in the `ChROSManager` may still be used if custom handlers aren't created in python.

The `Chrono::ROS` module currently only supports ROS 2 humble, and no additional dependencies are required beyond the ROS 2 installation. To build the `Chrono::ROS` module, set the cmake option `ENABLE_MODULE_ROS` to `ON`.

Locations:
- `Chrono::ROS` source code is maintained under `src/chrono_ros/`
- `Chrono::ROS` python bindings are maintained under `src/chrono_swig/interface/ros`
- Demos are located in `src/demos/ros/`
- Unit tests are located in `src/unit_tests/ros/`

## [Changed] Updated Chrono::VSG module

The Chrono::VSG module was updated to use newer version of the VSG libraries.
- Adds shadow support (enabled in several demos).
- Includes an optional argument to the `ChVisualSystemVSG` constructor to specify the tesselation resolution for round primitive shapes. This is provided as a number of divisions of a full circle, with a default value of 24, corresponding to a 15 degree angular increment. 
- No other changes to the public API.

The **required** VSG library versions are as follows:
- vsg 1.1.0
- vsgXchange 1.1.0
- vsgExamples 1.1.0
- vsgImGui - latest
- assimp 5.3.1

If the above requirements are not met, an error is issued during CMake configuration (if the Chrono::VSG module is enabled).  
The simplest way to satisfy the VSG version requirements is to build and install them using the scripts provided with the Chrono distribution (see the Chrono::VSG installation instructions).


## [Added] New motion functions and filters
The following functions were added
+ `ChFunction_BSpline`: univariate B-Spline motion function, able to approximate or exactly interpolate given waypoints and derivatives 
+ `ChFunction_Cycloidal`: smooth ramp function defined by amount of displacement and motion time
+ `ChFunction_DoubleS`: constant-jerk motion function, able to minimize motion time given velocity, acceleration and jerk kinematic constraints.
These and all other `ChFunction`s are now able to compute up to third derivative.

In addition, the new non-linear filters `utils::ChMotionlawFilter_SecondOrder` and `utils::ChMotionlawFilter_ThirdOrder` were introduced for tracking raw signals with smooth motion profiles, on the fly.

## [Changed] Updated ChBlender exporter to Blender4.0
Due to some obsoleted methods the ChBlender exported was not compatible with the latest Blender4.0. It is now updated.

## [Added] Unilateral distance constraint
The `ChLinkDistance` has been expanded to include also unilateral distance constraints. Through `SetMode`/`GetMode` three different behaviors are now available:
+ `BILATERAL` (default): current_distance = imposed_distance;
+ `UNILATERAL_MAXDISTANCE`: current_distance < imposed_distance; (e.g. a rope)
+ `UNILATERAL_MINDISTANCE`: current_distance > imposed_distance;

For the latter two cases a [*VI* solver](https://api.projectchrono.org/development/simulation_system.html#solvers) is required since the unilateral constraints are non-smooth.

## [Changed] Collision detection refactoring

This refactoring affects the specification of collision models and shapes attached to contactable objects, as well as the specification of a collision system associated with a Chrono physics system. With these changes, it is now possible to specify collision information in Chrono independent of a particular implementation (e.g., the Bullet-based collision detection or the Chrono-internal multicore collision detection system), both in term of collision models and collision systems. The refactored Chrono code synchronizes the API for specifying collision and visualization shapes and models, as well as the API for creating and associating collision and visualization systems. 

The main code architecture changes and resulting API changes are as described below.

**Basic geometric shapes**

The set of basic geometric shapes and associated class hierarchy (see [src/chrono/geometry/](https://github.com/projectchrono/chrono/tree/main/src/chrono/geometry)) was updated for consistency and to allow use as common low-level primitives for both collision and visualization shapes. The various basic geometric shapes inherit from `ChLine`, `ChSurface`, and `ChVolume` for 1-D, 2-D, and 3-D geometry, respectively. Each class encapsulates the geometric data necessary to completely describe that corresponding shape and implements methods specific to its manifold dimension (1, 2, or 3).

**Collision shapes**

This refactoring introduces classes for collision shapes, including both primitive shapes (e.g., `ChCollisionShapeSphere`) as well as compound objects (e.g., `ChCollisionShapeTriangleMesh`).
See [src/chrono/collision/](https://github.com/projectchrono/chrono/tree/main/src/chrono/collision).
In general, these objects wrap a basic geometric shape to specify the collision shape geometry, optionally with some additional collision-specific information (e.g., the radius of a sphere-swept collision surface). All Chrono collision shapes contain a contact material (of type `ChMaterialSurface`) which are combined into composite contact material properties for each colliding pair. All Chrono collision shapes are generic, in that they are independent of a particular collision detection implementation. 

**Visual shapes**

Simultaneously with the refactoring of collision detection in Chrono, the names of visual shapes were changed for consistency with corresponding collision shapes. 
See [src/chrono/assets/](https://github.com/projectchrono/chrono/tree/main/src/chrono/assets).

This renaming follows the pattern `ChSphereShape` -> `ChVisualShapeSphere`.

**Collision models**

A collision model is a collection of collision shapes with associated transforms (the pose of the shape within the model). A collision model is populated by adding shapes with optional transforms. For example,
```cpp
  auto ct_model = chrono_types::make_shared<ChCollisionModel>();
  auto ct_sphere = chrono_types::make_shared<ChCollisionShapeSphere>(ct_mat1, radius);
  auto ct_box = chrono_types::make_shared<ChCollisionShapeBox>(ct_mat2, length, width, height);
  ct_model->AddShape(ct_sphere, ChFrame<>());
  ct_model->AddShape(ct_box, ChFrame<>(ChVector<>(1,1,1), QUNIT));
```
Chrono collision models are generic, in that they are independent of a particular collision detection implementation. The function `ChCollisionModel::BuildModel()` was eliminated, as processing of generic collision shapes and models by a concrete collision detection system is deferred to a later time (namely, at initialization of the collision system).  The function `ChCollisionModel::ClearModel()`, now renamed `ChCollisionModel::Clear()` still exists for special uses; in a typical user code, there is no need to call this function before populating the model with collision shapes.

**Rigid body creation**

A rigid body is created with no collision model by default. A collision model can optionally be attached to a rigid body (in fact, the collision model is managed by the parent class `ChContactable`:
```cpp
  auto body = chrono_types::make_shared<ChBody>();
  body->AddCollisionModel(ct_model);
```
The collision model on a rigid body is assumed to be positioned at the body reference frame (which coincides with the body center of mass for a `ChBody` object, but may be non-centroidal for a `ChBodyAuxRef`).

Note that a body constructor need not specify anymore a "collision type". Similarly, the functions `ChSystem::NewBody` were obsoleted. With the current refactoring, a rigid body is independent of a particular collision detection system implementation.

For convenience, an alternative way of populating shapes in the collision model of a `ChContactable` is through the convenience function `ChContactable::AddCollisionShape`. This function can be called multiple times. A container collision model is created, as needed, the first time a collision shape is specified. The code snippets shown above can be thus be replaced with the following equivalent code:
```cpp
  auto body = chrono_types::make_shared<ChBody>();
  auto ct_sphere = chrono_types::make_shared<ChCollisionShapeSphere>(ct_mat1, radius);
  auto ct_box = chrono_types::make_shared<ChCollisionShapeBox>(ct_mat2, length, width, height);
  body->AddCollisionShape(ct_sphere, ChFrame<>());
  body->AddCollisionShape(ct_box, ChFrame<>(ChVector<>(1,1,1), QUNIT));
```

In accordance with these changes, all `ChBodyEasy***` classes were modified to remove from their constructors the last argument specifying a particular collision detection implementation.

After this refactoring, the specification of a collision model closely parallels the creation of a visualization model. For example,
```cpp
  auto cshape = chrono_types::make_shared<ChCollisionShapeBox>(cmat, Xsize, Ysize, Zsize);
  body->AddCollisionShape(cshape);
  body->SetCollide(true);
  
  auto vshape = chrono_types::make_shared<ChVisualShapeBox>(Xsize, Ysize, Zsize);
  body->AddVisualShape(vshape);
```

The function `ChContactable::GetCollisionModel` still exists and can be used to access the collision model of a body *after* it was created and attached to the body (this may be required to set optional parameters on the collision model, such as collision envelope, collision family, or collision family masks).

Finally, note that the function `BuildModel` was obsoleted as there is no need anymore to indicate the end of specification of a collision model; indeed, processing of the generic collision models now occurs at a later time, during initialization of the collision detection system (see below).

**Creation and association of a collision system**

By default, a Chrono physics system has no associated collision system. To enable collision detection and contact force interaction in a Chrono simulation, a collision system (`ChCollisionSystemBullet`, `ChCollisionSystemMulticore`, or potentially some other 3rd-party collision detection system) must be created and associated to the `ChSystem`. This can be done as follows:
```cpp
  auto coll_sys = chrono_types::make_shared<ChCollisionSystemBullet>();
  // set parameters specific to the concrete collision system implementation
  sys->SetCollisionSystem(coll_sys);
```
Alternatively, one of the two collision systems currently available in Chrono can be associated using:
```cpp
  sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
```
or
```cpp
  sys->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
```
If setting a collision system by type, the user can use `ChSystem::GetCollisionSystem` to access the underlying collision system in order to modify its settings or parameters.

Processing of all collision models specified for physics items in a Chrono system is performed (implementation-specific) during initialization of the collision system. This can be done by explicitly invoking `ChCollisionSystem::Initialize()` after all collision models have been specified and before performing any type of physics analysis or simulation.  The `Initialize` function invokes the `BindAll` method of a particular collision system implementation.  In most cases, the `ChCollisionSystem::Initialize()` method need not be explicitly invoked by the user as this is done automatically internally to Chrono before start of an analysis or simulation (and only once).

In addition to `BindAll`, a concrete collision system implementation also provides a function `BindItem` which can be used to request the collision system to process the collision model of a Chrono physics item after the collision system was initialized (for example, in situations where bodies are created at run-time).



## [Changed] Application of terrain forces to vehicle systems

Chrono offers several mechanism for applying external forces to rigid bodies in a multibody system. The first one, and arguably the easiest to implement in user code, relies on so-called accumulators for an external force an moment (accumulated as a resultant wrench at the body center of mass). It is the responsibility of the user to clear the body accumulator before loading updated forces and torques that will be applied next (e.g., over the next dynamics step). While simple to use, this approach is also fragile and a potential source of mistakes; indeed, there is no way to control if the same body accumulators are not being used simultaneously in different parts of the code.  Other mechanisms for applying external forces and torques are (i) creating objects of type `ChForce` (which can represent either a force or a torque) and associating an arbitrary number with any rigid body; and (ii) using objects of type `ChLoad` which associate a so-called "loadable" (e.g., a rigid body or an FEA node) with a load (concentrated, surface-distributed, or volumetric) and managing them in so-called "load containers" in the underlying Chrono system.

Because of the limitations and potential pitfalls of the accumulator approach, this is best left only for use in user code.  As such, all modeling elements in Chrono::Vehicle were switched to using loads and load containers. In the case of vehicle systems, these external loads represent the action of the terrain on the vehicle running gear (wheel spindles or track shoes, for wheeled and tracked vehicles, respectively).

As part of this change, the API for synchronizing the dynamic simulation of tracked vehicles at the beginning of a time step was simplified. Indeed, when using any of the available Chrono terrain systems (rigid or deformable) in a monolithic simulation setting, the interaction between terrain and track shoes leverages the underlying Chrono collision and contact system. As such, there is no need for the user to explicitly pass null terrain interaction forces which were eliminated from the signature of `ChTrackedVehicle::Synchronize` function.  The version that takes lists of such terrain forces (one per track shoe) is still available, for use in co-simulation (such as the Chrono::Vehicle terrain-vehicle co-simulation framework).

## [Changed] Modifications to the HHT integrator

As part of an ongoing set of changes to the implicit integrator based on the Hilber-Hughes-Taylor (HHT) method, `ChTimestepperHHT` was modified to eliminate the so-called "position" formulation (which had no proper theoretical support and was anyway not appropriate for systems that included bodies or nodes with rotational degrees of freedom). The current implementation, only provides the acceleration-level formulation suitable for the 2nd order multibody equations of motion. 

A numerical estimate of the rate of convergence for the underlying Newton solver is now evaluated internally (note that this convergence rate estimate can be calculated only after the third iterations; as such, a value of 1 is set for the first two iterations). The last convergence rate estimate is user-accessible through the `GetEstimatedConvergenceRate` function; together with the reported number of Newton iterations during the last integration step, this can be monitored in user code to provide an indicator of possible numerical difficulties in solving the system dynamics.

## [Added] Modeling hydraulic circuit elements and hydraulic actuators

Initial support was added for modeling elements of hydraulic circuits, including hydraulic pistons, directional valves, and throttle valves. These physics components are modeled with underlying dynamics described by ODEs based on the so-called lumped fluid approach. Two models of hydraulic actuators are also provided. The first one, `ChHydraulicActuator2` uses a hydraulic circuit with two volumes composed of a pump, a tank, a hydraulic cylinder, and a directional valve connected with two hoses. The second actuator, `ChHydraulicActuator3` models a hydraulic circuit with three volumes and also includes a throttle valve (as well as an additional hose). The hydraulic actuators are controlled by providing an actuation function which defines the desired (reference) spool position of the directional valve as a function of time; this reference, specified through a `ChFunction` object, can be either pre-defined for the time interval of interest or else adjusted interactively (for example, using a `ChFunction_Setpoint` that is updated externally from some user input). 

The underlying dynamics of the hydraulic actuator models are described as ODEs (for the current valve spool position and for the relevant pressures in the hydraulic circuit) and are implemented based on the `ChExternalDynamics` functionality.

Hydraulic modeling components can be used coupled with a Chrono mechanical system in a monolithic simulation, or else co-simulated. These two options are illustrated with a simple model of a hydraulically actuated crane in `demo_MBS_hydraulic_crane` and `demo_MBS_hydraulic_crane_cosim`, respectively.

If used in a tight coupling with a Chrono multibody system, a `ChHydraulicActuator` is connected between two points on two different bodies, e.g.,
```cpp
  auto actuator = chrono_types::make_shared<ChHydraulicActuator2>();
  actuator->SetInputFunction(actuation);
  actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
  actuator->Cylinder().SetInitialChamberPressures(3.3e6, 4.4e6);
  actuator->DirectionalValve().SetInitialSpoolPosition(0);
  actuator->Initialize(ground, crane, true, attachment_ground, attachment_crane);  // connect two bodies
  sys.Add(actuator);
```
In this case, the initial and current actuator length is inferred from the states of the connected bodies and the distance between the connection points, while the actuator forces are directly applied to the connected bodies.

If used in a co-simulation setting, the actuator is initialized stand-alone:
```cpp
  auto actuator = chrono_types::make_shared<ChHydraulicActuator2>();
  actuator->SetInputFunction(actuation);
  actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
  actuator->Cylinder().SetInitialChamberPressures(3.3e6, 4.4e6);
  actuator->DirectionalValve().SetInitialSpoolPosition(0);
  actuator->SetActuatorInitialLength(s0);
  actuator->Initialize();                                                        // initialize stand-alone
  sys.Add(actuator);
```
In this case, the current actuator length is provided from the outside, while the force generated by the actuator can be extracted with the `ChHydraulicActuatorBase::GetActuatorForce` function, thus allowing a force-displacement co-simulation setup.

## [Added] Support for modeling components with own dynamics

The new base class `ChExternalDynamics` allows modeling and inclusion in a Chrono system of a physics item that carries its own dynamics, described as a set of Ordinary Differential Equations (ODE). The states of such components are appended to those of the containing system and are integrated simultaneously with the system's equations of motion. These states can be accessed and used coupled with other components.

A user-provided modeling element inherits from `ChExternalDynamics` and defines the ODE initial value problem by implementing, at a minimum, the functions `SetInitialConditions` (to provide the ODE initial conditions) and `CalculateRHS` (to provide the ODE right-hand side function). Optionally, a derived class may also implement `CalculateJac` to provide the JAcobian of the right-hand side function with respect to the ODE states. The Jacobian is used only is the physics component is declared as stiff (by overriding the function `IsStiff`); if a Jacobian function is not provided, a finite-difference approximation is used.

This mechanism can be used to include external, black-box dynamics components into a Chrono simulation (e.g., controllers, actuators, ADAS vehicle components, etc.) and will be extended in the future to also support components with dynamics described as Differential Algebraic Equation (DAE) systems.

A simple illustration of using this new feature is provided in `demo_MBS_external_dynamics` for solving non-stiff and stiff versions of the Van der Pol oscillator.  The full definition of the user-provided derived class in that case is:
```cpp
class VanDerPolODE : public ChExternalDynamics {
  public:
    VanDerPolODE(double mu) : m_mu(mu) {}

    virtual int GetNumStates() const override { return 2; }

    virtual bool IsStiff() const override { return m_mu > 10; }

    virtual void SetInitialConditions(ChVectorDynamic<>& y0) override {
        y0(0) = 2.0;
        y0(1) = 0.0;
    }

    virtual void CalculateRHS(double time,                 // current time
                              const ChVectorDynamic<>& y,  // current ODE states
                              ChVectorDynamic<>& rhs       // output ODE right-hand side vector
                              ) override {
        rhs(0) = y(1);
        rhs(1) = m_mu * (1 - y(0) * y(0)) * y(1) - y(0);
    }

    virtual bool CalculateJac(double time,                   // current time
                              const ChVectorDynamic<>& y,    // current ODE states
                              const ChVectorDynamic<>& rhs,  // current ODE right-hand side vector
                              ChMatrixDynamic<>& J           // output Jacobian matrix
                              ) override {
        // Do not provide Jacobian information if problem not stiff
        if (!IsStiff())
            return false;

        // Generate analytical Jacobian
        J(0, 0) = 0;
        J(0, 1) = 1;
        J(1, 0) = -2 * m_mu * y(0) * y(1) - 1;
        J(1, 1) = m_mu * (1 - y(0) * y(0));
        return true;
    }

  private:
    double m_mu;
};
```


## [Changed] Renamed SPHTerrain and RCCar vehicle classes

For consistency, better suited names were given to the following classes:
- `SPHTerrain` was renamed to `CRMTerrain` (deformable terrain using the Continuous Representation Model, an SPH-based granular formulation)
- `RCCar` was renamed to `ARTcar` (the Autonomy Research Testbed car)

## [Changed] Moved drive mode to automatic transmissions

The concept of a drive mode was moved from `ChTransmission` to `ChAutomaticTransmission` as it is something that only exists for automatic transmissions. This is an API change.

## [Changed] Transmission gear numbering

Reverse gear is now indicated as -1, making room for a neutral gear as 0. Positive numbers are still forward gears. This is a semantic change to the existing API so please review your code as your IDE will most likely not pick up on it. Added to the API is a method to return the highest (forward) gear that is available.

## [Added] Redundant constraints remover
It is quite common to spot models, even in Chrono demos, where the number of constraints are even greater than the number of variables. While this is allowed in Chrono systems, it should be considered a bad practice. Redundant constraints unnecessarily increase the size of the problem, lead to noisy measurements of the link reactions, cause instability to direct solvers.  
Taking care of adding _only_ the required number of constraints is - and always will be - the best option for the user. However, whenever this approach might be too tedious or impractical, it is now possible to ask the system to get rid of those redundant constraints by calling:

`system.RemoveRedundantConstraints()`

It is recommended to:
- set the model to a generic configuration before the call, since there might be cases in which the redundancy of the constraints is due to a particular system configuration;
- initially set the `verbose` to `true` to double check which constraints have been removed.


## [Changed] Serialization expanded and improved
Serialization is now capable of storing and loading back entire `ChSystem`s consistently. Available classes include rigid bodies along with their visual assets (excluding ChVisualShapeFEA) and collision models, links (both ChLinkMate and ChLinkLock families), shafts, system solvers and collision systems. More classes will become available in the next future.

An additional macro called `CH_UPCASTING` now allows to properly handle those classes with multiple inheritance: if a class, *or any of its parent classes*, shows multiple inheritance it is strongly recommended to specify its inheritance scheme with the new `CH_UPCASTING` macro:
```
CH_UPCASTING(Derived, Base1)
CH_UPCASTING(Derived, Base2)
```
Assuming a more complex scheme, e.g.
     C1->D1
    /
A->B
    \
     C2->D2->E
```
CH_UPCASTING(A, B)
CH_UPCASTING(B, C1)
CH_UPCASTING(C1, D1)
CH_UPCASTING(B, C2)
CH_UPCASTING(C2, D2)
CH_UPCASTING(D2, E)
```
Further insights can be found in the documentation of the `CH_UPCASTING` macro. An additional `CH_UPCASTING_SANITIZED` macro is offered for those classes whose type contains characters not valid for a class name. For example `ChContactable_1vars<6>` is handled through the following:  
`CH_UPCASTING_SANITIZED(ChBody, ChContactable_1vars<6>, ChBody_ChContactable_1vars_6)`
 

## [Changed] Rewrite of Pac02 handling tire model

Some of the salient characteristic of the updated Pac02 handling tire model (implemented in the `ChPac02Tire` class) are:
- performs only steady state calculations.
- Fx and Fy can be combined by the Pacejka cosine weighting functions (requires specification of accurate coefficients). 
- the friction ellipsis code has been removed (if the approach based on the friction ellipsis combination method is acceptable, it is better to use the TMEasy or TMsimple tire models which are much simpler and offer the same level of accuracy).
- Pac02 model parameters can be set directly as before, but now can also be specified through an ADAMS/Car-compatible `TIR` file. Note that such TIR files may contain data for alternate tire models (such as Fiala or Pac89); if provided such a file, a Pac02 tire will *not* be initialized.

Setting Pac02 tire parameters through the C++ API is done as before. For example,
```cpp
  m_par.UNLOADED_RADIUS = 0.345;
  m_par.LFZO = 1.0;
```
Note that the names of member variables in the `MFCoeff` structure `m_par` are now in all-caps for consistency with the format of `TIR` files.

The concrete tire subsystem `Pac02Tire` class reads a full specification of a Chrono Pac02 tire from a `JSON` file. There are two ppossibilities:
1. Referencing a `TIR` file. For example:
   ```json
   {
    "Name": "Magic Formula Tire from ADAMS/Car TIR file",
    "Type": "Tire",
    "Template": "Pac02Tire",
    "Mass": 16.0,
    "Inertia": [
        1.021213842086728,
        1.828238395872898,
        1.021213842086728
    ],
    "TIR Specification File" : "VW_microbus/json/mf_185_80R14.tir",
    "Coefficient of Friction": 0.8,
    "Visualization": {
        "Mesh Filename Left":  "VW_microbus/van_tire.obj",
        "Mesh Filename Right": "VW_microbus/van_tire.obj",
        "Width": 0.185
    }
   }
   ```
2. Explicitly setting all tire parameters. For example:
   ```json
   {
    "Name": "Magic Formula Tire from parameters",
    "Type": "Tire",
    "Template": "Pac02Tire",
    "Mass": 16.0,
    "Inertia": [
        1.021213842086728,
        1.828238395872898,
        1.021213842086728
    ],
    "Coefficient of Friction": 0.8,
    "Use Mode" : 3,
    "Dimension": {
        "Unloaded Radius": 0.376,
        "Width": 0.185,
        "Aspect Ratio": 0.85,
        "Rim Radius": 0.1778,
        "Rim Width": 0.17
    },
   "Vertical": {
        "Vertical Stiffness": 332736.8,
        "Vertical Damping": 2307.3,
        "Nominal Wheel Load": 4700.0
    },
    "Scaling Factors": {
        "LFZO" : 1,                         // Scale factor of nominal (rated) load
        "LCX" : 1,                          // Scale factor of Fx shape factor
        "LMUX" : 1,                         // Scale factor of Fx peak friction coefficient
        "LEX" : 1,                          // Scale factor of Fx curvature factor
    // etc...
        "LKYG" : 1,
        "LCZ" : 1                           // Scale factor of vertical stiffness
    },
    "Longitudinal Coefficients": {
        "PCX1" : 1.5587,                    // Shape factor Cfx for longitudinal force         
        "PDX1" : 1.09,                      // Longitudinal friction Mux at Fznom         
        "PDX2" : -0.079328,                 // Variation of friction Mux with load         
        "PDX3" : 9.9376e-006,               // Variation of friction Mux with camber         
    // etc...
        "PTX2" : -0.0014739,                // Variation of SigKap0/Fz with load         
        "PTX3" : 0.03631                    // Variation of SigKap0/Fz with exponent of load         
    },
    "Lateral Coefficients": {
        "PCY1" : 1.4675,                    // Shape factor Cfy for lateral forces         
        "PDY1" : 0.94002,                   // Lateral friction Muy         
        "PDY2" : -0.17669,                  // Variation of friction Muy with load         
        "PDY3" : -0.69602,                  // Variation of friction Muy with squared camber         
    // etc...
        "RVY6" : 0,                         // Variation of Svyk/Muy*Fz with atan(kappa)         
        "PTY1" : 1.8473,                    // Peak value of relaxation length SigAlp0/R0         
        "PTY2" : 1.9465                     // Value of Fz/Fznom where SigAlp0 is extreme         
    },
    "Aligning Coefficients": {
        "QBZ1" : 9.2824,                    // Trail slope factor for trail Bpt at Fznom         
        "QBZ2" : -2.6095,                   // Variation of slope Bpt with load         
        "QBZ3" : -0.86548,                  // Variation of slope Bpt with load squared         
        "QBZ4" : -0.16332,                  // Variation of slope Bpt with camber         
        "QBZ5" : -0.35511,                  // Variation of slope Bpt with absolute camber         
        "QBZ9" : 13.946,                    // Slope factor Br of residual torque Mzr         
        "QBZ10" : 0,                        // Slope factor Br of residual torque Mzr         
    // etc...
        "SSZ1" : 0.026243,                  // Nominal value of s/R0: effect of Fx on Mz         
        "SSZ2" : -0.013391,                 // Variation of distance s/R0 with Fy/Fznom         
        "SSZ3" : 0.3923,                    // Variation of distance s/R0 with camber         
        "SSZ4" : -0.16022,                  // Variation of distance s/R0 with load and camber         
        "QTZ1" : 0.2,                       // Gyration torque constant         
        "MBELT" : 3.5                       // Belt mass of the wheel         
    },
    "Overturning Coefficients": {
        "QSX1" : 0,                         // Lateral force induced overturning moment         
        "QSX2" : 0,                         // Camber induced overturning couple         
        "QSX3" : 0                          // Fy induced overturning couple         
    },
    "Rolling Coefficients": {
        "QSY1"                     : 0.01,  // Rolling resistance torque coefficient         
        "QSY2"                     : 0,     // Rolling resistance torque depending on Fx         
        "QSY3"                     : 0,     // Rolling resistance torque depending on speed         
        "QSY4"                     : 0      // Rolling resistance torque depending on speed ^4         
    },
    "Visualization": {
        "Mesh Filename Left":  "VW_microbus/van_tire.obj",
        "Mesh Filename Right": "VW_microbus/van_tire.obj",
        "Width": 0.185
    }
   }
   ```

Notes:
- not all masses and inertia values are read from the `TIR` file and as such must be set separately. This is because the magic formula has seen numerous updates over the years and some specification files include inertia properties while other do not.
- `TIR` files may include data in units other than SI. The Chrono parser in `Pac02Tire` applies the appropriate conversions.
- when specifying tire parameters explicitly (option 2 above), all units **must** be SI (fortunately, most Pac02 parameters are non-dimensional).
- if a parameter is not explicitly set in the JSON file, it is set to a default value of `0.0`; similarly, scaling factors not explicitly defined in the JSON file are set to a default value of `1.0`.

## [Added] New URDF parser

The miscellaneous parsers provided with Chrono have been reorganized in a separate module, Chrono::Parsers. This module (and associated library) include the previous OpenSim and Adams parsers (previously part of the core Chrono library, as well as the Python parser (previously a separate library built together with the PyChrono wrapper libraries). In addition, the Chrono::Parsers module has a new parser which can populate a Chrono system with a mechanism specified through a [URDF](http://wiki.ros.org/urdf]) (Unified Robotics Description Format) file.

ChParserURDF is implemented based on the canonical ROS URDF parser, `urdfdom` and supports all modeling elements that can be specified through URDF. These include bodies and joints (revolute, prismatic, fixed, or free), visualization geometry, and collision geometry. URDF is limited to specifying only open-loop mechanisms, but additional joints can be added to the Chrono system once the input URDF file was parsed and processed.

Some of the more important capabilities of the ChParserURDF parser are:
- enabling specific joints as actuated (i.e., create Chrono motors instead of kinematic joints);
- support for different types of actuation (position/angle, linear speed/angular speed, or force/torque, depending on whether using a linear or a rotation motor);
- specification of different contact materials on different bodies;
- support for different interpretations of collision meshes (as triangular meshes, convex hulls, or node clouds);
- support for arbitrary actuation functions (on joints converted to Chrono motors).

In addition, a utility class (`ChRobotActuation`) was implemented to allow actuation of robot motors using interpolated data from files. This actuator class allows specifying a series of different phases for a generic robotic mechanism, including phases for assuming a start pose, the cycle phase itself, and a stopping phase to reach a rest pose. The `ChRobotActuation` allows definition of a custom callback for user-defined actions at each phase transition.

To build the Chrono URDF parser when the Chrono::Parsers module is enabled (set `ENABLE_MODULE_PARSERS` to `ON` during CMake configuration), several dependency libraries must be provided. For convenience, bat and shell scripts (for Windows and Linux, respectively) are provided in the Chrono distribution (under `contrib/build-scripts/urdf`).

Two example URDF models are provided: the R2D2 model from the ROS [URDF tutorial](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) and the previously available RoboSimian robot.

See `demo_PARSER_URDF` for an illustration of reading and visualizing a robot specified through URDF. See `demo_PARSER_URDF_RoboSimian` for a more complex example of building and actuating a robot specified through URDF.

## [Added] Support for STL 3D file format

We have added support for creating a triangular mesh (`ChTriangleMeshConnected`) from a **binary** STL () file. This is implemented through the functions `ChTriangleMeshConnected::LoadSTLMesh()` and `ChTriangleMeshConnected::CreateFromSTLFile()`.

Even though the STL format uses a redundant representation in which vertices are replicated for each mesh face, the Chrono reader functions collapse identical vertices, thus providing a minimal mesh representation and connectivity information.

## [Changed] Definition and use of primitive geometric shapes

Specification of low-level geometric shapes and their use as collision and visualization shapes was changed for consistency and a more intuitive API.  The main changes can be described as follows:
  - Geometric shapes with a directional axis (e.g., cylinder, cone, capsule) are always aligned with the Z axis.
  - Size information is now provided through more intuitive quantities (full lengths for a box sides, axis lengths for an ellipsoid, radius and length for a cylinder or a cone, etc).
    Notable exceptions are:
      - a sphere is specified by its radius,
      - a capsule is specified by a radius and the length of its cylindrical portion.
  - Geometric data for a shape contains no information on pose of that shape when used as a collision or visualization shape.  A transform (position and orientation) is specified only when a geometric shape is included in a visualization model or in a collision model. In particular, the specification of a cylinder by the coordinates of its end cap center points was removed. However, convenience functions are provided to facilitate this approach to constructing a cylindrical shape (see below).
  - For classes derived from `ChVisualShape` we provide constructors that take relevant arguments specifying the shape size; as such, access to the low-level underlying geometry (through the `GetGeometry()` functions) is needed only in very special situations.

While these changes affected a lot of the Chrono code base, user code must be updated only in a relatively few places:
  - Specification of `ChVisualShape` objects and inclusion in a visual model:

    Old code (visualization cylinder along Y axis):
    ```cpp
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(2, -0.2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(2, 0.5, 0);
    cyl->GetCylinderGeometry().rad = 0.3;
    cyl->AddMaterial(orange_mat);
    body->AddVisualShape(cyl);
    ```
    New code:
    ```cpp
    auto cyl = chrono_types::make_shared<ChCylinderShape>(0.3, 0.7);
    cyl->AddMaterial(orange_mat);
    body->AddVisualShape(cyl, ChFrame<>(ChVector<>(2, 0.15, 0), Q_from_AngX(CH_C_PI_2)));
    ```

  - Addition of primitive collision shapes to a collision model:
  
    Old code (collision cylinder along Y axis):
    ```cpp
    object->GetCollisionModel()->ClearModel();
    object->GetCollisionModel()->AddCylinder(object_mat, radius, radius, hlen, ChVector<>(0), ChMatrix33<>(1));
    object->GetCollisionModel()->BuildModel();
    ```
    New code:
    ```cpp
    object->GetCollisionModel()->ClearModel();
    object->GetCollisionModel()->AddCylinder(object_mat, radius, 2 * hlen, ChVector<>(0), Q_from_AngX(CH_C_PI_2));
    object->GetCollisionModel()->BuildModel();
    ```

  - Creation of a `ChBodyEasyCylinder`:

    Old code:
    ```cpp
    auto body = chrono_types::make_shared<ChBodyEasyCylinder>(radB, 0.4, 1000, true, false, mat);
    ```
    New code:
    ```cpp
    auto body = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radB, 0.4, 1000, true, false, mat);
    ```

Consult the various Chrono demos for examples of specifying visualization and collision shapes using the new API.

For convenience, the following mechanisms are provided to construct cylinders when the locations of the endcap centers are known:
  - visualization cylinder shape defined through its endcaps -- construct a helper `ChLineSegment` object:
  ```cpp
  geometry::ChLineSegment seg(ChVector<>(0, -(hl - 0.2) * sina, (hl - 0.2) * cosa),
                              ChVector<>(0, -(hl + 0.2) * sina, (hl + 0.2) * cosa));
  auto cyl_2 = chrono_types::make_shared<ChCylinderShape>(0.3, seg.GetLength());
  ground->AddVisualShape(cyl_2, seg.GetFrame());
  ```       
  - collision cylinder shape defined through its end-caps -- use the alternative version of `ChCollisionModel::AddCylinder`:
  ```cpp
  bool AddCylinder(                                   //
        std::shared_ptr<ChMaterialSurface> material,  ///< surface contact material
        double radius,                                ///< radius
        const ChVector<>& p1,                         ///< first end point
        const ChVector<>& p2                          ///< second end point
    );
  ```
  
In conjunction with the above changes to the basic primitive shapes, several other updates were made for a more consistent and intuitive API:
  - all utility functions defined in `utils::ChUtilsCreators.h` were updated to follow the new conventions.
  - the utility function `utils::AddBoxContainer` was modified to construct a box volume with given dimensions centered at the origin of the provided reference frame (previously, the center of the "bottom" wall was at the frame origin).
  - the utility function `fsi::AddBoxContaionerBCE` (previously named `fsi::AddContainerBCE`) was modified to follow the same convention as above.

For users of the `Chrono::Vehicle` module, note that these changes **do not** affect use of any of the vehicle subsystem templates nor do they require any changes to JSON specification files.

## [Changed] Chrono::Vehicle engine and transmission templates

New Chrono::Vehicle templates for the engine and transmission subsystems replace the old powertrain template. The new templates maintain the same modelling capabilities, but allow more flexibility in mixing and matching different models of engines with different transmission models. The coupling between an engine and a transmission is done at the motorshaft, with the engine providing the torque on this shaft and the transmission specifying the angular speed of the shaft. For interfacing with the vehicle system, an aggregate class, ChPowertrainAssembly, manages an engine and transmission and intermediates the coupling with a driveline vehicle subsystem through the driveshaft connecting the transmission to the driveline.

The following engine templates are available:
  - ChEngineShafts - template for modeling an engine using 1-D shaft elements and engine torque-speed maps including maps for engine losses.
  - ChEngineSimpleMap - template for a kinematic engine model based on torque-speed maps.
  - ChEngineSimple - template for a kinematic engine model based on a linear torque-speed dependency.

The following templates for automatic transmissions are available:
  - ChAutomaticTransmissionShafts - template for modelling an automatic transmission using 1-D shaft elements and a torque converter specified through the capacity factor and torque ratio maps.
  - ChAutomaticTransmissionSimpleMap - template for a kinematic model of an automatic transmission using shift maps.

Any of the above engine models can be coupled with either transmission model, as well with any Chrono::Vehicle driveline model (for either a wheeled or tracked vehicle).

While currently only templates for automatic transmissions are implemented, the new code design allows introduction of manual transmissions which will be implemented at a later date.

This code change requires modifications to any existing vehicle model, whether specified through a set of C++ classes providing concrete instantiations of various subsystem templates or else specified through a set of JSON files. Consult the vehicle models in the Chrono::Vehicle models library and the sample JSON specification files distributed with Chrono.


## [Added] New generic template for wheeled suspension subsystems

A new Chrono::Vehicle template for modeling a wheeled vehicle suspension was added. The `ChGenericWheeledSuspension` class permits definition of a suspension subsystem with arbitrary, user-defined topology and sets of bodies, joints, and spring-damper elements. The only elements assumed to always exist in a suspension subsystem are the two spindles and the two axle shafts which connect the spindles to a driveline. A companion class, `GenericWheeledSuspension` allows definition of an arbitrary suspension subsystem based on a JSON specification file. 

In this new template, modeling elements (bodies, joints, TSDAs, and RSDAs) are identified by their names. Positions and orientations are expected to be expressed relative to the suspension subsystem reference frame (except for collision and visualization shapes which must be provided relative to the associated physical modeling element). Any of the physical elements present in a suspension subsystem can be marked as "mirrored" or "non-mirrored"; in the former case, only the element on the left side (positive y) must be defined and two Chrono modeling elements are created, mirrored with respect to the x-z plane. Joints and spring-damper elements (TSDAs or RSDAs) can be connect any two bodies in the subsystem, as well as a suspension body to the chassis, a possible sub-chassis susbsystem, or a possible steering mechanism link. `ChGenericWheeledSuspension` provides a mechanism for identifying these three special bodies that are external to the suspension subsystem.

This new template allows for more flexibility in defining non-standard or concept suspension subsystems without the restrictions imposed by the existing Chrono::Vehicle suspension templates (in terms of the number of bodies, joints, force elements, and their connectivity). In fact, any of the existing suspension templates can be replicated in the new framework and two examples are provided: the `HMMWV_DoubleWishboneFront_replica.json` file contains an exact replica of the HMMWV front double wishbone suspension defined in `HMMWV_DoubleWishboneFront.json`, while `UAZBUS_FrontSAELeafspringAxle_replica.json` replicates the front suspension defined in `UAZBUS_FrontSAELeafspringAxle.json`; the latter example illustrates the definition of a suspension subsystem that includes both mirrored and non-mirrored components.

See `demo_VEH_WheeledVehicle` and `demo_VEH_SuspensionTestRig` which can be modified appropriately to use a HMMWV or UAZ vehicle with the new suspension specifications.

## [Changed] CMake configuration and utility build scripts

For consistency, the following changes were made to some of the Chrono CMake configuration scripts:
  - Chrono::Irrlicht module: `IRRLICHT_INSTALL_DIR` replaces the old IRRLICHT_ROOT
  - Chrono::Multicore module: `BLAZE_INSTALL_DIR` replaces the old BLAZE_DIR
  - Chrono::Vehicle module: `IRRKLANG_INSTALL_DIR` and `IRRKLANG_LIBRARY` replace the old CH_IRRKLANG_SDKDIR and CH_IRRKLANGLIB, respectively
  - Chrono::Synchrono module: `fastrtps_INSTALL_DIR` replaces the old fastrtps_ROOT
  - Chrono::OpenGL module: finding the necessary GL dependencies (GLEW and GLFW) now relies on CMake project configuration scripts for these libraries. As such, unless automatically detected, the user must set the CMake variables `GLEW_DIR` and `GLFW3_DIR`. For systems where default packages for GLEW and GLFW do *not* install their CMake project configuration scripts (e.g., Ubuntu), see below for information on building these libraries from sources.  For the headers-only GLM dependency, the user must set `GLM_INCLUDE_DIR`.

To help the configuration of certain Chrono modules, we added several scripts (for both Windows and Linux) to build dependencies from sources. The following scripts are available under contrib/build-scripts:
  - `opencrg/buildOpenCRG.bat` and `opencrg/buildOpenCRG.sh` can be used to build and install the OpenCRG library. Optionally, the script can first download the sources for version 1.1.2 of OpenCRG from a GitHub repository.
  - `opengl/buildGL.bat` and `opengl/buildGL.sh` can be used to build and install the GLEW, GLFW, and GLM dependencies for Chrono::OpenGL and Chrono::Sensor. The sources for these libraries are (optionally) downloaded from their respective SourceForce repositories. These scripts configure, build, and install all 3 necessary GL libraries under a common user-specified directory.
  - `vsg/buildVSG.bat` and `vsg/buildVSG.sh` can be used to build and install all dependencies required for the Chrono::VSG module. Their sources can be optionally be downloaded from their respective GitHub repositories. These scripts are provided as a more flexible and robust alternative to the vsgFramework approach to installing the necessary VSG dependencies. The buildVSG scripts install all necessary VSG libraries (VulkanSceneGraph, vsgXchange, vsgImGui, ImGui, ImPlot, vsgExamples, and assimp) under a common, user-specified directory.
Follow the instructions listed in comments at the top of each one of the above build scripts.

Finally, the scripts `buildChrono.bat` (for Windows), `buildChrono.sh` (for Linux), and `buildChronoMac.sh` (for MacOS) are provided as examples of CMake configuration for the various Chrono modules. They should be copied to a different directory and modified to reflect the setup on the user machine and to enable only those Chrono modules of interest. When executed, these scripts run the CMake configuration of Chrono and generate the files for building the Chrono libraries, as appropriate on each platform.


## [Added] SPHTerrain - continuum representation method for deformable terrain

A new Chrono::Vehicle terrain class, SPHTerrain, was added to model deformable terrain using the Continuum Representation Method (CRM), an SPH-based formulation that leverages the Chrono::FSI module.  An SPHTerrain can be created from data files with positions of SPH particles and BCE markers read from data files (these positions are assumed to be provided on an integer grid, in multiples of the initial separation of SPH particles) or from a height-map image file.  In addition, this type of terrain permits definition of rigid obstacles that may be embedded, partially or fully, in the terrain volume (to model, for example, embedded rocks); currently, rigid obstacles must be specified with trimesh geometry read from a Wavefront OBJ file.  Run-time visualization is supported through `ChFsiVisualizationVSG` or `ChFsiVisualizationGL` which leverage the Chrono::VSG and Chrono::OpenGL modules, respectively. See `demo_VEH_SPHTerrain_Obstacles`, `demo_VEH_SPHTerrain_WheeledVehicle`, and `demo_VEH_SPHTerrain_Viper`.

## [Added] TMSimple tire model

A new tire model (`ChTMsimple`) was added to Chrono::Vehicle. This tire model is of "force element" type and shares part of its formulation with the TMeasy model (both of these tire models were developed by Wolfgang Hirschberg from TU Graz in Austria). The goal of TMsimple is to provide a simple handling tire model with fewer parameters than TMeasy while still providing realistic (albeit reduced) functionality.

The TMsimple model
- calculated horizontal patch forces based on single functions (whereas TMeasy requires three piece-wise definitions)
- considers degressive influence of the vertical force Fz
- calculates rolling resistance

TMeasy requires 5 parameters to define its basic function for Fx and Fy: (1) slope at zero, (2) slip at force maximum, (3) maximal force, (4) slip at sliding initiation, and (5) sliding force. In contrast, TMsimple needs only 3 parameters: (1) slope, (2) maximal force, and (3) force at infinite slip.  A complete parameter set for Fx(sx,Fz) and Fy(sy,Fz) has 20 items for TMeasy and only 12 for TMsimple. Chrono's TMsimple implementation has a new stand-still/low speed algorithm for friction forces. It is not part of TMsimple itself, and could be adapted to other handling tire models as well.

## [Added] Blender plug-in for post-process visualization

A new tool has been developed. It is an add-on for the [Blender](http://blender.org) rendering/modeling/animation software, that allows importing Chrono simulation in the GUI of Blender. From the C++ side, the only requirement is using some export functions of the POSTPROCESS module. This aims at replacing the old POVray post-processing pipeline.

- Interactive 3D navigation of the scenes, and timeline scrubbing.
- Allows rendering of high-quality photo-realistic animations, using the Cycles physically-based unbiased path tracer that is available in Blender.
- The user can optionally modify the Chrono assets, once imported, by attaching custom materials, special FXs, more detailed meshes, etc. 
- Visualization of auxiliary references (center of masses, link markers etc)
- False color rendering of mesh attributes and glyph attributes, using colormaps
- Speed optimizations for the ChParticleCloud shapes

Details on this new tool is available at [the Chrono::Blender page](https://api.projectchrono.org/development/introduction_chrono_blender.html) on the ProjectChrono.org website.


## [Added] VSG-based run-time visualization module

A new module (`Chrono::VSG`) was added to provide an alternative for run-time visualization. This module uses VulkanSceneGraph ([VSG](https://vsg-dev.github.io/VulkanSceneGraph/)), a new cross-platform, high-performance scene graph library built upon the Vulkan graphics/compute API. Chrono::VSG implements most of the functionality expected by the base Chrono visual system; currently, missing support includes FEA and modal analysis visualizations (work in progress). Except for this, the `ChVisualSystemVSG` is interchangeable with `ChVisualSystemIrrlicht` with minor changes to user code. See for example the various Chrono MBS and vehicle demos that have been modified to work with either run-time visualization system.  

Unlike Irrlicht, VulkanSceneGraph provides a modern scene graph library which is under active development. Besides the many features provided by the core VSG library, the GUI system provided by the companion [vsgImGui](https://github.com/vsg-dev/vsgImGui) project offers the full set of capabilities of [ImGui](https://github.com/ocornut/imgui). The basic UI elements already exposed in `ChVisualSystemVSG` and `ChVehicleVisualSystemVSG` will be further expanded.

Simultaneous with the introduction of Chrono::VSG, we have modified the `ChVisualSystem` API (and reflected these changes in both `ChVisualSystemIrrlicht` and `ChVisualSystemVSG`) to provided new capabilities, such as the definition and rendering of visual models that are not associated with a Chrono physics item.

## [Changed] Support for linear and nonlinear vehicle force elements

The various pre-defined functors for use with TSDA and RSDA elements in vehicle models have been refactored for consistency. These include linear and nonlinear springs, dampers, and spring-damper force elements with optional bump stops (see the definitions in `src/chrono_vehicle/ChSubsysDefs.h`).  In addition, a new TSDA functor, `MapSpringDamper` allows for the definition of a general non-linear spring-damper (depending on both deformation and velocity) specified through bi-linear interpolation of tabular data.

All the pre-defined TSDA and RSDA functors can be specified in JSON files; consult the various JSON files for wheeled vehicle suspension systems in the Chrono data directory.

# Release 8.0.0 (2022-12-21)

## [Added] Chrono::Sensor features and updates

**Updates and Bug Fixes**

 - Upgraded to OptiX 7.5, requiring NVIDIA drive 515.X or newer
 - Fixed motion blur for cameras
 - Improve consistency of FOV model for camera and segmentation camera
 - Fix undefined behavior associated with zero-time transforms on ampere GPUs
 - Changed the intensity-dependent noise model to be parameterized by variance variance rather than standard deviation to allow negative correlation

**New Features**

 - Added fog to camera. This uses an exponential blending function with scattering coefficient. Can be enabled on a per-camera basis in the camera constructed. Parameters for fog scattering, color, and max visible distance are set in `ChScene`: `SetFogScattering(float coefficient)`, `SetFogColor(ChVector<float> color)` and `SetFogScatteringFromDistance(float distance)`.
 - Added radial distortion model (standard model in Matlab). The input parameters are the correction parameters that can be calibrated from distorted images. The FOV lens model and radial model use the effective FOV parameter. Instead, they use the calibrated parameter as would be obtained by calibrating the focal distance.
  - Radial model set as: `CameraLensModelType::RADIAL` in the camera
  - Parameters configured as `ChCameraSensor::SetRadialLensParameters(ChVector<float> params)`
 - Allow Chrono::Sensor to be built when GLFW library and headers are not found



## [Fixed] Closed-loop vehicle paths

The treatment of closed-loop Bezier curves (used for path-following vehicle lateral controllers) and the associated curve tracker was improved and fixed.  With this change, the flag indicating whether a path is open or closed is set in the constructor of `ChBezierCurve` and encapsulating objects (`ChBezierCurveTracker` and the different path-following vehicle driver models) query the underlying path.

If a Bezier curve is declared as closed, it is internally modified to add a new point as needed (coincident with the first one). If no Bezier control points are specified, the Bezier curve corresponding to the equivalent piece-wise cubic spline is constructed using C1 continuity conditions at the closing point.

## [Added] Miscellaneous Chrono::Vehicle extensions

**Wheeled vehicles**

- Added option to specify camber and toe angles during suspension subsystem construction. 

  A derived suspension class can override the functions `getCamberAngle` and `getToeAngle` to provide these angles expressed in radians). For a suspension subsystem specified through a JSON file, set the 'Camber Angle (deg)' and 'Toe Angle (deg)' values expressed in degrees. Note that all current Chrono::Vehicle models use default values of zero camber and toe angles.

 - Modified the JSON schema for various vehicle subsystem specification so that all angles that are expected to be provided in radians include the string "(deg)" in the corresponding JSON key name. 

   **Note**: this change requires users to update their JSON specification files! 

- Added option to specify preloads for tracked vehicle suspensions, as well as for the tensioner in `ChTranslationalIdler`.

**Tracked vehicles**

- Renamed the base class for a track vehicle suspension `ChTrackSuspension`. Two different suspension templates are provided: `ChTranslationalDamperSuspension` uses a translational damper, while `ChRotationalDamperSuspension` uses a rotational damper.

- Use a single set of classes to define all tracked vehicle wheels (road-wheels, idler wheels, and rollers). Two types of track wheels are derived from the base class `ChTrackWheel`: the first one, `ChDoubleTrackWheel` assumes that track shoes have a central guiding pin, while the second one `ChSingleTrackWheel` works in conjunction with track shoes with lateral guiding pins. Track wheels are used as members of the `ChTrackSuspension` and `ChIdler` base classes. 

  The `ChRoller` class was obsoleted since a roller can be modeled with one of the available track wheels.

- Added a new type of idler mechanism, `ChDistanceIdler` which is modeled with a connector arm pinned to the chassis. The idler wheel is attached to the connector arm with a revolute joint. A translational actuator dictates the relative position of the connector arm relative to the chassis, specified by the length of the actuator. 

  The previous type of idler template is now named `ChTranslationalIdler`.

- Added option to specify preloads for tracked vehicle suspensions, as well as for the tensioner in `ChTranslationalIdler`.

- Add alternative model for a double-pin track shoe which uses a single body to model the connectors between shoes. For track assemblies that use kinematic joints, this model provides the same track kinematics and dynamics with fewer bodies and joints and without additional complications due to redundant constraints. The previous model, using two distinct connector bodies between two consecutive shoes, is appropriate for the case where kinematic joints are replaced with bushings. The double-pin track shoe topology is specified during construction, using the enum `DoublePinTrackShoeType ` which can be either `TWO_CONNECTORS` or `ONE_CONNECTOR`.

## [Changed] Chrono::FSI API changes

The public API of Chrono::FSI was further refined to better encapsulate and hide the underlying CUDA implementation from the user.  
- Only two header files need to be included in most Chrono::FSI user programs: `ChSystemFsi.h` which contains the definition of the top-level FSI system class and `ChDefinitionsFsi.h` which defines various enumerations for problem settings and solution methods.
- Optionally, the `ChVisualizationFsi.h` header can be included to allow access to OpenGL-based run-time visualization.
- Simulation parameters (defining the problem and the solution method) can be still be set as before through a specification file in JSON format.  However, each parameter can also be programmatically set through various `Set***` methods of `ChSystemFsi`.
- The order in which various parameters are set is now arbitrary.  All dependent calculations are performed in the `ChSystemFsi::Initialize()` function which **must** be invoked once all setup is completed and before the start of the simulation loop.
- The set of accessor `Get***` functions of `ChSystemFsi` was enlarged to provide access to various simulation parameters (possibly defined in an input JSON file). 
- All public API functions in `ChSystemFsi` and `ChVisualizationFsi` only use C++ base types and Chrono types (such as ChVector<>, ChQuaternion<>, etc.)
- The only disk output from the Chrono::FSI module is optionally enabled by calling `ChSystemFsi::SetOutputDirectory()`. It is the caller's responsibility to ensure that the specified directory exists.  If Chrono::FSI output is enabled, various files with state and forces on rigid- and flex-body BCE markers are saved in an `fsi/` subdirectory of the specified output directory. 

The new optional run-time visualization support for Chrono::FSI simulation requires that the Chrono::OpenGL module is enabled and available. Visualization of SPH particles, boundary BCE markers, rigid-body BCE markers, and flex-body BCE markers can be enabled or disabled individually.  SPH particles are rendered with a point cloud; boundary BCE markers are rendered as boxes (of size equal to the initial particle spacing); solid-body BCE markers are rendered as spheres (of diameter equal to the initial particle spacing). See the various Chrono FSI demos for usage. Note that enabling run-time visualization adds the additional cost of transferring marker positions every time the simulation is rendered.

## [Added] User-defined SMC contact force calculation

A mechanism was added for overriding the default contact force calculation for a collision pair in an SMC system. The user must supply a class derived from `ChSystemSMC::ChContactForceSMC` and register it through a call to `ChSystemSMC::SetContactForceAlgorithm`.

The user is responsible to implement the virtual function 
```cpp
        virtual ChVector<> CalculateForce(
            const ChSystemSMC& sys,             ///< containing system
            const ChVector<>& normal_dir,       ///< normal contact direction (expressed in global frame)
            const ChVector<>& p1,               ///< most penetrated point on obj1 (expressed in global frame)
            const ChVector<>& p2,               ///< most penetrated point on obj2 (expressed in global frame)
            const ChVector<>& vel1,             ///< velocity of contact point on obj1 (expressed in global frame)
            const ChVector<>& vel2,             ///< velocity of contact point on obj2 (expressed in global frame)
            const ChMaterialCompositeSMC& mat,  ///< composite material for contact pair
            double delta,                       ///< overlap in normal direction
            double eff_radius,                  ///< effective radius of curvature at contact
            double mass1,                       ///< mass of obj1
            double mass2                        ///< mass of obj2
            ) const = 0;
```
which should return the contact force (resultant of the normal and tangential components) for an interaction between two penetrated shapes, given the geometric quantities for the collision.

The default implementation of the SMC contact force calculation is implemented in `ChContactSMC.h` and depends on various settings specified at the ChSystemSMC level (such as normal force model, tangential force model, use of material physical properties, etc).

For an example of overriding the default Chrono behavior, see `demo_IRR_callbackSMC.cpp`.

## [Changed] Redesigned run-time visualization system

The entire mechanism for defining visualization models, shapes, and materials, as well as constructing and attaching a run-time visualization system to a Chrono system was redefined for more flexibility and to allow plugging in alternative rendering engines.  The new class hierarchy allows sharing of visualization models among different physics items, visualization shapes among different models, and visualization materials among different shapes.

- A visualization material (`ChVisualMaterial`) defines colors (diffuse, ambient, specular, and emissive), textures, and other related properties.
- A visualization shape (`ChVisualShape`) is a geometric shape (primitive, curve, surface, or triangular mesh) with one or more associated visualization materials. If a shape has no associated material, a default material is used.
- A visualization model (`ChVisualModel`) is an aggregate of (pointers to) shapes and a transform which specifies the shape position relative to the model reference frame. Visualization shapes in a model are maintained in a vector of `ShapeInstance` (which is simply a typedef for a pair containing a shared pointer to a `ChVisualShape` and a `ChFrame`). Note that, currently a visualization model instance cannot be placed inside another visualization model, but that may be added in the future.
- A visualization model instance (`ChVisualModelInstance`) is a reference to a visualization model with an associated physics item.  A physics item may have an associated visualization model instance.  

`ChVisualSystem` defines a base class for possible concrete run-time visualization systems and imposes minimal common functionality. A ChSystem is attached to a visual system using `ChVisualSystem::AttachSystem`. The Chrono physics system will then trigger automatic updates to the visualization system as needed, depending on the particular type of analysis being conducted. The visualization system is set up in such a way that derived classes may allow simultaneous rendering of multiple Chrono systems; currently only ChVisualSystemOpenGL supports this feature.

**Defining visualization models**

The new mechanism for defining visualization shapes and materials for a Chrono physics item is illustrated in the following typical sequence:
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
The suggested mechanism is to define a concrete visualization system (derived from `ChVisualSystem`) and attach it to the Chrono system. Currently, an Irrlicht-based and an OpenGL-based visualization systems are provided through `ChVisualSystemIrrlicht` and `ChVisualSystemOpenGL`, respectively. These object replace the now obsolete ChIrrApp and ChOpenGLWindow. 

A typical sequence for creating and attaching an Irrlicht-based visualization system to a Chrono simulation is illustrated below:
```cpp
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("Chrono::Irrlicht visualization");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddCamera(ChVector<>(1, 2, 3));
    vis.AddTypicalLights();
    vis.AttachSystem(&sys);
```

Notes:
- Various parameters (such as windows size, windows title, camera vertical direction) can be set with various `Set***` methods.  These must be called before the visual system is initialized with ChVisualSystemIrrlicht::Initialize().
- After the call to `Initialize`, additional rendering elements (such as lights, a sky box, a camera) can be added to the visualization system with various `Add***` methods.
- Once the visual system is initialized **and** attached to the Chrono system, all currently defined visual models are processed and Irrlicht nodes created.  
- If visual models are created at a later time, these must be converted to Irrlicht nodes through a call to `ChVisualSystemIrrlicht::BindAll()` (to process all visual models in the Chrono system) or to `ChVisualSystemIrrlicht::BindItem` (to process the visual model for the specified Chrono physics item).

Additional rendering options can be enabled with calls to `Enable***` methods which must be made only after the visualization system was initialized and attached to a Chrono system.  These options include enabling various ways of rendering collision and contact information and body or link reference frames.

A typical simulation loop with Irrlicht-based run-time visualization has the form:
```cpp
    while (vis.Run()) {
        vis.BeginScene();
        vis.Render();
        // make additional Irrlicht-based rendering calls, for example to display a grid:
        irrlicht::tools::drawGrid(vis, 0.5, 0.5, 12, 12,
                                  ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(CH_C_PI_2)),
                                  ChColor(80, 110, 110), true);
        vis.EndScene();
        sys.DoStepDynamics(step_size);
    }
```

The Irrlicht visualization system also provides a GUI (displayed using the `i` key during simulation) which allows changing rendering options at run-time.

See the various Chrono demos (in `src/demos/irrlicht/`) for different ways of using the new visualization system mechanism in Chrono.

The OpenGL-based visualization system can be used effectively in the same way as ChVisualSystemIrrlicht.

Finally, note that all functions in the public API were changed to use only Chrono data types. In other words, user code need not use types from the Irrlicht or OpenGL namespaces.

**Vehicle-specific visualization system**

While a Chrono::Vehicle simulation can be rendered like any other Chrono system simulation (using either `ChVisualSystemIrrlicht` or `ChVisualSystemOpenGL`), customized derived classes are provided for Irrlicht-based rendering.  These classes provide additional rendering options specific to vehicle systems, including vehicle state information overlay text.  Use `ChWheeledVehicleVisualSystemIrrlicht` for wheeled vehicles and `ChTrackedVehicleVisualSystemIrrlicht` for tracked vehicles.

To enable the vehicle-specific features, attach the vehicle system to the visualization system, using `ChVehicleVisualSystem::AttachVehicle`.

See demos under `src/demos/vehicle/`. 

## [Changed] Vehicle inertia properties

The underlying mechanism for setting and querying inertia properties (mass, COM location, and inertia matrix) for vehicle systems and subsystems was redesign for consistency.  At the user API level, this change is reflected through a uniform manner to hoe these quantities are reported.

Any vehicle subsystem (of type `ChPart`), as well as any vehicle system (`ChWheeledVehicle` or `ChTrackedVehicle`) provide the following set of accessor methods:
- `GetMass()` returns the mass of the (sub)system.  In the case of a vehicle, this includes the mass of all vehicle subsystems.  Furthermore, the mass of a wheeled vehicle includes the mass of the tires.
- `GetCOMFrame()` returns the current COM (centroidal) frame.  This frame is relative to and expressed in the reference frame of the part or of the vehicle.
- `GetInertia()` returns the current inertia matrix (that is the articulated inertia).  The reported inertia matrix is given with respect to the centroidal frame of the part or vehicle.

In addition, a `ChPart` or `ChVehicle` also provide a method `GetTransform()` which returns the vehicle transform (translation and orientation encapsulated in a `ChFrame`) relative to the global (absolute) frame. Recall that, by convention, the vehicle reference frame is that of its main chassis.

## [Changed] CMake project configuration script

The CMake script `ChronoConfig.cmake`, generated automatically during Chrono CMake configuration and used in configuring third-party applications that depend on Chrono (via calls to `find_project(Chrono ...)`) was modified to produce the compiler and linker flags in CMake list variables (as opposed to space-separated strings as before).  The variables affected by this change are `CHRONO_CXX_FLAGS`, `CHRONO_C_FLAGS`, and `CHRONO_LINKER_FLAGS`.

This allows use of modern CMake in the configuration scripts for such an external project.  See the example in the `template_project/` directory in the Chrono distribution:
```
target_compile_definitions(myexe PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(myexe PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(myexe PUBLIC ${CHRONO_LINKER_FLAGS})
```

## [Changed] Right-handed frames in Chrono::Irrlicht

The Irrlicht library, wrapped in the Chrono::Irrlicht run-time visualization library uses the DirectX convention of left-handed frames.  This has been a long standing source of confusion for all Chrono users since Chrono simulations (always conducted using right-handed frames) were "mirrored" during rendering.

This set of changes forces Chrono::Irrlicht to use right-hand projection matrices resulting in renderings that are consistent with the underlying models and simulations.  All changes are internal and transparent to the user.  

We took this opportunity to make a small set of minor API changes, most of them simple function renames:
- ChIrrApp::AddTypicalLogo() was renamed to ChIrrApp::AddLogo().
- ChIrrApp::AddTypicalCamera() was renamed to ChIrrApp::AddCamera().
- ChIrrApp::AddTypicalSky() was renamed to ChIrrApp::AddSkyBox().
- ChIrrApp::AddTypicalLights() was changed to always construct two point lights with default settings (positions, radii, and colors).  The positions of these lights are different for a Y or Z camera vertical direction.  A user interested in changing the settings of the default lights should use the function ChIrrApp::AddLight() which allows specifying position, radius, and color.
- ChVehicleIrrApp::SetSkyBox() was obsoleted (the Chrono sky box is automatically added).

## [Added] Modal analysis module

A new module `MODULE_MODAL` has been added. The module uses an external dependency (the [Spectra](https://spectralib.org/) library for eigenvalue computation). 
Follow the Chrono::Modal installation guide for instructions on how to enable it.

The new class `ChModalAssembly` offer three main functionalities:

- **undamped modal analysis** of all the system being created within the sub assembly will be obtained. The modes and frequencies can be also displayed interactively if using the Irrlicht visualization system. 
  - The sub-assembly can also contain constraints between its sub parts. 
  - Rigid modes (for free-free structures) are supported
  - A custom generalized, sparse, constrained eigenvalue solver of Krylov-Schur type allows the computation of only the n lower modes. This allows handling large FEA systems. 
  
- **damped (complex) modal analysis** of the subsystem: this is like the previous case, but damping matrix is used too, hence obtaining complex eigenvalues/eigenvectors. Damping factors for the modes are output too, indicating stability or instability. *NOTE: while we wait that Spectra will enable complex eigenvalues in Krylov-Schur, a more conventional solver is used, that is not sparse - hence requiring more time and memory*

- **modal reduction** of the sub-assembly. Example of a scenario where this is useful: you have a tower modeled with thousands of finite elements, but you are just interested in the small oscillations of its tip, because you will mount a windmill on its tip. If you simulate thousands of finite elements just for this purpose, you waste CPU time, hence a modal reduction of the tower will discard all the DOFs of the finite elements and represent the overall behavior of the tower using just few modal shapes (ex. fore aft bending, lateral bending, etc.), with extreme CPU performance at the cost of a small reduction of fidelity.
  - Bodies and FEA nodes can be added to the sub-assembly as *internal*  or *boundary* interface nodes. Later one can call `ChModalAssembly::SwitchModalReductionON(int n_modes)` to replace the complexity of the internal nodes with few `n_modes` modal coordinates.
  - Boundary interface nodes can be connected to the rest of the multibody system as usual, using constraints, forces, etc.
  - Internal constraints can be used between internal nodes. Their effect too will be condensed in the modal reduction.
  - *NOTE*: at the moment only linear dynamics is supported for the sub-assembly, in the sense that the sub-assembly cannot withstand large rotations, ex. in a helicopter blade. Future developments will address this


## [Added] Callback mechanism for collision debug visualization

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



## [Changed] Translational and rotational spring-damper-actuators

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

## [Changed] Refactor Chrono::Vehicle suspension test rigs

The wheeled vehicle suspension test rig (STR) was modified to accept an arbitrary number of tested axles from any given vehicle.

The new STR will create posts / pushrods for all spindles (left and right) from all axles specified as "test axles".
Like before, one can construct an STR from a given vehicle (from one of the models in the Chrono vehicle models library or else created from a JSON specification file) or else from a JSON specification file for an STR.  However, the latter approach will now construct the entire vehicle (specified though a vehicle JSON file) but include only a user-specified subset of its axles for testing.
Note that this is not a limitation because Chrono::Vehicle was also modified to allow specification in a JSON file of a stripped-down vehicle model which need not include a driveline nor a steering mechanism and may even define a single axle.

Additional vehicle subsystems (such as steering mechanisms or sub-chassis components) can be adding to either type of STR (`ChSuspensionTestRigPlatform` or `ChSuspensionTestRigPushrod`) using the functions `IncludeSteeringMechanism` and `IncludeSubchassis`. This simply means that: (i) run-time visualization of the additional subsystem can be enabled and (ii) the additional subsystem is included in the rig output (if that is enabled).
The associated vehicle is initialized with its chassis fixed and its driveline automatically disconnected. Simulation of the test rig (through the function `ChSuspensionTestRig::Advance`) performs a simulation of the entire vehicle with all its components, but vehicle subsystems not explicitly included in testing are invisible and do not participate in any output.

See `demo_VEH_SuspensionTestRig` for various examples and options, and look at the JSON files used in that demo for changes in their formats.

Note also that the format for a data file with STR actuation information (used by a ChDataDriverSTR) was modified by moving the steering input in the 2nd column.
In other words, each line of this ASCII file should now contain:<br>
`    time  steering_input  left_post_0  right_post_0 left_post_1 right_post_1 …`

# Release 7.0.3 (2022-04-17)

## [Fixed]

- SIMD detection is combined into one CMake script
- Fixed SIMD feature detection with Clang, allowing support for Apple-M1 and generic AArch64 CPUs

# Release 7.0.2 (2022-04-03)

## [Fixed]

- Fixed bug in ANCF shells 3443 and 3883 where the incorrect Gauss quadrature weights and Jacobian elements were used when multiple layers of different sizes are defined
- Fixed bug where the active flag for a sub-block of DOFs for a ChBody was incorrectly set
- Updates to the continuous integration scripts

# Release 7.0.1 (2022-01-07)

## [Fixed]

- Fixed Chrono::Sensor class export (Windows)
- Fixed bug in ChPovRay related to processing of OBJ files
- Fixed demo program in sample project for vehicle co-simulation
- Fixed setting of MPI linker flags in CMake project configuration script

# Release 7.0.0 (2021-11-15)

## [Added] DDS communicator in Chrono::Synchrono module

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

## [Added] New terramechanics co-simulation module

This new module provides support for co-simulating various Chrono models of ground wheeled vehicles.  This framework implements an explicit co-simulation model (of force-displacement type) and uses an MPI layer for exchanging data between the participant nodes.

The co-simulation framework was designed to support:
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


## [Changed] Chrono::Fsi API redesign

For consistency with the main Chrono module and other optional Chrono modules, the Chrono::FSI API was changed as follows: 

- The user's interaction with the Chrono::FSI module was streamlined by exposing in the public API a single Chrono::FSI system object (of type `ChSystemFsi` ) and hiding the underlying implementation in a private class. 
- User code only needs to include one Chrono::FSI header in their project, namely `chrono_fsi/ChSystemFsi.h` and need not include any of the utility header files from `utils/`.
- Users can use standard C++ types to declare a scalar, and use Chrono types (`ChVector`, `ChQuaternion`, etc) to declare vectors, quaternions, etc. 
- The initialization of the parameters from a JSON file was changed from fsi::utils::ParseJSON() to `myFsiSystem.SetSimParameter()`, assuming the user has created an FSI system `myFsiSystem`. 
- A new function was added to set periodic boundary condition: `ChSystemFsi::SetBoundaries()`. 
- The function used to finalize the subdomains was changed from fsi::utils::FinalizeDomain() to `ChSystemFsi::SetSubDomain()`.
- The function used to set the output directory was changed from utils::PrepareOutputDir() to `ChSystemFsi::SetFsiOutputDir()`.
- The function used to add SPH particles was changed from myFsiSystem.GetDataManager()->AddSphMarker() to `ChSystem::AddSphMarker()`. 
- The functions used to add BCE particles were changed along the same lines; for instance, to add BCE particles for a cylinder, use `ChSystemFsi::AddBceCylinder()`. 
- The function used to output data was changed from fsi::utils::PrintToFile() to `ChSystemFsi::PrintParticleToFile()`. 

See the updated FSI demo programs for usage of the new Chrono::FSI API.

**Added - Option to build Chrono::FSI in single precision**

- Users can optionally configure Chrono::FSI in single precision by unsetting the CMake variable `USE_FSI_DOUBLE`
- By default, Chrono::FSI is configured and built in double precision
- Users should be careful opting for single precision as this can adversely impact simulation results


## [Changed] Sensor to improve performance and added features 

**Changed - Optix 7.2 as Dependency:**
 - Upgraded to Optix 7.2 from 6.5. 7.2 (exactly) is the only version supported.

**Changed - Refactored sensor code:**
 - sensors have been moved to `src/chrono_sensor/sensors/` to cleanup directory structure
 - all OptiX-dependent code was moved to `src/chrono_sensor/optix` to consolidate the dependency

**Changed - IMU to accelerometer and gyroscope:**
 - Split the IMU sensor into its components (ChAccelerometerSensor and ChGyroscopeSensor) to facilitate additional sensors. Using both sensors together with same update rate will produce the same behavior as the original IMU. These sensors are still maintained under `ChIMUSensor.h and ChIMUSensor.cpp`
  ```cpp
  ChAccelerometerSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose, std::shared_ptr<ChNoiseModel> noise_model);

  ChGyroscopeSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose, std::shared_ptr<ChNoiseModel> noise_model);
  ```
**Added - Magnetometer:**
 - Added magnetometer sensor alongside accelerometer and gyroscope. Sensor is maintained in `ChIMUSensor.h` and `ChIMUSensor.cpp`. 
 - Returns a magnetic field strength vector based on the orientation of the sensor, and the GPS location of the sensor and simulation.
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
 - Removed the ability to add instanced objects explicitly.
 - Recommended instancing is to create single ChTriangleMeshConnected, then adding that with many scales (using ChTriangleMeshShape) and positions (using ChBody).

**Changed - Shaders for visualization:**
 - Improved the material shaders to support physically-based materials and Phong materials in the same scene. 
 - Shading calls do NOT change API, but WILL be visible on objects.
 - Expanded parameters contained in `chrono::ChVisualMaterial` to include metallic, roughness, and other textures as well as whether to use a specular or metalic workflow. Will be detected for meshes loaded from file.

**Added - Global Illumination with OptiX Denoiser:**
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
- A radar sensor was added, with the initial version similar to lidar. Will return set of points that include range, azimuth, elevation, doppler velocity, amplitude of detection, and object id used for clustering
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

## [Changed] ANCF element improvements and additions

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

- For `ChElementShellANCF_3443`, a skin at the mid-surface is used just like `ChElementShellANCF_3423` and `ChElementShellANCF_3443`.

- For `ChElementHexaANCF_3813` and `ChElementHexaANCF_3843`, a linear quadrilateral face is added to the free faces just like `ChElementHexaANCF_3813_9`.


## [Added] New Chrono::Vehicle features

1. A mechanism was added to allow replacing selected kinematic joints with bushings in various Chrono::Vehicle templates.  Several wheeled vehicle suspension templates, the `ChBalancer` sub-chassis template, as well as the tracked vehicle suspension and track shoe templates were updated to include this option.  

   A particular joint connection with this new option will be modeled as a bushing if bushing data is provided and as a kinematic joint otherwise. For example, the connections of the upper control arms to the chassis in the double wishbone suspension will be modeled as revolute joints (as before) if the virtual method `getUCABushingData` return `nullptr` and as bushings otherwise.  Bushing information is passed as a structure which provides stiffness and damping in the "constrained" linear and rotational directions and stiffness and damping in the DOF directions of the corresponding kinematic joint (see `ChVehicleBushingData`).  When instantiating a vehicle subsystem template through a JSON specification file, a joint with this capability will be modeled as a bushing if a JSON key "Bushing Data" is included.

2. All wheeled vehicle suspension templates that used to model their tierods using distance constraints were updated to optionally use rigid bodies for the tierods (these include the double and single wishbone, MacPherson, multi-link). A derived class specifies the type of tierod model by overriding the virtual function `UseTierodBodies`.  In a JSON file specification for such a suspension, the tierods will be modeled as bodies if the "Tierod" object includes the keys "Mass", "Inertia", and "Radius" and as distance constraints otherwise.

   When tierods are modeled as rigid bodies they will be connected using a spherical and universal joint or using bushings, depending on whether or not bushing data is provided.

3. JSON-based specification of a wheeled vehicle was enhanced to allow specification of rear chassis and associated chassis connectors, as well as subchassis subsystems.  An example set of JSON specification files for modelling an MTV truck with rear walking beam suspensions is available under the `data/vehicle/mtv/` directory.

4. The interface between a Chrono::Vehicle and a powertrain was modified to completely decouple the two systems and use a force-displacement co-simulation approach for all possible combinations of powertrain and driveline templates. In particular, this now allows using a shafts-based powertrain to drive one of the “simple” drivelines.

   Note also that a drivetrain's `GetDriveshaftSpeed` function now always reports a positive angular speed for a vehicle moving forward and a negative value for reverse (although internally these signs must be reversed due to the particular implementation of the shafts-body constraint in Chrono).

5. The contact manager for tracked vehicles was extended to also allow use with a track test rig. Furthermore, new public methods on `ChTrackedVehicle` and `ChTrackTestRig` allow controlling rendering of contact information (normals and/or contact forces) for all monitored subsystems.

6. Support was added for specifying and applying user-defined external forces on a vehicle's chassis.  Such forces are defined in a class derived from `ChChassis::ExternalForce` which should override the `Update` method (to calculate new values for the force and/or its application point at each synchronization of the chassis state).  An arbitrary number of such forces can be defined and added using `ChChassis::AddExternalForce`.

7. A demonstration program (`demo_VEH_RenderJSON`) was created to illustrate visualization of a Chrono::Vehicle model based on JSON specification files.  Using the Chrono::OpenGL run-time visualization module, this demo program allows re-creating the vehicle model after a potential change to one or more JSON specification files (use key `U` to trigger).

## [Added] New robot models

Two new models were added to the collection Chrono robot models:

- The **Curiosity** Mars Rover is a six-wheel rover model. The model can simulate the Curiosity-class Mars rover which includes a passive Rocker-Bogie suspension system. The operation and the usage of the Curiosity Rover is similar to the Viper Lunar Rover. The steering function of the Curiosity Rover needs to be explicitly controlled by calling
  ```cpp
  SetSteerSpeed(double speed, WheelID id)
  ```
  This independent steering control allows the rover model to conduct many types of steering maneuvers. The linear DC motor model in Curiosity is similar to the DC motor in Viper (see below).

  `demo_ROBOT_Curiosity_SCM` illustrates the rover crossing symmetric obstacles on SCM deformable terrain and `demo_ROBOT_Curioisty_Rigid` shows the rover being operated on rigid terrain while climbing a stair-shaped obstacle. Both demos show the initialization process of the Curiosity rover model and the simulated Rocker-Bogie suspension system when crossing obstacles.

- The **Turtlebot** is a common basic robot used as demonstration in various robot simulation packages (e.g., Gazebo/ROS). This robot consists of two drive wheels and one passive wheel. The steering function can be controlled by calling 
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


## [Added] New multicore collision detection system

The collision detection system previously embedded in Chrono::Multicore was updated and also made available to the usual Chrono systems (ChSystemNSC and ChSystemSMC) as an alternative to the Bullet-based collision detection system.  The new collision detection system (`ChCollisionSystemChrono`) uses a single-level adaptive grid for broadphase; for the narrowphase, it uses analytical intersection functions for certain pairs of known primitive shapes with fallback to an MPR (Minkovski Portal Refinement) algorithm.  In addition to the features previously available in Chrono::Multicore, the new stand-alone collision detection system includes additional analytical collision functions (e.g., for box-box interaction), as well as support for ray intersection.

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
- ray casting is thread safe (i.e., multiple ray intersection tests can be done concurrently, for example in a parallel OpenMP for loop).

**Limitations.**
The main limitation of the new multicore collision detection system is that removal of collision models from the collision system is currently not supported.  As such, bodies with collision enabled cannot be removed from the system once added.

**Work in progress.**
The following enhancements are currently under development:
- ray intersection with generic convex shapes
- support for collision of flexible bodies


## [Added] Miscellaneous additions to Chrono::Gpu

**Added - Specification of the computational domain**

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


## [Added] New loads for ChNodeFEAxyzrot

New classes have been added for creating loads (with automatic Jacobian generation that allow also stiff loads) for ChNodeFEAxyzrot nodes, in detail:
- on a node of ChNodeFEAxyzrot type (user defined etc.)
- between two ChNodeFEAxyzrot (user defined, spherical bushing, plastic bushing, generic bushing, etc.)
- between a ChNodeFEAxyzrot and a ChBody (user defined, spherical bushing, plastic bushing, generic bushing, etc.)
Previously, these types of loads were available only for the ChNodeFEAxyz node (used in tetahedrons and bricks, for example) but not for ChNodeFEAxyzrot (used in beams and Reissner shells, for example). 

## [Added] Analytical box box collision detection algorithm in Chrono::Multicore

A new algorithm for analytical collision detection for box-box interactions was added to the parallel collision system implemented in Chrono:Multicore.
For collisions involving two boxes, this new algorithm is now used instead of the default MPR algorithm (when using narrow phase type `NARROWPHASE_R` or `NARROWPHASE_HYBRID_MPR`).

The new algorithm relies on the 15-axes test of Gottschalk, Lin, and Manocha (Siggraph 1996) for finding the direction of minimum intersection between two oriented boxes and then the collision detection is special-cased for all possible combinations of interacting features from the two boxes (9 different cases).
The analytical algorithm can produce up to 8 collision pairs and works with or without a collision envelope (thus being appropriate for both SMC and NSC contact formulations).

## [Added] Checkpointing capabilities in Chrono::Gpu

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

`ChSystemGpu::ReadParticleFile` is used to load particle positions and velocities from a CSV file. It is useful if the particle information is meant to be supplied from a file rather than programmatically.

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


## [Fixed] Particle volume samplers and generators

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

## [Changed] SCM deformable terrain improvements

The reference frame for calculation of normal and tangential forces has been changed to be aligned with the local normal (as opposed to always being aligned with the SCM frame).  This fixes the generated terrain forces for SCM patches defined from height maps.  The Bekker forces are aligned with the local terrain normal, while the tangential shear forces (Janosi-Hanamoto) lie in the local tangent plane.  Note that the normal at each grid node is based on the undeformed terrain.

In addition, the SCM implementation was changed to extend the logical grid beyond the (horizontal) limits specified by the user during initialization; this continuation is done by extending to infinity the logical patch using the terrain height and normal from the closest grid node within the specified domain.

Finally, support was added for inclusion of tire-soil parameters (in addition to soil-soil parameters), if these are available. To provide this information, attach a structure of type `SCMContactableData` as user-data to the desired contactable object (e.g. a tire or track shoe body):
```cpp
    auto tire_data = chrono_types::make_shared<chrono::vehicle::SCMContactableData>(Aratio, Mcoh, Mfric, Jshear);
    tire_body->SetUserData(tire_data);
```
The necessary data includes the SCM tangential force parameters, Mohr cohesion (Pa), friction angle (degrees), and the Janosi shear parameter (m), as well as a ratio that represents the weight of the tire-soil parameters in calculating the tangential force (using linear interpolation). A ratio value of 0 indicates using only the soil-soil parameters, while a value of 1 indicates using only the tire-soil parameters.  Typically, this ratio is set as the area ratio of tread surface over tire surface.  

## [Changed] Miscellaneous fixes to Chrono::Vehicle API

- Changed enum class names for suspension, driveline, and steering types to properly differentiate between wheeled and tracked vehicles.
   The new enum classes, defined in `ChSubsysDefs.h` are SuspensionTypeWV, DrivelineTypeWV, and SteeringTypeWV for wheeled vehicles and DrivelineTypeTV for tracked vehicles.

- Eliminated the setting for differential ratio in the various driveline templates.
   To model a differential using the Chrono class `ChShaftsPlanetary`, this value must always be -1 (and represents the speed ratio of the inverted planetary) and is therefore hard-coded.
   This affects driveline models that use the Chrono 1-D shaft modeling elements and the schema of associated JSON specification files.

- Modified all shafts-based driveline templates to expect a positive value for the conical gear ratios.

- Added option (`ChPowertrain::SetTransmissionMode`) for setting the transmission mode of a powertrain to either `AUTOMATIC` or `MANUAL` (the latter modeling a manumatic-type transmission).  
   If in `MANUAL` mode, gear shifting can be controlled with `ChPowertrain::ShiftUp`  and `ChPowertrain::ShiftDown`.

- Modified the "Simple CVT" powertrain template.
   In the new template specification, a parameter for max engine speed was added and the parameter for critical engine speed was removed.

- Added utility function to programmatically generate a sprocket visualization mesh (`ChSprocket::CreateVisualizationMesh`).
   All Chrono::Vehicle sprocket profiles are defined as a succession of line segments and circle arcs. The default visualization is of type `VisualizationType::PRIMITIVES` and is a 3-D line for the profile.  The utility function `ChSprocket::CreateVisualizationMesh` creates a trimesh that can be used to visualize the sprocket when in `VisualizationType::MESH` mode.

- Changed sign of sprocket angular speed reported by GetSprocketSpeed so that a positive value corresponds to forward vehicle movement.
   This change was made simply for convenience and consistency.

- Completed the braked differential steering driveline for tracked vehicles (`ChTrackDrivelineBDS`) to properly implement steering.
    In this driveline model, steering is achieved through braking; this is implemented through a driveline-specific utility function that combines the steering and braking controls.

- Added function `RandomSurfaceTerrain::EnableCollisionMesh` to optionally generate a terrain collision mesh.
    This is necessary for tracked vehicles or wheeled vehicles with rigid tires (which rely on the underlying Chrono contact system).

## [Added] New tracked vehicle model

The Marder ("marten" in German) is a tracked infantry fighting vehicle used by the German Bundeswehr since 1969. It has a running gear with 12 road wheels, sprocket, idler and 3 support rollers. The first two and the last two road wheels on every side are damped by telescopic dampers. It is driven by a 444 kW Diesel engine, torque converter with lockup and 4 gear automatic gearbox. It carries up to nine soldiers (commander, gunner, driver and six infantrymen).

The Chrono::Vehicle model is based only on public data available online and information found in literature. Although the original vehicle employs double-pin tracks, the current Chrono model only implements a single-pin track.

## [Changed] Support for Z up camera in Chrono::Irrlicht

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

## [Changed] Reading and writing collision meshes in Chrono::Gpu

The mechanism for specifying collision meshes in a `ChSystemGpuMesh` was changed to allow adding meshes in a sequential manner, at any point and as many times as desired, prior to invoking `ChSystemGpuMesh::Initialize()`. Various different functions are provided for adding a mesh from memory:
```cpp
    unsigned int AddMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,
                         float mass);
```
from a Wavefront OBJ file:
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

Chrono::Gpu module now outputs VTK meshes correctly by writing to files the nodal coordinates and connectivity, instead of triangle soups. It also no longer appends `_mesh` to the output filenames. Users can still write all meshes to a single file by 
```cpp
    void WriteMeshes(const std::string& outfilename) const;
```
or write a particular mesh to a file by
```cpp
    void WriteMesh(const std::string& outfilename, unsigned int i) const;
```

## [Added] Support for the Emscripten compiler targeting WebAssembly

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


# Release 6.0.0 (2021-02-10)

## [Added] New Chrono::Csharp module

The new Chrono::Csharp module provides a C# interface to selected Chrono functionality.  This allows using Chrono from C# programs and facilitates the integration of Chrono with external engines such as Unity.

The module relies on SWIG to automatically generate the interface library and wrapper C# classes.  Upon build, the module creates the wrapper C# files under a `chrono_csharp/` directory in the build tree and a number of shared libraries (dll on Windows, so on Linux) in either the `bin/` or `lib/` directory, depending on platform. Currently, the Chrono::Csharp module provides an interface to the multibody dynamics capabilities in the core Chrono module, as well as to Chrono::Vehicle and the associated vehicle models.

## [Added] RoboSimian, Viper, and LittleHexy models

Models of the legged RoboSimian robot, the wheeled Viper rover, and the six-propeller LittleHexy copter are now included in the collection of Chrono models.  These models have no dependencies beyond the core Chrono module, except for an optional utility class for RoboSimian visualization with Irrlicht. Python wrappers are also provided, allowing use of these models with PyChrono. Related demo programs illustrate the robots moving over rigid or SCM deformable terrain (using a core Chrono system) and over granular terrain (using the Chrono::Multicore module).

## [Added] Contact force reporting through user-provided callback

The `OnReportContact` method of a user-supplied reporter callback (derived from `ChContactContainer::ReportContactCallback`) is now called with the proper force and torque for the current contact when using a Chrono::Multicore parallel system (of either NSC or SMC type).  The reported contact force and torque are provided at the contact point and expressed in the *contact frame* (defined by the provided rotation matrix).

For examples of using the contact reporting feature with a Chrono::Multicore system, see `demo_MCORE_callbackNSC` and `demo_MCORE_callbackSMC`.

## [Changed] Chrono::Gpu module rename

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

## [Changed] Chrono::Multicore module rename

For consistency and to better reflect the purpose of this module, Chrono::Parallel was renamed to **Chrono::Multicore**.

The related API changes are simply replacements of *parallel* with *multicore*, keeping the same capitalization:
- `chrono_multicore/` replaces `chrono_parallel/`
- class names use `Multicore` instead of `Parallel` (e.g.; `ChSystemMulticore`)
- macro names use `MULTICORE` instead of `PARALLEL` (e.g.; `CHRONO_MULTICORE`)
- the CMake project configuration script ChronoConfig.cmake expects the component name `Multicore` instead of `Parallel` 

In addition, names of related demos, unit tests, and benchmark tests include the string `MCORE` instead of `PAR` (e.g.; `demo_MCORE_mixerNSC`).

Users of the Chrono::Multicore module should rerun CMake since the variables related to this module have also changed name (e.g.; `ENABLE_MODULE_MULTICORE`).

## [Added] Geometric stiffness for Euler beams

The geometric stiffness term is now introduced also for the chrono::ChElementBeamEuler beam element (Euler-Bernoulli corotational beams). It is turned on by default, and it is computed via an analytical expression, with minimal cpu overhead. 
Note that geometric stiffness was already considered in IGA and ANCF beams, only the Euler beam was missing. Geometric stiffness is responsible of the fact that if you pull a thin beam like a string, its natural frequencies will increase, or vice-versa, if you push it, its lateral stiffness decreases up to buckling instability. 
Note that Euler beams ware able to simulate buckling or pulled-string stiffening even before, but only doing time integration in multiple time steps: instead, if one exported the M,K matrices for doing modal analysis of a pre-stretched Euler beam after a static analysis, the K matrix was missing the contribution of the geometric stiffness hence frequencies were incorrect only in modal analysis.


## [Added] New Chrono::Synchrono module

The new `Chrono::SynChrono` (or simply SynChrono) module has been introduced. SynChrono aims to provide an easier entry point for physics-based autonomous vehicle simulations, and to this end it uses MPI to parallelize simulations in the case where there is no physical interaction between agents. For example in a simulation of two vehicles driving separately, there is no need to simulate interaction between them, yet they must have some knowledge of each other for visualization and for any sensors that they may carry.

SynChrono is equipped to synchronize any "agent" (e.g. an arbitrary robot whose state is defined by some combination of mechanisms), but currently there are concrete wrapper classes for synchronizing `Chrono::Vehicle`'s, these are `SynWheeledVehicleAgent` and `SynTrackedVehicleAgent`. Another example of an agent, that can be used as a model for a user-defined agent, is `SynEnvironmentAgent` which represents a smart traffic intersection. Synchronization of `SCMDeformableTerrain` is also possible.

While SynChrono's primary purpose is to synchronize the state of agents, the MPI communication that synchronizes state data can also be used to send other messages. Examples of these could be messages from an intelligent traffic light to a vehicle (see `flatbuffer/message/Syn[SPAT/MAP]Message`) or from a vehicle to a vehicle with some sensor information (see `SynSensorMessage`). SynChrono supports both `Chrono::Irrlicht` and `Chrono::Sensor`-camera rendering of scenes, and `visualization/` packages some simple defaults along with a class to facilitate easy swapping between the two.

## [Changed] Rename Intel MKL Pardiso interface module

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


## [Added] Saving POV-Ray files from Irrlicht interactive view

New feature in the Irrlicht interactive 3D view. When pressing the F12 key, a directory `povray_project` is immediately created on disk, and .pov .ini .assets etc. files are created inside it, so that later you can use POVray to load the .ini and render the simulation with high quality ray tracing. Press F12 again to stop saving the POVray files. Note that you must later edit the `povray_project/render_frames.pov` to change/add the lights, global illumination, and other specific raytracing settings.
This feature is available only if you build also the `POSTPROCESS` module, so check *both* `ENABLE_MODULE_IRRLICHT` and  `ENABLE_MODULE_POSTPROCESSING` in CMake.

Also, the API of the `ChPovRay` class has been simplified. One just uses the new `SetBasePath()` function to set the directory that will contain all .ini, .pov, etc. files, and anim/, output/ subdirectories. The user does not need to create these folders anymore, these are automatically generated if necessary, when setting up ChPovRay with `ExportScript()`. Also, some features of ChPovRay have been fixed / improved.


## [Added] Support for modelling wheeled trailers

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

## [Changed] Enhancements to Chrono::FSI

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

## [Added] New Chrono::Sensor module

A new module (`Chrono::Sensor`) has been introduced to allow for sensor simulation within Chrono. `Chrono::Sensor` provides an interface for modeling and simulating sensors in the Chrono system to provide input for perception and control algorithms. For example, `Chrono::Sensor` may be used in combination with `Chrono::Vehicle` to simulate an autonomous vehicle equipped with multiple cameras and lidars. The module contains an API for modeling sensors with noise and distortion using a filter-graph paradigm for customization. Rendered sensors (camera and lidar) utilize ray tracing via OptiX to generate synthetic data.

Parameterized models for camera, lidar, GPS and IMU have been added with the ability to extend or implement custom sensors.

`Chrono::Sensor` is designed around a `ChSensorManager` which maintains all time synchronization and resources management between the sensing module and the core chrono system. Sensors such as a `ChCameraSensor` and `ChLidarSensor` can be added to the manager and mounted to a Chrono body which will determine the sensor's dynamics. The sensor are maintained by the `ChSensorManager` and all data is received via an added `ChFilterAccess` in the filter graph, determined by the sensor's parameters.

Locations:
- `Chrono::Sensor` source code is maintained under `src/chrono_sensor/`
- Demos are located in `src/demos/sensor/`
- Sensor specific data is located in `data/sensor/`

## [Changed] Setting OpenMP number of threads

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


## [Changed] Redesigned SCM deformable terrain

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
  where `filename` is the name of an image file, `sizeX` and `sizeY` specify the patch extents in the reference plane, `min_height` and `max_height` define the height range (a purely black image pixel corresponds to min_height, a purely white pixel corresponds to max_height) and `resolution` is the SCM grid spacing.

- The option for adaptive mesh refinement was obsoleted. Performance of the new implementation is limited by the ray-casting operations and as such no additional benefits are obtained from starting with a coarse grid.

- A "moving patch" is now defined by specifying an object-oriented-box attached to a moving body. For example,
  ```cpp
  terrain.AddMovingPatch(my_body, ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));
  ``` 
  associates a moving patch with the box of size (5,3,1) attached at the center of the body reference frame of `my_body`.  Ray casting is performed only for the SCM grid nodes that are in the current projection of this OBB onto the SCM reference plane.

  If the user does not define any moving patches, SCM uses the projection of the current bounding box of all collision shapes in the system.

- Bulldozing effects are enabled using `SCMDeformableTerrain::EnableBulldozing`.
- SCM soil parameters and bulldozing settings are specified as before.

## [Added] Tracked vehicle support in PyChrono

Tracked vehicle templates and models are now exposed in Chrono::Python and available for use through PyChrono.


## [Changed] Constitutive models for EULER beams

Section properties of the ChElementBeamEuler are now defined via a **new class** `ChBeamSectionEuler` and subclasses. Old classes for Euler sections have been renamed and rewritten, the old classes have been **deprecated** and will be removed in future:
 - `ChBeamSectionBasic`, use  `ChBeamSectionEulerSimple` instead
 - `ChBeamSectionAdvanced`, use  `ChBeamSectionEulerAdvanced` instead
 
Note that in the previous release, the Sy and Sz values for **shear center** offset in `ChBeamSectionAdvanced` were assumed with **opposite sign** respect to the description and illustrative figure: now this bug is fixed, and shear center offset works the same as in Cosserat beams.

Also, a new class  `ChBeamSectionEulerGeneric` has been added, that does not make the assumption of uniform density and uniform elasticity, so it accepts directly the beam rigidity values bypassing the E and Izz Iyy values. 
 
To speed up coding in case of simple beams, two new classes `ChBeamSectionEulerEasyRectangular` and `ChBeamSectionEulerEasyCircular` have been added.


## [Changed] Constitutive models for IGA beams

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


## [Added] Obtaining body applied forces

The new functions `ChBody::GetAppliedForce` and `ChBody::GetAppliedTorque` return the body resultant applied force and torque, respectively.

1. These functions include contributions from all external applied loads acting on a body (e.g., gravitational forces, forces defined through ChForce elements, forces specified through ChLoad, springs, dampers, etc).
   However, they **do not** include any constraint forces.  In particular, this means that contact forces are not included when using the NSC formulation, but are included when using the SMC formulation.
   For the former case, use `ChBody::GetContactForce` and `ChBody::GetContactTorque` to obtain the resultant contact force and torque, respectively.
   
2. Note that reporting this information requires a traversal of the entire system and caching the generalized forces, a quantity that is otherwise not computed in the form required for this reporting.  To prevent any additional overhead when this information is not requested by the user, this is done using lazy evaluation.   In other words, no overhead is incurred at a simulation step if no applied forces are requested. On the other hand, there is a small (but non-zero) cost when a call to `ChBody::GetAppliedForce` or `ChBody::GetAppliedTorque` is made; however, this cost is constant at any given time, regardless of how many queries are made.  Note also that this additional cost is not incurred for Chrono::Parallel.


## [Added] Chrono::Vehicle simulation world frame

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


## [Changed] CASCADE module

1.  Support for OpenCASCADE 7.4.0. The API of OpenCASCADE introduced some changes in the 7.4.0 version so we also updated the CASCADE module of Chrono. Please download and upgrade your OpenCASCADE version as it is not backward compatible. (The module is optionally built via CMake configuration flag ENABLE_MODULE_CASCADE, also remember to update the CASCADE_INCLUDE_DIR and CASCADE_LIBDIR paths and to update your PATH if you added the path to Cascade dlls)

2.  The method `ChCascadeDoc::CreateBodyFromShape()` is obsolete. Just use the `ChBodyEasyCascade` class to obtain the same result, for example:
    ```cpp
    auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(myshape, ...);
    ```

3.  The mesh tesselation algorithm could give coarser or finer meshes with respect to the previous release.


## [Changed] Collision shapes and contact materials

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


# Release 5.0.1 (2020-02-29)

## [Fixed]

- Correct the ChElementBeamANCF applied moment calculation so that it uses normalized shape function derivatives
- Comment out code related to applying moments on ANCF elements (requires further testing))

# Release 5.0.0 (2020-02-24)

## [Changed] Refactoring of dense linear algebra

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

## [Changed] Eigen sparse matrices and updates to direct sparse linear solvers

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

# Release 4.0.0 (2019-02-22)

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
