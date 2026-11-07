Tire models {#wheeled_tire}
===========================


\tableofcontents

Chrono::Vehicle currently supports four different classes of tire models: rigid, handling, finite element, and multibody.

The choice of tire model is constrained by the choice of [terrain model](@ref vehicle_terrain). [Handling tires](@ref vehicle_tire_empirical) carry no collision geometry and compute tire forces from the terrain height, normal, and coefficient of friction only; they can be used with flat, rigid, CRG, and other height-field terrains, but not with deformable terrain. [Rigid](@ref wheeled_tire_rigid), [FEA](@ref wheeled_tire_fea), and [multibody](@ref wheeled_tire_mb) tires interact with the terrain through the Chrono contact system (or through FSI for CRM terrain) and are required for deformable terrain.

| Terrain                              | Handling tires | Rigid tires | FEA and MB tires |
|--------------------------------------|:--------------:|:-----------:|:----------------:|
| Flat, CRG                            | yes            | no          | no        |
| Random surface, obstacle (ObsMod)    | yes            | with collision mesh | with collision mesh |
| Rigid                                | yes            | yes         | yes       |
| SCM (deformable)                     | no             | yes         | yes       |
| Granular / DEM (deformable)          | no             | yes         | yes       |
| CRM / SPH (deformable)               | no             | yes         | yes       |
| FEA (deformable)                     | no             | yes         | yes       |

## Rigid tire model  {#wheeled_tire_rigid}

The rigid tires are the simplest of the three tire classes offered.  The assumption for these models is that the tire is completely rigid and it interacts with the ground and any other rigid objects through the same underlying friction and contact algorithms as the other rigid bodies in Chrono.  The contact geometry for these tires can be as simple as a cylinder or as complex as a 3D triangular mesh.  These models are not only useful for debugging the overall vehicle model, but they can also be used in cases where runtime is important, the terrain is much softer than the tire, and a highly detailed model of the tire is unnecessary.  Together with the FEA-based and multibody tires, rigid tires are the only option for off-road vehicle simulation on deformable terrain (SCM, granular, CRM, or FEA-based; see the description of available [terrain models](@ref vehicle_terrain)), since [handling tires](@ref vehicle_tire_empirical) cannot be used with such terrain. 

See [ChRigidTire](@ref chrono::vehicle::ChRigidTire) and [RigidTire](@ref chrono::vehicle::RigidTire).

The following is an example of a rigid tire with mesh geometry provided through a Wavefront OBJ file: 
\include "data/vehicle/hmmwv/tire/HMMWV_RigidMeshTire.json"

## Handling (semi-empirical) tire models {#vehicle_tire_empirical}

The second class of tires models offered are the semi-empirical ones commonly used for vehicle handling.  Chrono::Vehicle currently has implementations for Pacejka (89 and 2002), TMeasy, TMsimple, and Fiala tire models (all derived from [ChForceElementTire](@ref chrono::vehicle::ChForceElementTire)). Handling tire models are designed for flat road surfaces and they normally use single point contact or four point contact (TMeasy). For ride tests on undulated roads or for obstacle crossing a special contact algorithm called "envelope" has been implemented. It is based on a paper of Sui & Hershey and can be used with all handling tire models that are included in Chrono. Validation tests with a technology demonstrator show good results compared to measured real vehicle test data with low numerical effort. If an even better accuracy is needed the user should think about considering FEA based tire models for non-handling tests.

Handling tire models do not have any collision geometry: they do not participate in the Chrono collision detection and interact with the terrain exclusively through its height, normal, and coefficient of friction (see [ChTerrain](@ref chrono::vehicle::ChTerrain)). As a consequence, handling tires **cannot be used with deformable terrain** (SCM, granular, CRM, or FEA-based): such a tire would neither deform the soil nor experience sinkage or soil resistance. For deformable terrain, use [rigid](@ref wheeled_tire_rigid), [FEA](@ref wheeled_tire_fea), or [multibody](@ref wheeled_tire_mb) tires.

Some users may want to build their own handling tire parameter sets from tire test data. Be sure to get familiar with the different slip definitions and the different coordinate systems in each tire model!

### Pacejka 89 (Pac89) tire model {#wheeled_tire_pac89}

See [ChPac89Tire](@ref chrono::vehicle::ChPac89Tire) and [Pac89Tire](@ref chrono::vehicle::Pac89Tire).

### Pacejka 2002 (Pac02) tire model  {#wheeled_tire_pac02}

This model is an extension of Pacejka's earlier Magic Formula tire model with additional equations and coefficients.  Since a large number of vehicle dynamics maneuvers do not occur under steady-state slip conditions, the contact patch slip state equations are included to provide more accurate results under transient conditions. Due to the lack of reference data sets the inflation pressure dependence terms and the large camber terms are actually not implemented.

This Chrono tire model can also be specified using Adams/Car-compatible TIR files.

See [ChPac02Tire](@ref chrono::vehicle::ChPac02Tire) and [Pac02Tire](@ref chrono::vehicle::Pac02Tire).

### TMeasy tire model (basic version) {#wheeled_tire_tmeasy}

TMeasy (Tire Model Made Easy) has been developed by Prof. Dr. Georg Rill and is available as a commercial code (see http://www.tmeasy.de/ for more information). The aim of this model is to allow easy parameterization. Its complexity is roughly comparable to the Magic Formula based models. Furthermore, based on known parameter sets for trucks and passenger cars it is possible to estimate a complete parameter set from few input data. TMeasy considers nonlinear effects and includes contact patch slip state equations. The implementation used in Chrono lacks some functionality compared to the latest commercial TMeasy versions. For example, there is no belt dynamics and no dynamic parking torque calculation possible. The Chrono TMeasy tire implementation contains algorithms that have been published in the book [Road Vehicle Dynamics - Fundamentals and Modeling with MATLAB](https://www.routledge.com/Road-Vehicle-Dynamics-Fundamentals-and-Modeling-with-MATLAB/Rill-Castro/p/book/9780367199739), Georg Rill and Abel Arrieta Castro, CRC Press, 2020.

See [ChTMeasyTire](@ref chrono::vehicle::ChTMeasyTire) and [TMeasyTire](@ref chrono::vehicle::TMeasyTire).

A sample JSON file with a TMeasy tire specification is provided below:
\include "data/vehicle/hmmwv/tire/HMMWV_TMeasyTire.json"

### TMsimple tire model {#wheeled_tire_tmsimple}

See [ChTMsimpleTire](@ref chrono::vehicle::ChTMsimpleTire) and [TMsimpleTire](@ref chrono::vehicle::TMsimpleTire).

A sample JSON file with a TMsimple tire specification is provided below:
\include "data/vehicle/hmmwv/tire/HMMWV_TMsimpleTire.json"


### Fiala tire model  {#wheeled_tire_fiala}

The Fiala tire model implemented in Chrono::Vehicle is largely based on the transient Fiala tire model presented in the MSC ADAMS/tire help documentation, which uses tire slip state equations to improve the model's behavior at slow to zero forward velocities.  The Fiala tire model is based on a brush model assumption and only requires a small number of coefficients. This tire model assumes that the tire is at zero camber with respect to the road and does not have any provisions for generating overturning moments.  It does however couple the lateral and longitudinal slip states of the tire in its force and moment calculations, providing a more realistic description of combined slip. The Fiala tire model should not be used for serious vehicle handling simulations since it does not consider important effects that influence the results.

See [ChFialaTire](@ref chrono::vehicle::ChFialaTire) and [FialaTire](@ref chrono::vehicle::FialaTire).

A sample JSON file with a TMeasy tire specification is provided below:
\include "data/vehicle/hmmwv/tire/HMMWV_FialaTire.json"

The vertical load curve embedded in the above JSON file is show below:

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/FialaTire_vertical_load.png" width="500" />

## Validation of handling tire models {#handling_tire_validation}

We show below a validation study for 3 of the Chrono::Vehicle handling tire models done against experimental data provided for the FED-alpha vehicle by the Keweenaw Research Center (KRC) in Michigan. 

In this test, called `Steady State Cornering` (SSC), the vehicle drives along a circle beginning with the so-called "Ackermann-Speed". This value is around 3 m/s, and as such the slip values are pretty small and the vehicle follows the circle kinematically. The real test continues by increasing the vehicle speed in steps and holding it constant. These successive speed increase steps are repeated until the maximal possible lateral acceleration is reached, or the engine power is exhausted. The KRC test was conducted in left turn and right turns, to highlight any asymmetry of the vehicle (if present).

The goal of the SSC test is to infer if a vehicle is safe for an average driver. The vehicle should understeer, meaning the driver must provide more and more turning angle of the steering wheel with increasing lateral acceleration to keep the vehicle on the circle and maintain vehicle stability. Vehicles which are "oversteering" tend to turn toward the circle origin and become unstable, because there is not enough lateral slip reserve at the rear wheels. This oversteering behavior should be avoided for standard vehicles used in public traffic by normal drivers, although professional rally drivers prefer the oversteerers. 

For the FED-alpha, the SSC test has been defined as a quasi-steady-state run, where the speed changes continuously but very slowly, so that there are no noticeable dynamics effects. The simulation results are therefore shown as continuous lines. The signs of the data are chosen in a way that shows the right turn on the right side and the left turn on the left side.  Experimental KRC data is represented with dots.  The plots below show SSC simulation results for the Chrono Pac02, TMsimple, and TMeasy tire models superimposed over the KRC experimental data.

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/Pac02_SSC.png" width="650" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/TMsimple_SSC.png" width="650" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/TMeasy_SSC.png" width="650" />


## FEA-based tire models  {#wheeled_tire_fea}

The third class of tire models offered are full finite element representations of the tire.  While these models have the potential to be the most accurate due to their detailed physical model of the tire, they are also the most computationally expensive of the tire models currently available in Chrono::Vehicle.  Unlike the rigid or semi-empirical tire models, the finite element based tire models are able to account for the flexibility in both the tire and in the ground at the same time, which is an important characteristic for many types of off-road mobility and vehicle dynamics studies.  These finite element tire models leverage the nonlinear finite element capabilities in Chrono. 

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/FEA_tire_sections.png" width="600" />

All deformable tires derive from [ChDeformableTire](@ref chrono::vehicle::ChDeformableTire). The tires built on an FEA mesh (ANCF shell, Reissner shell, and solid-element tires) derive from [ChFEATire](@ref chrono::vehicle::ChFEATire), which provides access to the underlying mesh (`GetMesh`). Every deformable tire provides a triangle surface mesh over its nodes (`GetSurfaceMesh`): this is the tire contact surface if that is a triangle mesh, or else a surface over the same nodes that does not participate in collision. This surface is used to couple the tire with CRM terrain (see `CRMTerrain::AddFeaSurface`). The [multibody tire](@ref wheeled_tire_mb) also derives from ChDeformableTire, but it does not use an FEA mesh. CRM terrain couples only triangles with translational (XYZ) nodes, so Reissner shell tires cannot currently be used with CRM terrain.

### ANCF shell deformable tire {#vehicle_tire_ancf}

See [ChANCFTire](@ref chrono::vehicle::ChANCFTire) and [ANCFTire](@ref chrono::vehicle::ANCFTire).

The following JSON file contains the specification of an ANCFTire:
\include "data/vehicle/hmmwv/tire/HMMWV_ANCF4Tire.json"

### Reissner shell deformable tire {#vehicle_tire_reissner}

See [ChReissnerTire](@ref chrono::vehicle::ChReissnerTire) and [ReissnerTire](@ref chrono::vehicle::ReissnerTire).

## Multibody tire model {#wheeled_tire_mb}

The fourth class of tire models is the multibody (MB) tire, a lumped-mass model of the tire carcass. Like the FEA tires, it captures the tire deformation and the contact patch, but at a much lower computational cost.

See [ChMBTire](@ref chrono::vehicle::ChMBTire) and [MBTire](@ref chrono::vehicle::MBTire).

Point-mass nodes are placed on a set of rings (specified by their radii and transversal offsets) with a given number of divisions around the circumference. Nodes are connected by translational springs along the rings (circumferential) and across the rings (transversal), by rotational springs that resist bending along and across the rings, and by radial springs that connect the first and last rings to the rim. Internal pressure is applied at each node and contact with the terrain uses a node cloud or a triangle mesh contact surface. The MB tire does not use an FEA mesh and derives directly from [ChDeformableTire](@ref chrono::vehicle::ChDeformableTire).

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/MBTire_schematic.png" width="650" />

Spring coefficients can be specified in two ways (see `SetSpringCoefficientType`):
- `PER_SPRING`: the coefficients are used as given for every spring. The tire response then depends on the number of divisions and on the ring spacing: refining the mesh adds springs in series and in parallel and changes the tire stiffness.
- `DISTRIBUTED` (default): the coefficients are properties of the tire surface (stiffness per unit width and unit strain for the translational springs, bending stiffness per unit width for the rotational springs) and the coefficient of each spring is calculated from the mesh geometry. The tire response then converges as the mesh is refined, so the number of divisions becomes a pure discretization choice. Bending along the rings and across the rings (sidewall bending) can be given separate coefficients.

The same coefficients apply to all rings: the model does not distinguish between the tread (crown) and the sidewalls.

By default, the tire mass is distributed to the nodes in proportion to the surface area associated with each node, and the pressure load is a follower load acting along the current surface normal with a magnitude equal to the pressure times the current nodal area. Both can be disabled to recover the original model (uniform nodal masses and a constant pressure force directed from the wheel center to each node).

### Integrator and solver settings {#wheeled_tire_mb_solver}

The MB tire requires an SMC system. Its springs are stiff and damped (in particular the circumferential springs, which carry the stiffness of the tire belt), so the tire should be declared stiff (`IsStiff`, or `"Stiff": true` under `"Numerical Settings"` in a JSON specification). A stiff tire contributes its stiffness and damping Jacobians to the system matrix, and its forces are integrated implicitly. Otherwise, the spring forces are treated explicitly and the step size is limited by the stiffest and most heavily damped springs; with realistic coefficients, a non-stiff tire is unstable at practical step sizes. The tire Jacobians (in particular the follower pressure Jacobian) are not symmetric, so solvers that require a symmetric matrix (`MINRES`) cannot be used.

The recommended settings are the `EULER_IMPLICIT_LINEARIZED` integrator and the `PARDISO_MKL` direct solver (requires the Chrono::PardisoMKL module) with a locked sparsity pattern, and a step size of 1e-4 s. All MB tire demos use these settings, as do the YAML solver specifications `data/yaml/vehicle/tire_static_rig/rig_solver_mb.yaml` and `data/yaml/vehicle/wheel_rig/rig_solver_smc.yaml`. In a static compression test of a truck tire (static tire test rig, up to a load of 44 kN):
- With a step size of 1e-4 s, the load-deflection curve matches a reference solution (step size 1e-5 s) to within 0.1 mm. At 2e-4 s the error is 0.13 mm; at 5e-4 s and larger, the solution is no longer accurate.
- `EULER_IMPLICIT` gives the same results at the same cost for step sizes up to 1e-4 s, but needs more Newton iterations at larger steps. `EULER_IMPLICIT_PROJECTED` is about twice as expensive and `HHT` about three times as expensive, with no gain in accuracy.
- The iterative `BICGSTAB` solver is 4 to 5 times slower than `PARDISO_MKL`. It can be used if Chrono::PardisoMKL is not available, as can other sparse direct solvers that do not require symmetry (`SPARSE_LU`, `SPARSE_QR`, `MUMPS`).

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/MBTire_step_size.png" width="650" />

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/MBTire_solver_cost.png" width="650" />

### Contact {#wheeled_tire_mb_contact}

With an SMC (penalty) contact method, the tire deflection includes the contact penetration, which should be small compared to the tire deflection for results not to depend on the contact stiffness. The contact material Young's modulus is therefore a penalty parameter (together with the effective curvature radius used by the SMC contact force model), not the modulus of the tread rubber. Note that the combined contact stiffness is limited by the softer of the two contacting materials (e.g., the terrain). With a node cloud contact surface, the number of nodes in the contact patch, and therefore the total contact stiffness, grows with the number of divisions; specify a reference number of divisions (`SetContactReferenceDivisions`) to scale the contact material stiffness and damping accordingly. The tire contact statistics (`GetContactStats`) and the contact output channels of the static tire test rig report the number of contacts, the contact penetration, and the contact force. Collisions between elements of the tire contact surface (tire self-contact) are disabled, for both node cloud and triangle mesh contact surfaces.

The MB tire can be used on rigid, SCM, and CRM terrain. SCM terrain requires a triangle mesh contact surface. On CRM terrain, BCE markers are attached to the triangle surface mesh over the tire nodes (`GetSurfaceMesh`), independent of the contact surface type, and the soil forces are applied to the tire nodes (see `demo_VEH_WheelTestRig_CRM` and `data/yaml/vehicle/wheel_rig/wheel_rig_mbtire_crm.yaml`).

The following JSON file contains the specification of an MBTire:
\include "data/vehicle/hmmwv/tire/HMMWV_MBTire.json"

### Calibration and validation {#wheeled_tire_mb_calibration}

The parameters of an MB tire (spring coefficients, ring geometry, and mass) cannot be measured directly and must be calibrated against tire test data. The [static tire test rig](@ref wheeled_rig) reproduces the standard laboratory stiffness tests (radial, longitudinal, lateral, and torsional) and is the intended driver for calibration and validation, either through the C++ API (`demo_VEH_TireStaticTestRig`) or through a YAML specification (`demo_YAML_tire_static_rig`).

**Nokian 385/55R22.5 truck tire.** MB tire specifications are provided for a Nokian Hakka Truck Trailer 385/55R22.5 160K tire on a 22.5x11.75 rim, at an inflation pressure of 900 kPa:
- `data/vehicle/hmmwv/tire/HMMWV_MBTire_Nokian_Buffed.json`: the tested tire, which was buffed (tread removed). Its free radius (0.483 m) is the loaded radius plus the deflection reported in the radial test.
- `data/vehicle/hmmwv/tire/HMMWV_MBTire_Nokian.json`: the nominal tire, with the same coefficients and the full tread depth (free radius 0.498 m, mass 70.5 kg).

The corresponding static test rig specifications are `data/yaml/vehicle/tire_static_rig/tire_static_rig_hmmwv_mbtire_nokian_buffed.yaml` (the default selection in `demo_YAML_tire_static_rig`) and `tire_static_rig_hmmwv_mbtire_nokian.yaml`.

The test data include radial load-deflection curves at 600, 700, 800, and 900 kPa (up to the 4500 kg load index), the footprint, and the longitudinal, lateral, and torsional stiffness. Only the radial compression tests were used for calibration; the remaining tests are kept for validation.

The parameters of the buffed tire were set as follows:
- Geometry: the rim radius corresponds to the 22.5 in rim, the outer rings span the section width, and the shoulder rings sit 1 mm below the crown ring, which gives the nearly flat tread of a wide truck tire. With a more rounded tread profile and only three rings across the tread, only the crown ring is in contact at low load, and the load-deflection curve shows two distinct slopes.
- Circumferential springs: 5e7, representative of the steel belt. Much softer circumferential springs let the tire grow under the inflation pressure (with 1e7, the inflated radius exceeds the measured one by 18 mm).
- Transversal springs: 1.5e6, the main parameter controlling the slope of the load-deflection curve at high load.
- Circumferential bending: 3e3, which controls the stiffness at low load (contact patch formation). Radial springs (5e6) and transversal (sidewall) bending (10) have little influence on the radial response.
- Damping: 300 for all translational springs, no bending damping.
- Contact: Young's modulus 2e8 for the tire and 2e9 for the test rig plate.

With these parameters, the inflated radius at 900 kPa is 0.487 m (0.483 m measured). The table below compares the radial deflection (mm) of the buffed tire with the test data.

| Load (kN)                  | 8.9  | 13.3 | 17.8 | 22.1 | 26.5 | 31.1 | 35.5 | 39.9 |
|----------------------------|:----:|:----:|:----:|:----:|:----:|:----:|:----:|:----:|
| Test, 900 kPa              | 8.2  | 12.1 | 16.0 | 19.6 | 23.3 | 27.0 | 30.5 | 34.0 |
| MB tire, 900 kPa           | 9.3  | 12.7 | 17.3 | 21.2 | 24.0 | 26.7 | 29.5 | 32.3 |
| Test, 600 kPa              | 10.8 | 16.3 | 21.6 | 26.5 | 31.3 | 36.2 | 41.0 | 45.7 |
| MB tire, 600 kPa           | 11.0 | 16.9 | 21.7 | 25.2 | 28.8 | 32.6 | 36.5 | 40.8 |

<img src="http://www.projectchrono.org/assets/manual/vehicle/tire/MBTire_Nokian_radial_calibration.png" width="650" />

At 900 kPa, the RMS error is 1.1 mm over the measured load range. At 600 kPa (same coefficients, lower pressure), the RMS error is 2.9 mm, and the model is too stiff at high load. Between 900 and 600 kPa, the deflection at the highest load grows by about 26% in the model and by 34% in the test: the model attributes too large a share of the vertical stiffness to the springs rather than to the inflation pressure. With a single set of coefficients for all rings, recalibration alone cannot correct this.
