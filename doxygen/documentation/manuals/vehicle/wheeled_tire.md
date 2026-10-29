Tire models {#wheeled_tire}
===========================


\tableofcontents

Chrono::Vehicle currently supports three different classes of tire models: rigid, handling, and finite element. 

The choice of tire model is constrained by the choice of [terrain model](@ref vehicle_terrain). [Handling tires](@ref vehicle_tire_empirical) carry no collision geometry and compute tire forces from the terrain height, normal, and coefficient of friction only; they can be used with flat, rigid, CRG, and other height-field terrains, but not with deformable terrain. [Rigid](@ref wheeled_tire_rigid) and [FEA](@ref wheeled_tire_fea) tires interact with the terrain through the Chrono contact system (or through FSI for CRM terrain) and are required for deformable terrain.

| Terrain                              | Handling tires | Rigid tires | FEA tires |
|--------------------------------------|:--------------:|:-----------:|:---------:|
| Flat, CRG                            | yes            | no          | no        |
| Random surface, obstacle (ObsMod)    | yes            | with collision mesh | with collision mesh |
| Rigid                                | yes            | yes         | yes       |
| SCM (deformable)                     | no             | yes         | yes       |
| Granular / DEM (deformable)          | no             | yes         | yes       |
| CRM / SPH (deformable)               | no             | yes         | yes       |
| FEA (deformable)                     | no             | yes         | yes       |

## Rigid tire model  {#wheeled_tire_rigid}

The rigid tires are the simplest of the three tire classes offered.  The assumption for these models is that the tire is completely rigid and it interacts with the ground and any other rigid objects through the same underlying friction and contact algorithms as the other rigid bodies in Chrono.  The contact geometry for these tires can be as simple as a cylinder or as complex as a 3D triangular mesh.  These models are not only useful for debugging the overall vehicle model, but they can also be used in cases where runtime is important, the terrain is much softer than the tire, and a highly detailed model of the tire is unnecessary.  Together with the FEA-based tires, rigid tires are the only option for off-road vehicle simulation on deformable terrain (SCM, granular, CRM, or FEA-based; see the description of available [terrain models](@ref vehicle_terrain)), since [handling tires](@ref vehicle_tire_empirical) cannot be used with such terrain. 

See [ChRigidTire](@ref chrono::vehicle::ChRigidTire) and [RigidTire](@ref chrono::vehicle::RigidTire).

The following is an example of a rigid tire with mesh geometry provided through a Wavefront OBJ file: 
\include "data/vehicle/hmmwv/tire/HMMWV_RigidMeshTire.json"

## Handling (semi-empirical) tire models {#vehicle_tire_empirical}

The second class of tires models offered are the semi-empirical ones commonly used for vehicle handling.  Chrono::Vehicle currently has implementations for Pacejka (89 and 2002), TMeasy, TMsimple, and Fiala tire models (all derived from [ChForceElementTire](@ref chrono::vehicle::ChForceElementTire)). Handling tire models are designed for flat road surfaces and they normally use single point contact or four point contact (TMeasy). For ride tests on undulated roads or for obstacle crossing a special contact algorithm called "envelope" has been implemented. It is based on a paper of Sui & Hershey and can be used with all handling tire models that are included in Chrono. Validation tests with a technology demonstrator show good results compared to measured real vehicle test data with low numerical effort. If an even better accuracy is needed the user should think about considering FEA based tire models for non-handling tests.

Handling tire models do not have any collision geometry: they do not participate in the Chrono collision detection and interact with the terrain exclusively through its height, normal, and coefficient of friction (see [ChTerrain](@ref chrono::vehicle::ChTerrain)). As a consequence, handling tires **cannot be used with deformable terrain** (SCM, granular, CRM, or FEA-based): such a tire would neither deform the soil nor experience sinkage or soil resistance. For deformable terrain, use [rigid](@ref wheeled_tire_rigid) or [FEA](@ref wheeled_tire_fea) tires.

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

Finally, the third class of tire models offered are full finite element representations of the tire.  While these models have the potential to be the most accurate due to their detailed physical model of the tire, they are also the most computationally expensive of the tire models currently available in Chrono::Vehicle.  Unlike the rigid or semi-empirical tire models, the finite element based tire models are able to account for the flexibility in both the tire and in the ground at the same time, which is an important characteristic for many types of off-road mobility and vehicle dynamics studies.  These finite element tire models leverage the nonlinear finite element capabilities in Chrono. 

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/FEA_tire_sections.png" width="600" />

### ANCF shell deformable tire {#vehicle_tire_ancf}

See [ChANCFTire](@ref chrono::vehicle::ChANCFTire) and [ANCFTire](@ref chrono::vehicle::ANCFTire).

The following JSON file contains the specification of an ANCFTire:
\include "data/vehicle/hmmwv/tire/HMMWV_ANCF4Tire.json"

### Reissner shell deformable tire {#vehicle_tire_reissner}

See [ChReissnerTire](@ref chrono::vehicle::ChReissnerTire) and [ReissnerTire](@ref chrono::vehicle::ReissnerTire).

### Multibody (MB) deformable tire {#vehicle_tire_mb}

See [ChMBTire](@ref chrono::vehicle::ChMBTire) and [MBTire](@ref chrono::vehicle::MBTire).

The MB tire is a lumped-mass model of the tire carcass. Point-mass nodes are placed on a set of rings (specified by their radii and transversal offsets) with a given number of divisions around the circumference. Nodes are connected by translational springs along the rings (circumferential) and across the rings (transversal), by rotational springs that resist bending along and across the rings, and by radial springs that connect the first and last rings to the rim. Internal pressure is applied at each node and contact with the terrain uses a node cloud or a triangle mesh contact surface. The MB tire is much cheaper than the FEA tires, while still capturing the tire deformation and contact patch.

Spring coefficients can be specified in two ways (see `SetSpringCoefficientType`):
- `PER_SPRING`: the coefficients are used as given for every spring. The tire response then depends on the number of divisions and on the ring spacing: refining the mesh adds springs in series and in parallel and changes the tire stiffness.
- `DISTRIBUTED` (default): the coefficients are properties of the tire surface (stiffness per unit width and unit strain for the translational springs, bending stiffness per unit width for the rotational springs) and the coefficient of each spring is calculated from the mesh geometry. The tire response then converges as the mesh is refined, so the number of divisions becomes a pure discretization choice. Bending along the rings and across the rings (sidewall bending) can be given separate coefficients.

By default, the tire mass is distributed to the nodes in proportion to the surface area associated with each node, and the pressure load is a follower load acting along the current surface normal with a magnitude equal to the pressure times the current nodal area. Both can be disabled to recover the original model (uniform nodal masses and a constant pressure force directed from the wheel center to each node).

The MB tire is typically integrated with an SMC system, the `EULER_IMPLICIT_PROJECTED` integrator, and an iterative linear solver that does not require symmetry (`BICGSTAB`). When the tire is set as stiff (`IsStiff`), it contributes its stiffness and damping Jacobians to the system matrix; these Jacobians (in particular the follower pressure Jacobian) are not symmetric. In that case, use a direct sparse solver: `PARDISO_MKL` (fastest, requires the Chrono::PardisoMKL module) or `SPARSE_LU`, with a locked sparsity pattern. Iterative solvers converge slowly on the stiff system and `SPARSE_QR` is considerably slower than `SPARSE_LU`. A stiff tire allows a larger integration step (about 4 times larger than a non-stiff tire for comparable accuracy), but each step is more expensive, so a non-stiff tire is usually faster overall.

With an SMC (penalty) contact method, the tire deflection includes the contact penetration, which should be small compared to the tire deflection for results not to depend on the contact stiffness. Note that the combined contact stiffness is limited by the softer of the two contacting materials (e.g., the terrain). With a node cloud contact surface, the number of nodes in the contact patch, and therefore the total contact stiffness, grows with the number of divisions; specify a reference number of divisions (`SetContactReferenceDivisions`) to scale the contact material stiffness and damping accordingly. The tire contact statistics (`GetContactStats`) and the contact output channels of the static tire test rig report the number of contacts, the contact penetration, and the contact force.

The following JSON file contains the specification of an MBTire:
\include "data/vehicle/hmmwv/tire/HMMWV_MBTire.json"
