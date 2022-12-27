Tire models {#wheeled_tire}
===========================


\tableofcontents

Chrono::Vehicle currently supports three different classes of tire models: rigid, semi-empirical, and finite element. 

## Rigid tire model  {#wheeled_tire_rigid}

The rigid tires are the simplest of the three tire classes offered.  The assumption for these models is that the tire is completely rigid and it interacts with the ground and any other rigid objects through the same underlying friction and contact algorithms as the other rigid bodies in Chrono.  The contact geometry for these tires can be as simple as a cylinder or as complex as a 3D triangular mesh.  These models are not only useful for debugging the overall vehicle model, but they can also be used in cases where runtime is important, the terrain is much softer than the tire, and a highly detailed model of the tire is unnecessary.  In fact, the semi-empirical tire models are not suitable for off-road vehicle simulation, i.e. when using a deformable terrain (SCM, granular, or FEA-based; see the description of available [terrain models](@ref vehicle_terrain)).  For such scenarios, rigid or FEA-based tire models are the only two available options currently implemented in Chrono::Vehicle. 

See [ChRigidTire](@ref chrono::vehicle::ChRigidTire) and [RigidTire](@ref chrono::vehicle::RigidTire).

The following is an example of a rigid tire with mesh geometry provided through a Wavefront OBJ file: 
\include "../../data/vehicle/hmmwv/tire/HMMWV_RigidMeshTire.json"

## Semi-empirical tire models {#vehicle_tire_empirical}

The second class of tires models offered are the semi-empirical ones commonly used for vehicle handling.  Chrono::Vehicle currently has implementations for Pacejka (89 and 2002), TMeasy, and Fiala tire models. Handling tire models are designed for flat road surfaces and they normally use single point contact or four point contact (TMeasy). For ride tests on ondulated roads or for obstacle crossing a special contact algorithm called "envelope" has been implemented. It is based on a paper of Sui & Hershey and can be used with all handling tire models that are included in Chrono. Validation tests with a technology demonstrator show good results compared to measured real vehicle test data with low numerical effort. If an even better accuracy is needed the user should think about considering FEA based tire models for non-handling tests.

Some users may want to build their own handling tire parameter sets from tire test data. Be sure to get familiar with the different slip definitions and the different coordinate systems in each tire model!

### Pacejka 89 (Pac89) tire model {#wheeled_tire_pac89}

See [ChPac89Tire](@ref chrono::vehicle::ChPac89Tire) and [Pac89Tire](@ref chrono::vehicle::Pac89Tire).

### Pacejka 2002 (Pac02) tire model  {#wheeled_tire_pac02}

This model is an extension of Pacejka's earlier Magic Formula tire model with additional equations and coefficients.  Since a large number of vehicle dynamics maneuvers do not occur under steady-state slip conditions, the contact patch slip state equations are included to provide more accurate results under transient conditions (TBD!). Due to the lack of reference data sets the inflation pressure dependence terms and the large camber terms are actually not implemented.

See [ChPac02Tire](@ref chrono::vehicle::ChPac02Tire) and [Pac02Tire](@ref chrono::vehicle::Pac02Tire).

### TMeasy tire model (basic version) {#wheeled_tire_tmeasy}

TMeasy (Tire Model Made Easy) has been developed by Prof. Dr. Georg Rill and is available as a comercial code (see http://www.tmeasy.de/ for more information). The aim of this model is to allow easy parametration. Its complexity is roughly comparable to the Magic Formula based models. Furthermore, based on known parameter sets for trucks and passenger cars it is possible to estimate a complete parameter set from few input data. TMeasy considers nonlinear effects and includes contact patch slip state equations. The implementation used in Chrono lacks some functionality compared to the latest comercial TMeasy versions. For example, there is no belt dynamics and no dynamic parking torque calculation possible. The Chrono TMeasy tire implementation contains algorithms that have been published in the book [Road Vehicle Dynamics - Fundamentals and Modeling with MATLAB](https://www.routledge.com/Road-Vehicle-Dynamics-Fundamentals-and-Modeling-with-MATLAB/Rill-Castro/p/book/9780367199739), Georg Rill and Abel Arrieta Castro, CRC Press, 2020.

See [ChTMeasyTire](@ref chrono::vehicle::ChTMeasyTire) and [TMeasyTire](@ref chrono::vehicle::TMeasyTire).

A sample JSON file with a TMeasy tire specification is provided below:
\include "../../data/vehicle/hmmwv/tire/HMMWV_TMeasyTire.json"


### Fiala tire model  {#wheeled_tire_fiala}

The Fiala tire model implemented in Chrono::Vehicle is largely based on the transient Fiala tire model presented in the MSC ADAMS/tire help documentation, which uses tire slip state equations to improve the model's behavior at slow to zero forward velocities.  The Fiala tire model is based on a brush model assumption and only requires a small number of coefficients. This tire model assumes that the tire is at zero camber with respect to the road and does not have any provisions for generating overturning moments.  It does however couple the lateral and longitudinal slip states of the tire in its force and moment calculations, providing a more realistic description of combined slip. The Fiala tire model should not be used for serious vehicle handling simulations since it does not consider important effects that influence the results.

See [ChFialaTire](@ref chrono::vehicle::ChFialaTire) and [FialaTire](@ref chrono::vehicle::FialaTire).

A sample JSON file with a TMeasy tire specification is provided below:
\include "../../data/vehicle/hmmwv/tire/HMMWV_FialaTire.json"

The vertical load curve embedded in the above JSON file is show below:

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/FialaTire_vertical_load.png" width="500" />

## FEA-based tire models  {#wheeled_tire_fea}

Finally, the third class of tire models offered are full finite element representations of the tire.  While these models have the potential to be the most accurate due to their detailed physical model of the tire, they are also the most computationally expensive of the tire model currently available in Chrono::Vehicle.  Unlike the rigid or semi-empirical tire models, the finite element based tire models are able to account for the flexibility in both the tire and in the ground at the same time, which is an important characteristic for many types of off-road mobility and vehicle dynamics studies.  These finite element tire models leverage the nonlinear finite element capabilities in Chrono. 

<img src="http://www.projectchrono.org/assets/manual/vehicle/wheeled/FEA_tire_sections.png" width="600" />

### ANCF shell deformable tire {#vehicle_tire_ancf}

See [ChANCFTire](@ref chrono::vehicle::ChANCFTire) and [ANCFTire](@ref chrono::vehicle::ANCFTire).

The following JSON file contains the specification of an ANCFTire:
\include "../../data/vehicle/hmmwv/tire/HMMWV_ANCF4Tire.json"

### Reissner shell deformable tire {#vehicle_tire_reissner}

See [ChReissnerTire](@ref chrono::vehicle::ChReissnerTire) and [ReissnerTire](@ref chrono::vehicle::ReissnerTire).
