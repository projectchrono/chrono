Driveline models {#wheeled_driveline}
=====================================

\tableofcontents

The driveline subsystem (aka drivetrain) is the set of components that deliver power to the vehicle's driven wheels.
Currently, Chrono::Vehicle provides templates for traditional drivelines connected to a single motor (in the powertrain subsystem) and delivering power to one, two, or more of the vehicle's axles.

The interfaces between the driveline and powertrain and between the driveline and the driven wheels are force-displacement connections. In particular, the powertrain supplies torque to the driveline's driveshaft and the driveline sets the driveshaft's angular velocity.  Similarly, the driveline transmits and applies torques to the axles of the driven wheels and receives the current angular velocity of the wheel axles. Depending on the type of driveline and powertrain, the driveline-powertrain and driveline-wheel connections can be enforced through constraints or purely kinematically.


## Four-wheel shafts-based drivetrain {#wheeled_driveline_4WD_shafts}

This drivetrain template is modeled using various Chrono 1-D shaft elements ([ChShaft](@ref chrono::ChShaft)) and specialized shaft connection elements.  Such elements are used to model the differentials ([ChShaftsPlanetary](@ref chrono::ChShaftsPlanetary)), conical gears ([ChShaftsGearboxAngled](@ref chrono::ChShaftsGearboxAngled)), and clutches ([ChShaftsClutch](@ref chrono::ChShaftsClutch)) for differential locking.

See [ChShaftsDriveline4WD](@ref chrono::vehicle::ChShaftsDriveline4WD) and [ShaftsDriveline4WD](@ref chrono::vehicle::ShaftsDriveline4WD).

The image below shows the 4WD shafts-based driveline connected to a shafts-based powertrain model.

<img src="http://www.projectchrono.org/assets/manual/vehicle/shafts_powertrain.png" width="800" />

The various template parameters available for a 4WD shafts-based driveline are illustrated in the following sample JSON specification file:
\include "data/vehicle/hmmwv/driveline/HMMWV_Driveline4WD.json"


## Two-wheel shafts-based drivetrain {#wheeled_driveline_2WD_shafts}

This drivetrain template is similar to the 4WD shafts-based drivetrain model, but can only drive the wheels associated with a single vehicle axle.  This axle is arbitrary, so this drivetrain model can be used to model both front-wheel drive and rear-wheel drive vehicles.

See [ChShaftsDriveline2WD](@ref chrono::vehicle::ChShaftsDriveline2WD) and [ShaftsDriveline2WD](@ref chrono::vehicle::ShaftsDriveline2WD).

A sample JSON file with the specification of a 2WD shafts-based driveline is:
\include "data/vehicle/hmmwv/driveline/HMMWV_Driveline2WD.json"


## Four-wheel kinematic drivetrain {#wheeled_driveline_4WD_simple}

This template can be used to model a 4WD driveline. It uses a constant front/rear torque split (a value between 0 and 1) and a simple model for Torsen limited-slip differentials.

See [ChSimpleDriveline](@ref chrono::vehicle::ChSimpleDriveline) and [SimpleDriveline](@ref chrono::vehicle::SimpleDriveline).

A sample JSON file with the specification of a 2WD shafts-based driveline is:
\include "data/vehicle/hmmwv/driveline/HMMWV_DrivelineSimple.json"


## X-wheel kinematic drivetrain {#wheeled_driveline_XWD_simple}

This simple driveline template can be used to model a XWD driveline, capable of driving one or more vehicle axles. It uses a constant torque split depending on the number of driven axles and a simple model for Torsen limited-slip differentials.

See [ChSimpleDrivelineXWD](@ref chrono::vehicle::ChSimpleDrivelineXWD) and [SimpleDrivelineXWD](@ref chrono::vehicle::SimpleDrivelineXWD).

A sample JSON file with the specification of a 2WD shafts-based driveline is:
\include "data/vehicle/MAN_Kat1/driveline/MAN_5t_DrivelineSimpleXWD.json"
