Driver subsystem {#vehicle_driver}
==================================

## Interactive driver

The interactive driver can drive (steer/accelerate/brake) a simulated vehicle by user input. It is only possible with Irrlicht based programs.
- key 'a' increases left turn steering wheel angle
- key 'd' increases right turn steering wheel angle
- key 'w' increases throttle pedal position
- key 's' increases brake pedal postion

## Data-based (Open Loop) driver
Some important vehicle test manoeuvers are based on time dependend steering/throttling/brake signals. No feedback of any kind is considered. Just a setting is given and we can look at the vehicle's reaction.

An ASCII data file with driver inputs contains four columns, for time (s), steering input (a non-dimensional quantitiy in \f$[-1,1]\f$, with \f$-1\f$ indicating full steering to the left), throttle input (a non-dimensional quantity between \f$[0,1]\f$, with \f$1\f$ indicating full throttle), and braking (a non-dimensional quantity between \f$[0,1]\f$, with \f$1\f$ indicating full braking force).

A sample driver data file is listed below
\include "../../data/vehicle/generic/driver/Sample_Maneuver.txt"

<img src="http://www.projectchrono.org/assets/manual/vehicle/curves/DataDriver.png" width="500" />

At any given time, the current driver inputs (steering, throttle, and braking) are obtained by piece-wise linear interpolation of the provided data.  Beyond the last time entry, driver inputs are kept constant at their last values.


## Close-loop driver models

- path-follower
- constant-speed controller

