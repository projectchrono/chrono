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

Close-loop driver models need a control strategy and consider feedback. Feedback can lead to instable behavior, so the controller parameters have to be chosen wisely. The example parameters have been working in a wide range of use. The user should start with one of the given example parameter sets and only modify it if necessary.

- path-follower

To make the vehicle follow a given path it is necessary to measure the lateral path deviation and to generate a steering wheel angle that minimizes the deviation. A well known solution for this problem is the PID controller (P=proportional, I=Integral, D=Differential). Taking the pure P variant one only needs to set the P-gain. This will work in many cases but pure P controllers can never reduce the lateral deviation to zero. The residual deviation decreases with increasing P-gain. If the P-gain is too large, the vehicle begins to oscillate around the demanded vehicle path. A residual path deviation can be eliminated by setting the I-gain to a value of around 5% to 10% of the P-gain. By setting the D-gain it is possible to apply damping if a path oscillation occurs. If the I-gain is used, the simulated manoeuver should not take longer than about 2 minutes. If it takes more time, the controller states should be resetted every 2 minutes to avoid instabilities.

The next chart (wikipedia) shows how chrono implements the PID controllers:

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/PID_en.svg/2880px-PID_en.svg.png" width="500"/>

- r(t) = demanded path signal
- e(t) = lateral path deviation
- y(t) = actual path signal
- u(t) = steering signal

This chart (wikipedia) shows the influence of the gain factors:

<img src="https://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif" width="500"/>

A human driver does not only react on deviation changes, he is able to anticipate. In chrono this ability is simulated by taking a reference point for measuring the deviation in front of the vehicle body. The distance of the reference point from the vehicle reference system is called look-ahead-distance and is an important controller input parameter.

- constant-speed controller

To maintain a given vehicle speed the PID controller can be used. The difference to the path controller is that it uses a speed deviation instead of the lateral path deviation. 

- r(t) = demanded speed signal
- e(t) = speed deviation
- y(t) = actual speed signal
- u(t) = throttle/brake signal

The class ChPathFollowerDriver implements the PID lateral controller in combination with a PID speed controller. It works well at extreme manoeuvers like double lane change.

An interesting alternative for standard road driving manoeuvers is ChPathFollowerDriverSR. It has a PID speed controller but takes lateral control strategy that considers human and vehicle properties. The anticipation uses look-ahead time instead of a look-ahead distance which means the effective look-ahead distance varies with the speed.

- optimal-speed controller

The constant-speed controller is good for many standard driving manoeuvers. For driving on a long and curved road it is interesting how fast the vehicle can negociate the whole course. For cases like this the ChHumanDriver class has been developed. Both - the lateral and the speed controllers - use human bevavior, vehicle properties and anticipation as well as field of view of the driver. The lateral controller is identical to ChPathFollowerDriverSR.



