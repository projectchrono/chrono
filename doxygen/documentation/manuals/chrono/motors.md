
Motors      {#motors}
========

*Motors* are objects that can be used to move one part respect to the other; they are used to represent robotic actuators, engines, etc.
 
In general, you should consider using motors all the times that you need to control the motion of an object respect to another. The [demo_motors](@ref tutorial_demo_motors) provides a good set of examples on their use.

There are various types of motors in Chrono: the most relevant distinction being between motors that provide rotation between 3D parts, motors that provide linear motion between 3D parts, and motors that provide motion between simplified 1D objects.

Depending on the application, it may be enough to enforce a precise angular velocity (ex. when simulating a rotating mixer), in other cases you many need to enforce a precise angle or position between two parts (ex. when simulating robots and automation, if assuming that actuators do not have compliance and the control is infinitely stiff), and in other cases you may want to enforce a torque/force load between the parts, and delegate the motion control to more sophisticated algorithms. All these modes are supported in Chrono, and are summarized in this table:


.                        |  3D rotational motors         | 3D linear motors            | 1D motors for drivelines    
------------------------ | ----------------------------- | --------------------------- | ------------------------
Impose displacement      |  ChLinkMotorRotationAngle     | ChLinkMotorLinearPosition   | ChShaftsMotorAngle      
Impose speed             |  ChLinkMotorRotationSpeed     | ChLinkMotorLinearSpeed      | ChShaftsMotorSpeed      
Impose load              |  ChLinkMotorRotationTorque    | ChLinkMotorLinearForce      | ChShaftMotorTorque      
Connect to 1D driveline  |  ChLinkMotorRotationDriveline | ChLinkMotorLinearDriveline  | -                       


<div class="ce-warning"> 
The old chrono::ChLinkEngine and chrono::ChLinkLinActuator classes are obsolete: users are encouraged to use the new ChLinkMotor classes described in this page.
</div> 


\tableofcontents


# 3D rotational motors   {#rotational_motors}

These motors connect two parts of class chrono::ChLinkBodyFrame, i.e objects that have translation+rotation in space, for example chrono::ChBody or chrono::fea::ChNodeFEAxyzrot.

All rotational motors are inherited from chrono::ChLinkMotorRotation.
All rotational motors assume that a part A rotates about a part B, where **rotation axes 
are the Z directions** of two auxiliary frames F1 and F2 connected to the two parts, so we consider the Z axis for the frames to be the "spindle" of the motor: 

![](http://www.projectchrono.org/assets/manual/pic_ChLinkMotorRotation.png)

Reactions and joint rotations/velocities are computed with respect to the *master frame*, that is frame F2. For example use ChLinkMotorRotation::GetMotorRot(), ChLinkMotorRotation::GetMotorRot_dt() and ChLinkMotorRotation::GetMotorRot_dtdt() to get the current motor angle, angular velocity, angular acceleration. Note that the angle is computed by keeping track of multiple rotations, so it is not limited in the -PI..+PI periodic range; otherwise you might use GetMotorRotPeriodic() and GetMotorRotTurns().

Note that angles are considered in [rad], not in degrees!. Ex. a 180° turn = PI [rad] = 3.1415 [rad] etc. Same for angular velocity, that is [rad/s], and angular acceleration, that is in [rad/s^2].

By default, all rotational motors also provide a revolute constraint for the other relative degrees of freedom (translation about **X**,**Y**,**Z** and rotation about **RX** and **RY**, except rotation RZ that is the one controlled by the motor) so you do not need to create additional joints, like ChLinkLockRevolute for example, to keep the two parts together. Anyway, if you prefer, this behavior can be changed by using the ChLinkMotorRotation::SetSpindleConstraint() function, that can accept the following options:

- FREE : enforces no constraint on spindle direction/alignment
- REVOLUTE: enforces **X**,**Y**,**Z**, **RX**, **RY** constraints (default)
- CYLINDRICAL: enforces **X**,**Y**, **RX**, **RY** constraints 
- OLDHAM: enforces **RX**, **RY** constraints 

There are four types of rotational motors inherited from ChLinkMotorRotation, discussed more in detail in the following sections: 

- **chrono::ChLinkMotorRotationAngle**, that enforces a rotation on Z as a angle(time) function,

- **chrono::ChLinkMotorRotationSpeed**, that enforces a rotation on Z as a speed(time) function,

- **chrono::ChLinkMotorRotationTorque**, that applies a torque(time) load between the two parts, about Z,

- **chrono::ChLinkMotorRotationDriveline**, that connects the 3D rotation between the two parts about Z, 
  to a 1D driveline of your choice, modeled with chrono::ChShaft 1D elements connected by one or 
  more 1D motors/clutches/gears/etc.
  
In general, the process of adding a motor involves the following steps:

- Create the motor from the desired ChLinkMotorXxxyyy class
- Use ```mymotor->Initialize(…)``` to connect two parts, 
  specifying the motor frame in absolute coordinates
- Add the motor to a ChSystem
- Optional: depending on link type, set link properties. Most often this 
  means setting some chrono::ChFunction object for the motor speed or angle etc.

Example:

~~~{.cpp}
// Create the motor
auto rotmotor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

// Connect the rotor and the stator and add the motor to the system:
rotmotor->Initialize( rotor,                // body A (slave)
                      stator,               // body B (master)
                      ChFrame<>(ChVector<>(1,0,0)) // motor frame, in abs. coords
                      );
                      
// Add the motor to the system
mphysicalSystem.Add(rotmotor);

// Create a ChFunction to be used for the motor: for example a constant 
// angular speed, in [rad/s], ex. 1 PI/s =180°/s
auto mwspeed = chrono_types::make_shared<ChFunction_Const>(CH_C_PI); 

// Let the motor use our motion function:
rotmotor->SetSpeedFunction(mwspeed);

~~~

In the following sub-sections you can find additional informations about the various types of rotational motors.



## ChLinkMotorRotationAngle   

The chrono::ChLinkMotorRotationAngle motor imposes angle between parts.

This is a simple but very useful type of rotational actuator. It assumes that 
you know the exact angular angle of the rotor respect to the stator,
as a function of time:  

\f[ 
 \alpha = f(t)
\f]

Use this to simulate servo drives in robotic systems and automation, 
where you can assume that the motor rotates with an infinitely stiff
and reactive control, that exactly follows your prescribed motion profiles.

Note: this is a rheonomic motor, i.e. it generates the motion
*geometrically* by strictly enforcing the angle constraint; 
no compliance is allowed, this means that if the
rotating body hits some hard contact, the solver might give unpredictable 
oscillatory or diverging results because of the contradiction.

The angle is provided by an object of class chrono::ChFunction, that returns 
the angle (in [rad]) as a function of time. Use SetAngleFunction() to set such function.

Example:

~~~{.cpp}
// Create the motor
auto rotmotor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Connect the rotor and the stator and add the motor to the system:
rotmotor2->Initialize(rotor2,                // body A (slave)
                      stator2,               // body B (master)
                      ChFrame<>(positionA2)  // motor frame, in abs. coords
                      );
mphysicalSystem.Add(rotmotor2);

// Create a ChFunction to be used for the ChLinkMotorRotationAngle
auto msineangle = chrono_types::make_shared<ChFunction_Sine>(
                                        0,      // phase [rad]
                                        0.05,   // frequency [Hz]
                                        CH_C_PI // amplitude [rad]
                                        ); 
// Let the motor use this motion function as a motion profile:
rotmotor2->SetAngleFunction(msineangle);
~~~




## ChLinkMotorRotationSpeed

The chrono::ChLinkMotorRotationSpeed motor imposes angular speed between parts.

This is one of the simplest types of rotational actuators. It assumes that 
you know the exact angular speed of the rotor respect to the stator,
as a function of time: 

\f[ 
 \omega = f(t),  \quad   \omega = \frac{d\alpha}{dt}
\f]

This type of motor is helpful when you need to simulate fans, rotating cranks,
wheeled robots, etc., where you are not so interested in actual rotation angle,
but you rather focus on speed.

Note: this is a rheonomic motor, i.e. it generates the motion
*geometrically* by strictly enforcing the speed constraint;  
therefore no compliance is allowed and this means that if the
rotating body hits some hard contact, the solver might give unpredictable 
oscillatory or diverging results because of the contradiction.
    
The angular speed is provided by an object of class chrono::ChFunction, that returns 
the angular speed (in [rad/s]) as a function of time. Use SetSpeedFunction() to set such function.

Example:

~~~{.cpp}
// Create the motor
auto rotmotor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

// Connect the rotor and the stator and add the motor to the system:
rotmotor->Initialize( rotor,                // body A (slave)
                      stator,               // body B (master)
                      ChFrame<>(ChVector<>(1,0,0)) // motor frame, in abs. coords
                      );
                      
// Add the motor to the system
mphysicalSystem.Add(rotmotor);

// Create a ChFunction to be used for the motor: 
auto mwspeed = chrono_types::make_shared<ChFunction_Const>(CH_C_PI); 

// Let the motor use our motion function:
rotmotor->SetSpeedFunction(mwspeed);
~~~

The ChLinkMotorRotationSpeed contains a hidden state that performs the time integration
of the angular speed setpoint: \f$ \alpha(t) = \int_0^t \omega dt \f$. Such angle is then imposed to the
constraint at the positional level too, thus avoiding angle error
accumulation (angle drift). Optionally, such positional constraint
level can be disabled as follows:

~~~{.cpp}
rotmotor->SetAvoidAngleDrift(false);
~~~


## ChLinkMotorRotationTorque

The chrono::ChLinkMotorRotationTorque motor imposes torque between parts.

For this motor, you must specify a time-dependent torque as: 

\f[ 
 T = f(t)
\f]

The torque is provided by an object of class chrono::ChFunction, that returns 
the Torque (in [Nm]) as a function of time. Use SetTorqueFunction() to set such function.

The torque, if constant, will accelerate indefinitely the spindle: it would have little use.
So you may need to implement some time-varying torque. This lead us to two interesting, yet advanced, uses of this motor:

- you may provide a custom function implemented by yourself (inheriting from chrono::ChFunction)
  where its Get_y() member returns a torque that changes not only because of time, but also
  because of other information: for example you can implement a torque(speed) function 
  to represent a three phase induction motor, 
  
- you may continuously adjust the value of the torque at each step of the simulation, to 
  simulate a PID controller, or a man-in-the-loop system, or other controllers; this can be achieved 
  at least in two ways:
  
  - provide a custom custom function inherited from chrono::ChFunction_SetpointCallback, where you
    implement the SetpointCallback() method (containing code that computes torque T, automatically
    called at each timestep); 
    
  - or just use a concrete chrono::ChFunction_Setpoint function, in this case you just have to 
    manually call myfunction->SetSetpoint(...) in your simulation loop


~~~{.cpp}
// Create the motor
auto rotmotor4 = chrono_types::make_shared<ChLinkMotorRotationTorque>();

// Connect the rotor and the stator and add the motor to the system:
rotmotor4->Initialize(rotor4,                // body A (slave)
                      stator4,               // body B (master)
                      ChFrame<>(positionA4)  // motor frame, in abs. coords
                      );
mphysicalSystem.Add(rotmotor4);

// Implement our custom  torque function.
// We could use pre-defined ChFunction classes like sine, constant, ramp, etc.,
// but in this example we show how to implement a custom function: a 
// torque(speed) function that represents a three-phase electric induction motor.
// Just inherit from ChFunction and implement Get_y() so that it returns different
// values (regrdless of time x) depending only on the slip speed of the motor:
class MyTorqueCurve : public ChFunction {
public: 
    // put here some data that you need when evaluating y(x):
    double E2; // voltage on coil, etc.
    double R2;
    double X2;
    double ns;
    std::shared_ptr<ChLinkMotorRotationTorque> mymotor;

    virtual MyTorqueCurve* Clone() const override { return new MyTorqueCurve(*this); }

    virtual double Get_y(double x) const override { 
        // The three-phase torque(speed) model
        double w = mymotor->GetMotorRot_dt(); 
        double s = (ns-w)/ns;// slip
        double T = (3.0/2*CH_C_PI*ns)*(s * E2*E2 * R2) / (R2*R2 + pow(s*X2,2)); // electric torque curve
        T -= w*5; // simulate also a viscous brake
        return T;
    }
};
// Create the function object from our custom class, and initialize its data:
auto mtorquespeed = chrono_types::make_shared<MyTorqueCurve>(); 
mtorquespeed->E2 = 120;
mtorquespeed->R2 = 80;
mtorquespeed->X2 = 1;
mtorquespeed->ns = 6;
mtorquespeed->mymotor = rotmotor4;

// Let the motor use this motion function as a motion profile:
rotmotor4->SetTorqueFunction(mtorquespeed);
~~~



## ChLinkMotorRotationDriveline

The chrono::ChLinkMotorRotationDriveline motor can embed a complex 1D driveline.

This is the most powerful motor type. It allows the creation of 
generic 1D powertrain inside this 3D motor. Think at this driveline as an invisible
set of one-dimensional shafts, reducers, motors, that connect the rotational degree 
of freedom of part A and the rotational degree of freedom of part B.

Powertrains/drivelines are defined by connecting a variable number of 
1D objects such as chrono::ChShaft, chrono::ChClutch, chrono::ChShaftsMotor, etc. 
In this way, for example, you can represent a drive+flywheel+reducer, hence taking into account 
of the inertia of the flywheel without the complication of adding a full 3D shape that 
represents the flywheel, and withoput needing 3D constraint for gears, bearings, etc.

The 1D driveline is "interfaced" to the two connected three-dimensional
parts using two "inner" 1D shafts, each rotating as the connected 3D part;
it is up to the user to build the driveline that connects those two shafts.

Some guidelines:

- In order to connect the two 3D parts to the 1D powertrain, you must use two 
  invisible "inner shafts" of chrono::ChShaft class, that you can access via 
  mymotor->GetInnerShaft1() and mymotor->GetInnerShaft2().  
  
- Most often the driveline is like a graph starting at inner shaft 2 (consider 
  it to be the truss for holding the motor drive, also the support for reducers 
  if any) and ending at inner shaft 1 (consider it to be the output, i.e. the 
  slow-rotation spindle).

- You may want to change the inertia of "inner" 1D shafts, (each has default 1kg/m^2).
  
  - Note: they adds up to 3D inertia when 3D parts rotate about the link shaft.
  
  - Note: do not use too small values compared to 3D inertias: it might negatively affect
    the precision of some solvers; this is especially true with the default SOR solver, 
    and less critical with BARZILAIBORWEIN or APGD or MKL solvers. Hint: if you have troubles with the
    precision of the solver, rather diminish the 3D inertia of stator/rotor parts and increase
    the inertia of the inner shafts.

- In general, the default SOR solver might have difficulties converging if you mix very 
  low-inertia parts with high-inertia parts in the driveline, for example if you model a 
  motor+reducer where the motor has a very small inertia. Either you increase the iteration 
  number or you switch to more advanced (but slower) solvers such as APGD, BARZILAIBORWEIN 
  or MKL. Anyway, in well-designed actuators for robotic applications with high dynamics, the 
  reduced inertia of the fast motor spindle is always comparable to the inertia of the moving load 
  at the output spindle of the reducer, \f$ 1/\tau^2 J_{mot}\approx J_{load}\f$ so the problem is not so perceptible.
  
- look at the manual for [1D motors](@ref shaft_motors) to understand which types of 1D motors you can 
  embed in the driveline; they can work at angle/speed/torque level, just like their 3D counterparts.
  
In the following example we will create a hidden 1D driveline made with a 1D motor and a 1D reducer (gearbox with transmission ratio \f$ \tau \f$), as often happens in robotic applications. The following picture shows the topology of the driveline that we will create:

![](http://www.projectchrono.org/assets/manual/pic_ChLinkMotorRotationDriveline.png)


~~~{.cpp}
// Create the motor
auto rotmotor5 = chrono_types::make_shared<ChLinkMotorRotationDriveline>();

// Connect the rotor and the stator and add the motor to the system:
rotmotor5->Initialize(rotor5,                // body A (slave)
                      stator5,               // body B (master)
                      ChFrame<>(positionA5)  // motor frame, in abs. coords
                      );
mphysicalSystem.Add(rotmotor5);

// You may want to change the inertia of 'inner' 1d shafts, (each has default 1kg/m^2)
// Note: using too small values compared to 3D inertias affects solver convergence.
rotmotor5->GetInnerShaft1()->SetInertia(0.2); // [kg/m^2]
rotmotor5->GetInnerShaft2()->SetInertia(0.2); // [kg/m^2]

// Now create the driveline. We want to model a drive+reducer sytem.
// This driveline must connect "inner shafts" of s1 and s2, where:
//  s1, is the 3D "rotor5"  part A (ex. a robot arm) and 
//  s2, is the 3D "stator5" part B (ex. a robot base). 
// In the following scheme, the motor is [ DRIVE ], the reducer is [ REDUCER ], 
// the shafts ( shown with symbol || to mean inertia) are: 
//  S1: the 1D inner shaft for s1 robot arm (already present in ChLinkMotorRotationDriveline)
//  S2: the 1D inner shaft for s2 robot base (already present in ChLinkMotorRotationDriveline) 
//  A : the shaft of the electric drive
//
//      S2                A                    S1
// 3d<--||---[ DRIVE ]---||-----[ REDUCER ]----||-->3d
// s2   ||                      [         ]         s1
//      ||----------------------[         ]
//                    

// Create 'A', a 1D shaft. This is the shaft of the electric drive, representing its inertia.

auto my_shaftA = chrono_types::make_shared<ChShaft>();
my_shaftA->SetInertia(0.03);
mphysicalSystem.Add(my_shaftA);

// Create 'DRIVE', the hi-speed motor model - as a simple example use a 'imposed speed' motor: this 
// is the equivalent of the ChLinkMotorRotationSpeed, but for 1D elements:

auto my_drive = chrono_types::make_shared<ChShaftsMotorSpeed>();
my_drive->Initialize(
        my_shaftA,                   // A , the rotor of the drive
        rotmotor5->GetInnerShaft2()  // S2, the stator of the drive 
        );
mphysicalSystem.Add(my_drive);
// Create a speed(time) function, and use it in my_drive:
auto my_driveangle = chrono_types::make_shared<ChFunction_Const>(25*CH_C_2PI);  // 25 [rps] = 1500 [rpm]
my_drive->SetSpeedFunction(my_driveangle); 


// Create the REDUCER. We should not use the simple ChShaftsGear because
// it does not transmit torque to the support. So use ChShaftsPlanetary 
// and use it in ordinary mode, keeping the carrier as truss: so it
// will connect three parts: the carrier(here the truss), the in shaft, the out shaft.

auto my_reducer = chrono_types::make_shared<ChShaftsPlanetary>();
my_reducer->Initialize(
    rotmotor5->GetInnerShaft2(),// S2, the carrier (truss) 
    my_shaftA,                  // A , the input shaft
    rotmotor5->GetInnerShaft1() // S1, the output shaft
    );
my_reducer->SetTransmissionRatioOrdinary(1.0/100.0); // ratio between wR/wA
mphysicalSystem.Add(my_reducer);
~~~




# 3D linear motors {#linear_motors}

These motors connect two parts of class chrono::ChLinkBodyFrame, i.e objects that have translation+rotation in space, for example chrono::ChBody or chrono::fea::ChNodeFEAxyzrot.

All linear motors are inherited from chrono::ChLinkMotorLinear. 
They assume that a part A translates about a part B, where **the guide is the X direction of the auxiliary frame F2**, and the slider is the auxiliary frame F1; both frames being connected to the respective body.
 

![](http://www.projectchrono.org/assets/manual/pic_ChLinkMotorLinear.png)

Reactions and joint rotations/velocities are computed with respect to the *master frame*, that is frame F2. For example use ChLinkMotorLinear::GetMotorPos(), ChLinkMotorRotation::GetMotorPos_dt() and ChLinkMotorRotation::GetMotorPos_dtdt() to get the current motor displacement, velocity, acceleration. 

By default, all linear motors also provide a prismatic constraint for the other relative degrees of freedom (translation about **Y**,**Z** and rotation about **RX**, **RY, **RZ**, except rotation translation Z that is the one controlled by the motor) so you do not need to create additional joints, like ChLinkLockPrismatic for example, to keep the two parts together. Anyway, if you prefer, this behavior can be changed by using the ChLinkMotorLinear::SetGuideConstraint() function, that can accept the following options:

- FREE : enforces no constraint on roller direction/alignment
- PRISMATIC: enforces **Y**,**Z**, **RX**, **RY**, **RZ** constraints (default)
- SPHERICAL: enforces **Y**,**Z**  constraints 

There are four types of rotational motors inherited from ChLinkMotorLinear, discussed more in detail in the following sections: 

- **chrono::ChLinkMotorLinearPosition**, that enforces a motion on X as a position(time) function,

- **chrono::ChLinkMotorLinearSpeed**, that enforces a motion on X as a speed(time) function,

- **chrono::ChLinkMotorLinearForce**, that applies a force(time) load between the two parts, about X,

- **chrono::ChLinkMotorLinearDriveline**, that connects the 3D motion between the two parts about X, 
  to a 1D driveline of your choice, modeled with chrono::ChShaft 1D elements connected by one or 
  more 1D motors/clutches/gears/etc.
  
In general, the process of adding a motor involves the following steps:

- Create the motor from the desired ChLinkMotorLinearXxxyyy class
- Use ```mymotor->Initialize(…)``` to connect two parts, 
  specifying the motor frame in absolute coordinates
- Add the motor to a ChSystem
- Optional: depending on link type, set link properties. Most often this 
  means setting some chrono::ChFunction object for the motor speed or position etc.

In the following sub-sections you can find additional informations about the various types of rotational motors.



## ChLinkMotorLinearPosition   

The chrono::ChLinkMotorLinearPosition motor imposes a displacement between parts.

This is a simple but very useful type of linear actuator. It assumes that 
you know the exact position of the slider along the X axis of the guide, \f$ p_x \f$
as a function of time:  

\f[ 
 p_x = f(t)
\f]

Use this to simulate linear actuators in robotic systems and automation, 
where you can assume that the slider moves with an infinitely stiff
and reactive control that exactly follows your prescribed motion profile.
(in a real world this is just an approximation, because even the best 
motor will have some oscillatory behavior around the desired setpoint, as the
control system have some compliance and latency; yet if this can be neglected, this motor
is the best choice). 

Note: this is a rheonomic motor, i.e. it generates the motion
*geometrically* by strictly enforcing the displacement constraint; 
no compliance is allowed, this means that if the
sliding body hits some hard contact, the solver might give unpredictable 
oscillatory or diverging results because of the contradiction.

The position is provided by an object of class chrono::ChFunction, that returns 
the position (in [m]) as a function of time. Use SetPositionFunction() to set such function.

Example:

~~~{.cpp}
// Create the linear motor
auto motor1 = chrono_types::make_shared<ChLinkMotorLinearPosition>();

// Connect the guide and the slider and add the motor to the system:
motor1->Initialize(slider1,              // body A (slave)
                   guide1,               // body B (master)
                   ChFrame<>(ChVector<>(3,1,0))  // motor frame, in abs. coords
                   );
mphysicalSystem.Add(motor1);

// Create a ChFunction to be used for the ChLinkMotorLinearPosition
auto msine = chrono_types::make_shared<ChFunction_Sine>(
                                     0,     // phase
                                     0.5,   // frequency
                                     1.6    // amplitude
                                    );
// Let the motor use this motion function:
motor1->SetMotionFunction(msine);
~~~




## ChLinkMotorLinearSpeed

The chrono::ChLinkMotorLinearSpeed motor imposes a speed between parts.

This is one of the simplest types of linear actuators. It assumes that 
you know the exact speed of the slider along the X direction of the guide, \f$ v_x \f$
as a function of time: 

\f[ 
 v_x = f(t),  \quad   v_x = \frac{dp_x}{dt}
\f]

Note: this is a rheonomic motor, i.e. it generates the motion
*geometrically* by strictly enforcing the speed constraint;  
therefore no compliance is allowed and this means that if the
sliding body hits some hard contact, the solver might give unpredictable 
oscillatory or diverging results because of the contradiction.
    
The speed is provided by an object of class chrono::ChFunction, that returns 
the speed (in [m/s]) as a function of time. Use SetSpeedFunction() to set such function.

Example:

~~~{.cpp}
// Create the linear motor
auto motor2 = chrono_types::make_shared<ChLinkMotorLinearSpeed>();

// Connect the guide and the slider and add the motor to the system:
motor2->Initialize(slider2,              // body A (slave)
                   guide2,               // body B (master)
                   ChFrame<>(positionB2)  // motor frame, in abs. coords
                   );
mphysicalSystem.Add(motor2);

// Create an harmonic ChFunction to be used for the ChLinkMotorLinearSpeed
auto msp = chrono_types::make_shared<ChFunction_Sine>(CH_C_PI_2,  // phase
                                             0.5,        // frequency
                                             0.8         // amplitude
                                             );
// Let the motor use this motion function:
motor2->SetSpeedFunction(msp);
~~~

The ChLinkMotorLinearSpeed contains a hidden state that performs the time integration
of the speed setpoint: \f$ p_x(t) = \int_0^t v_x dt \f$. Such \f$ p_x \f$ is then imposed to the
constraint at the positional level too, thus avoiding error accumulation (position drift). 
Optionally, such positional constraint level can be disabled as follows:

~~~{.cpp}
motor->SetAvoidPositionDrift(false);
~~~


## ChLinkMotorLinearForce

The chrono::ChLinkMotorLinearForce motor imposes a force between parts.

For this motor, you must specify a time-dependent force as: 

\f[ 
 F = f(t)
\f]

The force is provided by an object of class chrono::ChFunction, that returns 
the Torque (in [Nm]) as a function of time. Use SetForceFunction() to set such function.

The force, if constant, will accelerate indefinitely the slider: it would have little use.
Similarly, an "open loop" option is to provide a nonlinear \f$ F(t) \f$ at the beginning 
(ex using a feedforward model), but then there is no guarantee about the
precise position of the slider, when the simulation runs.

This means that, unless you update the force F at each time
step using some type of feedback controller, this actuator
cannot be used to follow some position setpoint. Implementing
your controller might complicate things, but it could be closer to
the behavior of a real actuator, that have some delay, bandwidth
latency and compliance - for example, differently from
other types such as ChLinkMotorLinearPosition  and
ChLinkMotorLinearSpeed, this force motor does not enforce any
constraint on the direction of motion, so if it the slider hits
some hard contact, it just stops and keeps pushing, and no troubles
with the solver happen.

So you may need to implement some time-varying force. Exactly as happens for
the ChLinkMotorRotationTorque, this lead us to two interesting and advanced
uses of this motor:

- you may provide a custom function implemented by yourself (inheriting from chrono::ChFunction)
  where its Get_y() member returns a torque that changes not only because of time, but also
  because of other information: for example you can implement a force(speed) function 
  to represent a three phase induction motor plus a ballscrew reducer, 
  
- you may continuously adjust the value of the torque at each step of the simulation, to 
  simulate a PID controller, or a man-in-the-loop system, or other controllers; this can be achieved 
  at least in two ways:
  
  - provide a custom custom function inherited from chrono::ChFunction_SetpointCallback, where you
    implement the SetpointCallback() method (containing code that computes torque T, automatically
    called at each timestep); 
    
  - or just use a concrete chrono::ChFunction_Setpoint function, in this case you just have to 
    manually call myfunction->SetSetpoint(...) in your simulation loop

    
An example is represented here, that is a PID controller implemented using the approach of a ChFunction_SetpointCallback:

~~~{.cpp}
// Create the linear motor
auto motor4 = chrono_types::make_shared<ChLinkMotorLinearForce>();

// Connect the guide and the slider and add the motor to the system:
motor4->Initialize(slider4,              // body A (slave)
                   guide4,               // body B (master)
                   ChFrame<>(positionB4)  // motor frame, in abs. coords
                   );
mphysicalSystem.Add(motor4);

// Create a ChFunction that computes F by a user-defined algorithm, as a callback.
// One quick option would be to inherit from the ChFunction base class, and implement the Get_y() 
// function by putting the code you wish, as explained in demo_CH_functions.cpp. However this has some
// limitations. A more powerful approach is to inherit from ChFunction_SetpointCallback, that automatically
// computes the derivatives, if needed, by BDF etc. Therefore:
// 1. You must inherit from the ChFunction_SetpointCallback base class, and implement the SetpointCallback() 
//    function by putting the code you wish. For example something like the follow:

class MyForceClass : public ChFunction_SetpointCallback {
  public:
    // Here some specific data to be used in Get_y(), 
    // add whatever you need, ex:
    double setpoint_position_sine_amplitude;
    double setpoint_position_sine_freq;
    double controller_P; // for our basic PID
    double controller_D; // for our basic PID
    double last_time;
    double last_error;
    double F;
    std::shared_ptr<ChLinkMotorLinearForce> linearmotor; // may be useful later

    // Here we will compute F(t) by emulating a very basic PID scheme.
    // In practice, it works like a callback that is executed at each time step.
    // Implementation of this function is mandatory!!! 
    virtual double SetpointCallback(const double x) override { 
        // Trick: in this PID example, we need the following if(..)  to update PID 
        // only when time changes (as the callback could be invoked more than once per timestep):
        double time = x;
        if (time > last_time) {
            double dt = time - last_time;
            // for example, the position to chase is this sine formula:
            double setpoint = setpoint_position_sine_amplitude * sin(setpoint_position_sine_freq*CH_C_2PI* x );
            double error    = setpoint - linearmotor->GetMotorPos();
            double error_dt = (error-last_error)/dt;
            // for example, finally compute the force using the PID idea:
            F =  controller_P * error + controller_D * error_dt;
            last_time = time;
            last_error = error;
        }
        return F;    
    }
 };

// 2. Create the function from the custom class...
auto mFcallback = chrono_types::make_shared<MyForceClass>();
//    ...and initialize its custom data
mFcallback->setpoint_position_sine_amplitude = 1.6;
mFcallback->setpoint_position_sine_freq = 0.5;
mFcallback->controller_P = 42000; // proportional P term in PID, the higher, the "stiffer" the control
mFcallback->controller_D = 1000;  // derivative D term in PID, the higher, the more damped
mFcallback->last_time = 0; 
mFcallback->last_error = 0;
mFcallback->F = 0;
mFcallback->linearmotor = motor4;

// 3. Let the motor use our custom force:
motor4->SetForceFunction(mFcallback);
~~~



## ChLinkMotorLinearDriveline

The chrono::ChLinkMotorLinearDriveline motor can embed a complex 1D driveline.

This is the most powerful linear motor type. It allows the creation of 
generic 1D powertrain inside this 3D motor. Think at this driveline as an invisible
set of one-dimensional shafts, reducers, motors, that connect the translational degree 
of freedom of part A and the rotational degree of freedom of part B.

Powertrains/drivelines are defined by connecting a variable number of 
1D objects such as chrono::ChShaft, chrono::ChClutch, chrono::ChShaftsMotor, etc. 
In this way, for example, you can represent a drive+flywheel+reducer+pulley, 
hence taking into account of the inertia of the flywheel without the complication of adding a full 3D shape that represents the flywheel, and withoput needing 3D constraint for gears, bearings, etc.

The 1D driveline is "interfaced" to the two connected three-dimensional
parts using two "inner" 1D shafts, each representing the 1 DOF for the translation
of the connected 3D part along the guide direction;
it is up to the user to build the driveline that connects those two shafts.
    

Some guidelines:

- In order to connect the two 3D parts to the 1D powertrain, you must use two 
  invisible "inner shafts" of chrono::ChShaft class, that you can access via 
  mymotor->GetInnerShaft1lin() and mymotor->GetInnerShaft2lin().  
  
- Note: in the part 2 there is an additional inner shaft that operates on rotation;
  this is needed because, for example, maybe you want to model a driveline like a 
  drive+screw; you will anchor the drive to part 2 using this rotational shaft; so 
  reaction torques arising because of inner flywheel accelerations can be transmitted 
  to this shaft. Use  mymotor->GetInnerShaft2rot()  to access it.
  
- Most often the driveline is like a graph starting at inner shaft 2 (consider 
  it to be the truss of the linear guide, that holds the motor drive and supports for reducers 
  if any) and ending at inner shaft 1 (consider it to be the output, i.e. the 
  slider).

- You may want to change the inertia of "inner" 1D shafts, (each has default 1kg).
  
  - Note: they adds up to 3D inertia when 3D parts translate about the guide.
  
  - Note: do not use too small values compared to 3D inertias: it might negatively affect
    the precision of some solvers; this is especially true with the default SOR solver, 
    and less critical with BARZILAIBORWEIN or APGD or MKL solvers. Hint: if you have troubles with the
    precision of the solver, rather diminish the 3D inertia of guide/slider parts and increase
    the inertia of the inner shafts.

- In general, the default SOR solver might have difficulties converging if you mix very 
  low-inertia parts with high-inertia parts in the driveline, for example if you model a 
  motor+reducer where the motor has a very small inertia. Either you increase the iteration 
  number or you switch to more advanced (but slower) solvers such as APGD, BARZILAIBORWEIN 
  or MKL. Anyway, in well-designed actuators for robotic applications with high dynamics, the 
  reduced inertia of the fast motor spindle is always comparable to the inertia of the moving load 
  at the output spindle of the reducer, \f$ 1/\tau^2 J_{mot}\approx J_{load}\f$ so the problem is not so perceptible.
  
- look at the manual for [1D motors](@ref shaft_motors) to understand which types of 1D motors you can 
  embed in the driveline; they can work at position/speed/force level, just like their 3D counterparts.
    
In the following example we will create a hidden 1D driveline made with a 1D motor and a 1D reducer (rack-pinion with transmission ratio \f$ \tau \f$), as often happens in robotic applications. The following picture shows the topology of the driveline that we will create:

![](http://www.projectchrono.org/assets/manual/pic_ChLinkMotorLinearDriveline.png)

In this example, also, we'll use a motion profile that consists in a cubic ascending ramp, a pause, a descending ramp, a pause, and then repeated forever; to this end we'll use chrono::ChFunction_Sequence and chrono::ChFunction_Repeat

~~~{.cpp}
// Create the motor
auto motor5 = chrono_types::make_shared<ChLinkMotorLinearDriveline>();

// Connect the rotor and the stator and add the motor to the system:
motor5->Initialize(   slider5,              // body A (slave)
                      guide5,               // body B (master)
                      ChFrame<>(positionB5) // motor frame, in abs. coords
                      );
mphysicalSystem.Add(motor5);

// You may want to change the inertia of 'inner' 1D shafts, ("translating" shafts: each has default 1kg)
// Note: do not use too small values compared to 3D inertias: it might slow solver convergence.
motor5->GetInnerShaft1lin()->SetInertia(3.0); // [kg]
motor5->GetInnerShaft2lin()->SetInertia(3.0); // [kg]
motor5->GetInnerShaft2rot()->SetInertia(0.8); // [kg/m^2] 

// Now create the driveline. We want to model a drive+reducer sytem.
// This driveline must connect "inner shafts" of s1 and s2, where:
//  s1, is the 3D "slider5" part B (ex. a gantry robot gripper) and 
//  s2, is the 3D "guide5"  part A (ex. a gantry robot base). 
// In the following scheme, the motor is [ DRIVE ], the reducer is [ RACKPINION ], 
// the shafts ( shown with symbol || to mean inertia) are: 
//  S1: the 1D inner shaft for s1 robot arm (already present in ChLinkMotorLinearDriveline)
//  S2: the 1D inner shaft for s2 robot base (already present in ChLinkMotorLinearDriveline) 
//  B : the shaft of the electric drive
// Note that s1 and s2 are translational degrees of freedom, differently 
// from ChLinkMotorLinearDriveline.
//
//     S2rot              B                      S1lin
//    <--||---[ DRIVE ]---||-----[ RACKPINION ]----||--> 3d 
// 3d <                          [            ]          s1 
// s2 < S2lin                    [            ]
//    <--||----------------------[            ]
//  

// Tell the motor that the inner shaft  'S2rot' is Z, orthogonal to 
// the X direction of the guide (default was X, as for screw actuators)

motor5->SetInnerShaft2RotDirection(VECT_Z); // in link coordinates

// Create 'B', a 1D shaft. This is the shaft of the electric drive, representing its inertia. 

auto my_shaftB = chrono_types::make_shared<ChShaft>();
my_shaftB->SetInertia(0.33);  // [kg/m^2]
mphysicalSystem.Add(my_shaftB);

// Create 'DRIVE', the hispeed motor - as a simple example use a 'imposed speed' motor: this 
// is the equivalent of the ChLinkMotorRotationAngle, but for 1D elements:

auto my_driveli = chrono_types::make_shared<ChShaftsMotorAngle>();
my_driveli->Initialize(
                    my_shaftB,                   // B    , the rotor of the drive
                    motor5->GetInnerShaft2rot()  // S2rot, the stator of the drive  
        );
mphysicalSystem.Add(my_driveli);

// Create a angle(time) function. It could be something as simple as
//   auto my_functangle = chrono_types::make_shared<ChFunction_Ramp>(0,  180);  
// but here we'll rather do a back-forth motion, made with a repetition of a sequence of 4 basic functions:

auto my_functsequence = chrono_types::make_shared<ChFunction_Sequence>();
auto my_funcsigma1 = chrono_types::make_shared<ChFunction_Sigma>( 180, 0 , 0.5); // diplacement, t_start, t_end 
auto my_funcpause1 = chrono_types::make_shared<ChFunction_Const>(0);  
auto my_funcsigma2 = chrono_types::make_shared<ChFunction_Sigma>(-180, 0 , 0.3); // diplacement, t_start, t_end 
auto my_funcpause2 = chrono_types::make_shared<ChFunction_Const>(0);  
my_functsequence->InsertFunct(my_funcsigma1, 0.5, 1.0, true); // fx, duration, weight, enforce C0 continuity
my_functsequence->InsertFunct(my_funcpause1, 0.2, 1.0, true); // fx, duration, weight, enforce C0 continuity
my_functsequence->InsertFunct(my_funcsigma2, 0.3, 1.0, true); // fx, duration, weight, enforce C0 continuity
my_functsequence->InsertFunct(my_funcpause2, 0.2, 1.0, true); // fx, duration, weight, enforce C0 continuity
auto my_functangle = chrono_types::make_shared<ChFunction_Repeat>();
my_functangle->Set_fa(my_functsequence);
my_functangle->Set_window_length(0.5+0.2+0.3+0.2);
my_driveli->SetAngleFunction(my_functangle); 

// Create the RACKPINION. 
// It will connect three parts: 
// - S2lin, the carrier (here the truss) to transmit reaction force, 
// - B,     the in (rotational) shaft, 
// - S1lin, the out (translational) shaft.

auto my_rackpinion = chrono_types::make_shared<ChShaftsPlanetary>();
my_rackpinion->Initialize(
    motor5->GetInnerShaft2lin(),// S2lin, the carrier (truss) 
    my_shaftB,                  // B,     the input shaft
    motor5->GetInnerShaft1lin() // S1lin, the output shaft
    );
my_rackpinion->SetTransmissionRatios(-1, -1.0/100.0, 1);
mphysicalSystem.Add(my_rackpinion);
~~~





# 1D motors {#shaft_motors}

These motors are used for building schematic drivelines using 1D parts of class chrono::ChShaft, i.e. objects that have only a degree of freedom (ex. the rotation angle) and an inertia. 
 
Drivelines can be used also in chrono::ChLinkMotorRotationDriveline and ChLinkMotorLinearDriveline, interfacing them with 3D parts. 


There are three types of 1D motors inherited from ChShaftsMotor, and they discussed more in detail in the following sections: 

- **chrono::ChShaftsMotorAngle**, that enforces a rotation on Z as a angle(time) function,

- **chrono::ChShaftsMotorSpeed**, that enforces a rotation on Z as a speed(time) function,

- **chrono::ChShaftsMotorTorque**, that applies a torque(time) load between the two shafts,

 
  
In general, the process of adding a 1D motor involves the following steps:

- Create the motor from the desired ChShaftMotorXxxyyy class
- Use ```mymotor->Initialize(…)``` to connect two ChShaft objects, 
- Add the motor to a ChSystem
- Optional: depending on link type, set link properties. Most often this 
  means setting some chrono::ChFunction object for the motor speed or angle or torque etc.

In the following sub-sections you can find additional informations about the various types of rotational motors.



## ChShaftsMotorAngle   

The chrono::ChShaftsMotorAngle motor imposes an angle, in [rad], between two chrono::ChShafts objects.

This is a simple but very useful type of rotational actuator. It assumes that 
you know the exact angular angle of the rotor respect to the stator,
as a function of time:  

\f[ 
 \alpha = f(t)
\f]

Note: most often a ChShaft represent a 1DOF rotation, with rotational inertia [kg/m^2], but given the
dimension-less design of Chrono, if you need you can also consider it as a 1DOF translation, 
with linear inertia [m] like for a translating one degree of freedom. In this case, the 
chrono::ChShaftsMotorAngle would represent an imposed position displacement [m].

Use this to simulate servo drives in robotic systems and automation, 
where you can assume that the motor rotates with an infinitely stiff
and reactive control, that exactly follows your prescribed motion profiles.

Note: this is a rheonomic motor, i.e. it generates the motion
*geometrically* by strictly enforcing the angle constraint; 
no compliance is allowed.

The angle is provided by an object of class chrono::ChFunction, that returns 
the angle (in [rad]) as a function of time. Use SetAngleFunction() to set such function.



## ChShaftsMotorSpeed

The chrono::ChShaftsMotorSpeed motor imposes speed between two chrono::ChShafts objects.

This is one of the simplest types of rotational actuators. It assumes that 
you know the exact angular speed of the rotor respect to the stator,
as a function of time: 

\f[ 
 \omega = f(t),  \quad   \omega = \frac{d\alpha}{dt}
\f]

This type of motor is helpful when you need to simulate fans, rotating cranks,
wheeled robots, etc., where you are not so interested in actual rotation angle,
but you rather focus on speed.

Note: this is a rheonomic motor, i.e. it generates the motion
*geometrically* by strictly enforcing the speed constraint;  
therefore no compliance is allowed.
    
The angular speed is provided by an object of class chrono::ChFunction, that returns 
the angular speed (in [rad/s]) as a function of time. Use SetSpeedFunction() to set such function.

The ChShaftsMotorSpeed contains a hidden state that performs the time integration
of the angular speed setpoint: \f$ \alpha(t) = \int_0^t \omega dt \f$. Such angle is then imposed to the
constraint at the positional level too, thus avoiding angle error
accumulation (angle drift). Optionally, such positional constraint
level can be disabled as follows:

~~~{.cpp}
rotmotor->SetAvoidAngleDrift(false);
~~~


## ChShaftsMotorTorque

The chrono::ChShaftsMotorTorque motor imposes torque between two chrono::ChShafts objects.

For this motor, you must specify a time-dependent torque as: 

\f[ 
 T = f(t)
\f]

The torque is provided by an object of class chrono::ChFunction, that returns 
the Torque (in [Nm]) as a function of time. Use SetTorqueFunction() to set such function.

The torque, if constant, will accelerate indefinitely the spindle: it would have little use.
So you may need to implement some time-varying torque. This lead us to two interesting, yet advanced, uses of this motor:

- you may provide a custom function implemented by yourself (inheriting from chrono::ChFunction)
  where its Get_y() member returns a torque that changes not only because of time, but also
  because of other information: for example you can implement a torque(speed) function 
  to represent a three phase induction motor, 
  
- you may continuously adjust the value of the torque at each step of the simulation, to 
  simulate a PID controller, or a man-in-the-loop system, or other controllers; this can be achieved 
  at least in two ways:
  
  - provide a custom custom function inherited from chrono::ChFunction_SetpointCallback, where you
    implement the SetpointCallback() method (containing code that computes torque T, automatically
    called at each timestep); 
    
  - or just use a concrete chrono::ChFunction_Setpoint function, in this case you just have to 
    manually call myfunction->SetSetpoint(...) in your simulation loop


  
# How to control motors  {#how_to_control_motors}

A typical scenario is where you want to control an actuator during the simulation. The basic behavior of the motors described above is not enough, as the simple approach of setting positions as function of time is something that can be done once at the beginning of the simulation, and it does not support run-time updates. Here we discuss how to implement run-time control of motors: this can happen, for instance, in the following cases:

- for **man-in-the-loop** interactive simulations, for instance in videogames, where someone operates on 
  a joystick in order to steer the wheels, or to drive the arm of an excavator, etc.
  
- for **hardware-in-the-loop** simulations, where the actuator rotation (or speed, or torque, etc.)
  depends on the input from some external device
  
- for **simulation of closed-loop control** like PID controllers, digital servos with feedback 
  algorithms, etc.

- etc.

In all these cases, the common denominator is that you need to change the motor properties continuously during the simulation.
There are many ways to do this, but here we list the suggested approaches:

- **Constraint-based approach**.
  1. First create a motor that *imposes position exactly via a constraint*, that is one of the
     following:
     - **chrono::ChLinkMotorRotationAngle**, if you deal with a rotational actuator
     - **chrono::ChLinkMotorLinearPosition**, if you deal with a linear actuator
     - whatever constraint that interfaces with a 1D driveline (chrono::ChLinkMotorRotationDriveline or 
       chrono::ChLinkMotorLinearDriveline), then add a **chrono::ChShaftsMotorAngle** in the driveline.
  2. You might be tempted to use a ChFunction_Const for the angle/position, and then 
     continuously change its value using myfunct->Set_yconst() during the simulation loop: this
     would work only roughly, because these constraint-based motors require also the derivative in 
     order to work smoothly. So the proper ways to control the angle/position of the motor, with
     automatic computation of derivative of the setpoint, is one of the following:
     - provide a custom custom function inherited from chrono::ChFunction_SetpointCallback, where you
       implement the SetpointCallback() method (containing code that computes angle/position, automatically
       called at each timestep); 
     - or just use a concrete chrono::ChFunction_Setpoint function, in this case you just have to 
       manually call myfunction->SetSetpoint(...) in your simulation loop, specifying the angle/position.
       
- **Load-based approach**
  1. First create a motor that *applies a load*, that is one of the
     following:
     - **chrono::ChLinkMotorRotationTorque**, if you deal with a rotational actuator
     - **chrono::ChLinkMotorLinearForce**, if you deal with a linear actuator
     - whatever constraint that interfaces with a 1D driveline (chrono::ChLinkMotorRotationDriveline or 
       chrono::ChLinkMotorLinearDriveline), then add a **chrono::ChShaftsMotorTorque** in the driveline.
  2. Use SetMotorForce() or SetMotorTorque() methods and pass one of the following:
     - provide a custom custom function inherited from chrono::ChFunction_SetpointCallback, where you
       implement the SetpointCallback() method (containing code that computes torque/force, automatically
       called at each timestep); 
     - or just use a concrete chrono::ChFunction_Setpoint function, in this case you just have to 
       manually call myfunction->SetSetpoint(...) in your simulation loop, specifying the torque/force.
  
The constraint-based approach is suggested if you need a very efficient and idealized actuator model, and you do not care about the control model: the motor reacts instantly to your input, regardless if in real life the control system would be able to reach the set-point or not (assumption of infinitely reactive, infinitely stiff control). Ex: videogames, real-time simulators, etc. 

Remember that a side effect of constraint-based motors is that if the moving parts hit some immovable obstacles you may fall into a contradictory situation where constraints cannot be satisfied at once, so the solver might give oscillatory or unstable results.

The load-based approach is suggested if you want to introduce a control model, usually a closed-loop PID controller. It is up to you to compute a position setpoint, to compute the current error, and to compute the PID control output to be used in the motor as a torque/force setpoint. 

Since a controller must be implemented, the load-based approach is more complicate; however there is the benefit of a more realistic behavior of the actuator (the motion has oscillatory errors, delays, latencies, overshooting, elastic compliance, etc. exactly in a real servo actuator). Also, differently from constraint-based motors, you do not have problems in case the moving parts hit some immovable obstacles, as there are no contradictions with other constraints.


# Examples

See also:

- [demo_motors](@ref tutorial_demo_motors)


