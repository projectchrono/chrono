
Motors      {#motors}
========

Motors are objects that can be used to impose a motion, either _linear_ (@ref chrono::ChLinkMotorLinear "ChLinkMotorLinear") or _rotational_ (@ref chrono::ChLinkMotorRotation "ChLinkMotorRotation"), either to 1D (i.e. @ref chrono::ChShaft "ChShaft") or 3D objects (i.e. @ref chrono::ChBody "ChBody").

Since @ref chrono::ChLinkMotorRotation "ChLinkMotorRotation" and @ref chrono::ChLinkMotorLinear "ChLinkMotorLinear" inherit from @ref chrono::ChLinkMate "ChLinkMate" all the considerations made on this latter apply also for these motors.

With some exceptions, the motion is controlled through @ref chrono::ChFunction "ChFunction" objects, discussed [here](@ref ChFunction_objects).

| -                        | 3D Rotational                                                            | 3D Linear                                                            | 1D Linear/Rotational                                        |   
| :----------------------- | :---------------------------------------------------------------------:  | :------------------------------------------------------------------: | :--------------------------------------------------------:  |
| Impose displacement      | @ref chrono::ChLinkMotorRotationAngle "ChLinkMotorRotationAngle"         | @ref chrono::ChLinkMotorLinearPosition "ChLinkMotorLinearPosition"   | @ref chrono::ChShaftsMotorPosition "ChShaftsMotorPosition"  |   
| Impose speed             | @ref chrono::ChLinkMotorRotationSpeed "ChLinkMotorRotationSpeed"         | @ref chrono::ChLinkMotorLinearSpeed "ChLinkMotorLinearSpeed"         | @ref chrono::ChShaftsMotorSpeed "ChShaftsMotorSpeed"        |
| Apply load               | @ref chrono::ChLinkMotorRotationTorque "ChLinkMotorRotationTorque"       | @ref chrono::ChLinkMotorLinearForce "ChLinkMotorLinearForce"         | @ref chrono::ChShaftsMotorLoad "ChShaftsMotorLoad"      |
| Connect to 1D driveline  | @ref chrono::ChLinkMotorRotationDriveline "ChLinkMotorRotationDriveline" | @ref chrono::ChLinkMotorLinearDriveline "ChLinkMotorLinearDriveline" | -                                                           |


<div class="ce-warning"> 
The old chrono::ChLinkLockLinActuator classes are obsolete: users are encouraged to use the new ChLinkMotor classes described in this page.
</div> 


\tableofcontents


# 3D Rotational Motors   {#rotational_motors}

These motors connect two parts of class @ref chrono::ChBodyFrame "ChBodyFrame", i.e objects that have translation+rotation in space, for example @ref chrono::ChBody "ChBody" or @ref chrono::fea::ChNodeFEAxyzrot "ChNodeFEAxyzrot".

All rotational motors are inherited from @ref chrono::ChLinkMotorRotation "ChLinkMotorRotation" and consider a rotation around the Z axes of the constrained frames expressed in radians as base unit.

![](http://www.projectchrono.org/assets/manual/pic_ChLinkMotorRotation.png)

Rotational motors allow multiple turns and offers either the wrapped @ref chrono::ChLinkMotorRotation::GetMotorAngleWrapped() "GetMotorAngleWrapped()" or unwrapped @ref chrono::ChLinkMotorRotation::GetMotorAngle() "GetMotorAngle()" motor angle.

Relative displacement|speed|acceleration are retrieved through @ref chrono::ChLinkMotorRotation::GetMotorAngle() "GetMotorAngle()" | @ref chrono::ChLinkMotorRotation::GetMotorAngleDt() "GetMotorAngleDt()" | @ref chrono::ChLinkMotorRotation::GetMotorAngleDt2() "GetMotorAngleDt2()"

By default, all rotational motors also embed a revolute constraint along the Z axis unless otherwise specified by means of the @ref chrono::ChLinkMotorRotation::SetSpindleConstraint() "SetSpindleConstraint()" method, that can accept the following options:

- FREE : enforces no constraint on spindle direction/alignment
- REVOLUTE: enforces **X**,**Y**,**Z**, **RX**, **RY** constraints (default)
- CYLINDRICAL: enforces **X**,**Y**, **RX**, **RY** constraints 
- OLDHAM: enforces **RX**, **RY** constraints 
  
In general, the process of adding a motor involves the following steps:

- Create the motor from the desired ChLinkMotorXxxyyy class
- Use one of the ```Initialize()``` methods available on the class
- Add the motor to a ChSystem
- Associate a @ref chrono::ChFunction "ChFunction" object that describes the motion function;
  methods names differ depending on the specific derived class.

Example:

~~~{.cpp}
// Create the motor
auto rotmotor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

// Connect the rotor and the stator and add the motor to the system:
rotmotor->Initialize(rotor,                // body A
                     stator,               // body B
                     ChFramed(ChVector3d(1,0,0)) // motor frame, in abs. coords
                     );
                      
// Add the motor to the system
mphysicalSystem.Add(rotmotor);

// Create a ChFunction to be used for the motor: for example a constant 
// angular speed, in [rad/s], ex. 1 PI/s =180°/s
auto mwspeed = chrono_types::make_shared<ChFunctionConst>(CH_PI); 

// Let the motor use our motion function:
rotmotor->SetSpeedFunction(mwspeed);

~~~



# 3D Linear Motors {#linear_motors}

These motors connect two parts of class @ref chrono::ChBodyFrame "ChBodyFrame", i.e objects that have translation+rotation in space, for example @ref chrono::ChBody "ChBody" or @ref chrono::fea::ChNodeFEAxyzrot "ChNodeFEAxyzrot".

All linear motors are inherited from @ref chrono::ChLinkMotorLinear "ChLinkMotorLinear" and the assume the Z axis being the direction of the allowed direction.
 
![](http://www.projectchrono.org/assets/manual/pic_ChLinkMotorLinear.png)

By default, all linear motors also provide a prismatic constraint for the other relative degrees of freedom (translation about **Y**,**Z** and rotation about **RX**, **RY, **RZ**, except rotation translation Z that is the one controlled by the motor) so you do not need to create additional joints, like ChLinkLockPrismatic for example, to keep the two parts together. Anyway, if you prefer, this behavior can be changed by using the ChLinkMotorLinear::SetGuideConstraint() function, that can accept the following options:

Relative displacement|speed|acceleration are retrieved through @ref chrono::ChLinkMotorLinear::GetMotorPos() "GetMotorPos()" | @ref chrono::ChLinkMotorLinear::GetMotorPosDt() "GetMotorPosDt()" | @ref chrono::ChLinkMotorLinear::GetMotorPosDt2() "GetMotorPosDt2()"

By default, all linear motors also embed a prismatic constraint along the Z axis, unless otherwise specified by means of the @ref chrono::ChLinkMotorLinear::SetGuideConstraint() "SetGuideConstraint()" method, that can accept the following options:

- FREE : enforces no constraint on roller direction/alignment
- PRISMATIC: enforces **X**, **Y**, **RX**, **RY**, **RZ** constraints (default)
- SPHERICAL: enforces **X** and **Y**  constraints 

The initialization is similar to the `ChLinkMotorRotation` case.


## 3D Driveline Motors  {#driveline_motors}

The @ref chrono::ChLinkMotorLinearDriveline "ChLinkMotorLinearDriveline" and @ref chrono::ChLinkMotorRotationDriveline "ChLinkMotorRotationDriveline" allow to impose a relative motion (translational and rotational respectively) between two 3D bodies by coupling it with the translation/rotation of a @ref chrono::ChShaft "ChShaft" (that, we may remember, could act as either a 1D translational or rotational element). The constrained direction is always along Z.

Because of this coupling any motion imposed on the shaft leads to a consequent motion on the constrained frames (linear or rotational, depending on the motor type), but especially any reaction force/torque felt by the constrained bodies is reflected back to the shaft.

This means that, contrary to the other @ref chrono::ChLinkMotor "ChLinkMotor"s that were fed directly by a "signal" provided through a @ref chrono::ChFunction "ChFunction", this class preserve the balance of power and describes a full coupling at both displacement and force level.

Particular care should be put in setting proper values to the inertia of the @ref chrono::ChShaft "ChShaft"s objects to avoid instabilities, especially with low inertias and high speeds.

## 1D Motors  {#shaft_motors}

Motors of the @ref chrono::ChShaftsMotor "ChShaftsMotor" kind are operating on pair of objects of the 1D space.

The usage is similar to those of the @ref chrono::ChLinkMotor "ChLinkMotor" types with the difference that constrained objects are of the @ref chrono::ChShaft "ChShaft" type.


# Examples

See also:

- [demo_MBS_motors](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_motors.cpp)


