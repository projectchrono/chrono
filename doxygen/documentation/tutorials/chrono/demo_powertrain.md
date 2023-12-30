Creating A Powertrain In Chrono (demo_CH_powertrain.cpp) {#tutorial_demo_powertrain}
==========================

Tutorial covers basic approaches to build 
systems that embed powertrains made with 1-degree-of-freedom 
items (rotating shafts):

- connect 1D shafts with transmission ratios
- connect 1D shafts with 1D clutches, brakes, etc.
- connect a 1D shaft to a 3D body.
- No GUI: only text output.



\dontinclude demo_CH_powertrain.cpp

## Example 1: create a simple powertrain with ChShaft objects

This models a very basic powertrain with two shafts `A` and `B`,
connected by a reducer `[ t ]` with transmission ratio `t`. The shafts are
free to rotate. Shaft A has an applied torque `Ta`, so A and B will
constantly accelerate. Each shaft must have some inertia and each one is
marked as `||` in the following scheme:
~~~{.c}
        A           B
   Ta  ||---[ t ]---||
   
~~~
First, create a ChShaft, a 1-degree-of-freedom '1D' mechanical object that
can only rotate. It has one inertia value
and maybe one applied torque. The ChShaft objects do not have
any meaning in 3D, they are just 'building blocks'.
		
\skip my_shaftA
\until my_system.Add(my_shaftA);

Next, create another shaft. Note that this falls back on the use of shared pointers for ChShaft
objects. Also note that the elements must be added to the ChSystem object that manages this simulation.

\skip my_shaftB
\until my_system.Add(my_shaftB);

Finally, create a ChShaftsGear that represents a simplified model
of a reducer with transmission ratio `t` between two ChShaft objects.

 <div class="ce-info">
Note that a full blown 3D powertrain could be created using a collection of rigid bodies
of ChBody type that would be connected via ChLinkLockRevolute and ChLinkGear 3D constraints. However,
 this would introduce additional complexity to the model which is not always warranted. The 1D items 
 of ChShaft type keep the model much simpler.
 </div> 

 
\skip my_shaft_gearAB
\until my_system.Add(my_shaft_gearAB);


## Example 2: a clutch between two shafts

This models a very basic powertrain with two shafts A and B
connected by a clutch `[ c ]`. Each shaft (see flywheels `||` in schematic below)
starts with a nonzero speed and is free to rotate independently
until the clutch is activated. At that point, they will decelerate
until they have the same speed.
~~~{.c}
       A           B
  Ta  ||---[ c ]---||

~~~

First, create a ChShaft that starts with nonzero angular velocity:

\skip auto my_shaftA = chrono_types::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);

Next, create another ChShaft with opposite initial angular velocity:

\skip my_shaftB
\until my_system.Add(my_shaftB);

Now, create a ChShaftsClutch that represents a simplified model
of a clutch between two ChShaft objects (something that limits
the max transmitted torque, up to slippage).

\skip my_shaft_clutchAB
\until my_system.Add(my_shaft_clutchAB);

Below, the simulation is started with the clutch disengaged:

\skipline SetModulation

## Example 3: an epicycloidal reducer

An epicycloidal reducer is modeled below using the ChShaftsPlanetary
constraint.

The ChShaftsPlanetary sets up a kinematic constraint between three
shafts. One of them will be _fixed_ and represents the truss
of the reducer - in epicycloidaal reducer, this is the role of the
large gear with inner teeth. The two remaining shafts are the
input and output shafts. In other cases, such as the differential
planetary gear of cars, all three shafts are free. A brake is applied on the output shaft - it is
enough that one of these last two shafts is fixed.

In the following schematic, the brake is `[ b ]`, the planetary (the
reducer) is `[ p ]`, the shafts are `A,B,C,D`, the applied torque is `Ta`, inertias
of free shafts are shown as flywheels `||` , and fixed shafts are marked with `*` .
~~~{.c}

       A           B            D
  Ta  ||---[ p ]---||---[ b ]---*
           [   ]---*
                   C
				   
~~~

First, create shaft A with an applied torque:

\skip auto my_shaftA = chrono_types::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);

Next, create shaft B:

\skip my_shaftB
\until my_system.Add(my_shaftB);

Create shaft C, which will be fixed (used as truss of epicycloidal reducer):
 
\skip my_shaftC
\until my_system.Add(my_shaftC);

Create a ChShaftsPlanetary that represents a simplified model
of a planetary gear between _three_ ChShaft objects (e.g., a car differential).
An epicycloidal reducer is a special type of planetary gear.

\skip my_shaft_planetaryBAC
\until my_shaftC);

The ratios of the planetary can be set using a simplified formula for the
so called _Willis_ case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
and leave the truss C free (the outer gear with inner teeth in our reducer); which is
the transmission ratio \f$ t_0 \f$ that we get. It is simply \f$ t_0=-Z_a/Z_c \f$, with \f$ Z_i \f$ = num of teeth of gears.
Use the following to set all the three ratios:

\skip t0
\until my_system.Add(my_shaft_planetaryBAC);

Now, let's make a shaft D that is fixed and thus used for the right side
of a clutch (so the clutch will act as a brake).

\skip my_shaftD
\until my_system.Add(my_shaftD);

Next, make the brake. It is, in fact, a clutch between shafts B and D, where
D is fixed as a truss so that the clutch will operate as a brake.

\skip my_shaft_clutchBD
\until my_system.Add(my_shaft_clutchBD);


## Example 4: constraint between a ChBody and a ChShaft

Suppose we want to create a 3D model, for instance a slider-crank,
built with multiple ChBody objects. Moreover, we want to create a
powertrain, for instance a motor, a clutch, etc., for the rotation of
the crank. Connecting _1D items_ of ChShaft class with the 
_3D items_ of ChBody class calls for the use of a ChShaftsBody constraint,
shown as `[ bs ]` in the schematic in which the 3D body is shown as `<>`.
This example also considers a 'torsional spring damper' `C`, shown as `[ t ]`,
that connects shafts `A` and `C` (C is shown as `*` because fixed).
~~~{.c}

        B             A           C
  Ta   <>---[ bs ]---||---[ t ]---*
	
	
~~~

First, create 'A', a 1D shaft:

\skip auto my_shaftA = chrono_types::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);

Next, create 'C', a 1D shaft, fixed:

\skip my_shaftC
\until my_system.Add(my_shaftC);

Create 'B', a 3D rigid body:

\skip my_bodyB
\until my_system.Add(my_bodyB);

Define the torsional spring-damper between shafts A and C:

\skip my_shaft_torsionAC
\until my_system.Add(my_shaft_torsionAC);

Define the shaft 'A' connected to the rotation of the 3D body 'B'.
Finally, specify the direction (in body coordinates) along which the
shaft will affect the body:

\skip my_shaftbody_connection
\until my_system.Add(my_shaftbody_connection);
 

## Example 5: torque converter and thermal engine

This example highlights the use of a torque converter. 
The torque converter is represented by a ChShaftsTorqueConverter
object that connects three 1D items of ChShaft class:
- the input shaft A, i.e., the impeller connected to the engine
- the output shaft B, i.e., the turbine connected to the gears and wheels
- the stator C, which does not rotate and transmits reaction to the truss
In the following schematic, the torque converter is represented as `[ tc ]`,
the thermal engine is marked with `[ e ]`, and the breaking torque is `Tb`
(C shown as `*` since fixed).

First, create 'A', a 1D shaft:

\skip auto my_shaftA = chrono_types::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);
 
Next, create 'B', a 1D shaft:

\skip my_shaftB
\until my_system.Add(my_shaftB);

Create 'C', a 1D shaft, fixed:

\skip my_shaftC
\until my_system.Add(my_shaftC);

Create 'D', a 1D shaft, fixed:

\skip my_shaftD
\until my_system.Add(my_shaftD);

Define the torque converter and connect the shafts:
A (input), B (output), C(truss stator)
 
\skip my_torqueconverter
\until my_torqueconverter->SetCurveTorqueRatio(mT);

Define the thermal engine acting on shaft A, the input to
the torque converter. Note that the thermal engine also
requires shaft D, which is used to transmit the
reaction torque back to a truss (the motor block).

**Option A**: use a ChShaftsMotor in MOT_MODE_TORQUE mode.
It works, but most often this is more useful when in MOT_MODE_SPEED.

\skip auto my_motor = chrono_types::make_shared<ChShaftsMotor>();
\until my_system.Add(my_motor);

**Option B**: use a ChShaftsTorque - it simply applies a torque
to my_shaftA (say, the crankshaft), and the negative torque
to my_shaftD (say, the motor block).
It is a quick approach. Note that 
the torque should be changed at each timestep if a torque curve is to be emulated.

\skip auto my_motor = chrono_types::make_shared<ChShaftsTorque>();
\until my_system.Add(my_motor);

**Option C**: a more versatile approach where one can
define a torque curve and a throttle value via the
ChShaftsThermalEngine.

\skip auto my_motor = chrono_types::make_shared<ChShaftsThermalEngine>();
\until my_motor->SetTorqueCurve(mTw);


## The complete listing:

\include demo_CH_powertrain.cpp
