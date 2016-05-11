Creating A Powertrain In Chrono (demo_powertrain.cpp) {#tutorial_demo_powertrain}
==========================

Tutorial which teaches the basic approach to build 
systems that embed powertrains, made with 1-degree-of-freedom 
items (rotating shafts).

- connect 1D shafts with transmission ratios
- connect 1D shafts with 1D clutches, brakes, etc.
- connect a 1D shaft to a 3D body.
- No GUI: only text output.



\dontinclude demo_powertrain.cpp

## Example 1: create a simple power train with ChShaft objects

We will model a very basic powertrain with two shafts `A` and `B`,
connected by a reducer `[ t ]` with transmision ratio 't'. Shafts are
free to rotate, shaft A has an applied torque `Ta`, so A and B will
constantly accelerate. Each shaft must have some inertia, it's like a
flywheel, marked as `||` in the following scheme:
~~~{.c}
        A           B
   Ta  ||---[ t ]---||
   
~~~
Create a 1-degree-of-freedom '1D' mechanical object, that
is a ChShaft (an item that can oly rotate, with one inertia value
and maybe one applied torque). The ChShaft objects do not have
any meaning in 3d: they are just 'building blocks' for making
power trains as in imput-output black box schemes.
		
\skip my_shaftA
\until my_system.Add(my_shaftA);

Create another shaft. Note that we use shared pointers for ChShaft
objects, as we did for ChBody objects. Also, note that we must add them
to the ChSystem.

\skip my_shaftB
\until my_system.Add(my_shaftB);

Create a ChShaftsGear, that represents a simplified model
of a reducer, with transmission ratio t, between two ChShaft objects.
(Note that you could also build a 3D powertrain by creating full rigid bodies
of ChBody type and join them using ChLinkLockRevolute, ChLinkGear 3D constraints,
but this would introduce many unnecessary degrees of freedom/constraints
whereas the 1D items of ChShaft type, in this example, make things much simplier).

\skip my_shaft_gearAB
\until my_system.Add(my_shaft_gearAB);


## Example 2: a clutch between two shafts

We will model a very basic powertrain with two shafts A and B,
connected by a clutch `[ c ]`. Shafts (see flywheels `||` in scheme)
starts with nonzero speed, and are free to rotate independently
until the clutch is activated: since activation, they will decelerate
until they have the same speed.
~~~{.c}
       A           B
  Ta  ||---[ c ]---||

~~~

Create a ChShaft that starts with nonzero angular velocity:

\skip auto my_shaftA = std::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);

Create another ChShaft, with opposite initial angular velocity:

\skip my_shaftB
\until my_system.Add(my_shaftB);

Create a ChShaftsClutch, that represents a simplified model
of a clutch between two ChShaft objects (something that limits
the max transmitted torque, up to slippage).

\skip my_shaft_clutchAB
\until my_system.Add(my_shaft_clutchAB);

Let's begin the simulation with the clutch disengaged:

\skipline SetModulation

## Example 3: an epicycloidal reducer

We will model an epicycloidal reducer using the ChShaftsPlanetary
constraint.

The ChShaftsPlanetary makes a kinematic constraint between three
shafts: so one of them will be _fixed_ and will represent the truss
of the reducer -in epicycloidaal reducer, this is the role of the
large gear with inner teeth- and the two remaining shafts are the
input and output shafts (in other cases, such as the differential
planetary gear of the cars, all three shafts are free).

Also, a brake is applied for the output shaft: the ChShaftsClutch
will be used to this end, it's enough that one of the two shafts is fixed.

In the following scheme, the brake is `[ b ]`, the planetary (the
reducer) is `[ p ]`, the shafts are `A,B,C,D`, applied torque is `Ta`, inertias
of free shafts are shown as flywheels `||` , and fixed shafts are marked with `*` .
~~~{.c}

       A           B            D
  Ta  ||---[ p ]---||---[ b ]---*
           [   ]---*
                   C
				   
~~~

Create shaft A, with applied torque:

\skip auto my_shaftA = std::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);

Create shaft B

\skip my_shaftB
\until my_system.Add(my_shaftB);

Create shaft C, that will be fixed 
(to be used as truss of epicycloidal reducer)
 
\skip my_shaftC
\until my_system.Add(my_shaftC);

Create a ChShaftsPlanetary, that represents a simplified model
of a planetary gear between THREE ChShaft objects (ex.: a car differential)
An epicycloidal reducer is a special type of planetary gear.

\skip my_shaft_planetaryBAC
\until my_shaftC);

We can set the ratios of the planetary using a simplified formula, for the
so called _Willis_ case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
and leave free the truss C (the outer gear with inner teeth in our reducer); which is
the transmission ratio \f$ t_0 \f$ that we get? It is simply \f$ t_0=-Z_a/Z_c \f$, with \f$ Z_i \f$ = num of teeth of gears.
So just use the following to set all the three ratios automatically:

\skip t0
\until my_system.Add(my_shaft_planetaryBAC);

Now, let's make a shaft D, that is fixed, and used for the right side
of a clutch (so the clutch will act as a brake).

\skip my_shaftD
\until my_system.Add(my_shaftD);

Make the brake. It is, in fact a clutch between shafts B and D, where
D is fixed as a truss, so the clutch will operate as a brake.

\skip my_shaft_clutchBD
\until my_system.Add(my_shaft_clutchBD);


## Example 4: constraint between a ChBody and a ChShaft

Suppose you want to create a 3D model, for instance a slider-crank,
built with multiple ChBody objects; moreover you want to create a
powertrain, for instance a motor, a clutch, etc, for the rotation of
the crank. How to connect the _1D items_ of ChShaft class to the 
_3D items_ of ChBody class? 

The solution is to use the ChShaftsBody constraint,
shown as `[ bs ]` in the following scheme, where the 3D body is shown as `<>`.
In this example we also add a 'torsional spring damper' `C`, shown as `[ t ]`
that connects shafts `A` and `C` (C is shown as `*` because fixed).
~~~{.c}

        B             A           C
  Ta   <>---[ bs ]---||---[ t ]---*
	
	
~~~

Create 'A', a 1D shaft:

\skip auto my_shaftA = std::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);

Create 'C', a 1D shaft, fixed:

\skip my_shaftC
\until my_system.Add(my_shaftC);

Create 'B', a 3D rigid body:

\skip my_bodyB
\until my_system.Add(my_bodyB);

Make the torsional spring-damper between shafts A and C:

\skip my_shaft_torsionAC
\until my_system.Add(my_shaft_torsionAC);

Make the shaft 'A' connected to the rotation of the 3D body 'B'.
We must specify the direction (in body coordinates) along which the
shaft will affect the body:

\skip my_shaftbody_connection
\until my_system.Add(my_shaftbody_connection);
 

## Example 5: torque converter and thermal engine

In this example we use a torque converter.

The torque converter is represented by a ChShaftsTorqueConverter
object, that connects three 1D items of ChShaft class:
- the input shaft A, ie. the impeller connected to the engine
- the output shaft B, i.e. the turbine connected to the gears and wheels
- the stator C, that does not rotate and transmits reaction to the truss.
In the following scheme, the torque converter is represented as `[ tc ]`,
and we also add a thermal engine, shown with `[ e ]`, and a breaking torque `Tb`
(C is shown as `*` because fixed).

Create 'A', a 1D shaft:

\skip auto my_shaftA = std::make_shared<ChShaft>();
\until my_system.Add(my_shaftA);
 
Create 'B', a 1D shaft:

\skip my_shaftB
\until my_system.Add(my_shaftB);

Create 'C', a 1D shaft ,fixed:

\skip my_shaftC
\until my_system.Add(my_shaftC);

Create 'D', a 1D shaft, fixed:

\skip my_shaftD
\until my_system.Add(my_shaftD);

Make the torque converter and connect the shafts:
A (input),B (output), C(truss stator)
 
\skip my_torqueconverter
\until my_torqueconverter->SetCurveTorqueRatio(mT);

Make the thermal engine, acting on shaft A, the input to
the torque converter. Note that the thermal engine also
requires another shaft D, that is used to transmit the
reaction torque back to a truss (the motor block).

**Option A**: use a ChShaftsMotor, in the MOT_MODE_TORQUE mode.
It works, but most often this is more useful when in MOT_MODE_SPEED.

\skip auto my_motor = std::make_shared<ChShaftsMotor>();
\until my_system.Add(my_motor);

**Option B**: use a ChShaftsTorque, it just applies a torque
to my_shaftA (say, the crankshaft) and the negative torque
to my_shaftD (say, the motor block).
It is a quick approach. But you should take care of changing
the torque at each timestep if you want to simulate a torque curve...

\skip auto my_motor = std::make_shared<ChShaftsTorque>();
\until my_system.Add(my_motor);

**Option C**: a more powerful approach where you can
define a torque curve and a throttle value, using the
ChShaftsThermalEngine.

\skip auto my_motor = std::make_shared<ChShaftsThermalEngine>();
\until my_motor->SetTorqueCurve(mTw);


## The complete listing:

\include demo_powertrain.cpp
