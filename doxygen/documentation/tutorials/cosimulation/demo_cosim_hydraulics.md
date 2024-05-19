Cosimulation with Simulink (demo_COSIM_hydraulics.cpp)  {#tutorial_demo_cosim_hydraulics}
==========================

This tutorial explains how to use co-simulation to simulate a hydraulic system that moves a simple mechanism. 
The hydraulic system is managed by Simulink, while the mechanism with moving parts and joints is simulated by 
Chrono.

The approach described can serve as a template for complex scenarios in which mechanisms with multiple hydraulic cylinders actuate moving parts. In general, Simulink features are used for hydraulic subsystem simulation and controls, whereas Chrono can be used for the remaining components of the mechanical system:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_01.png)


# Prerequisites

The prerequisites for this tutorial are:

* The Chrono core module should be properly installed/working.
* [Simulink](http://www.mathworks.com/products/simulink) must be installed.
* The Matlab [Instrument Control Toolbox](http://www.mathworks.com/products/instrument) should be installed.
* The Matlab [SimHydraulics Toolbox](http://www.mathworks.com/products/simhydraulics) should be installed.

<div class="ce-info">
SimHydraulics is an optional Matlab toolbox for Simulink. This specific Chrono demo requires it yet other examples of cosimulation might not necessarily need it.
</div>


# Background 

Two way co-simulation draws on two simulation tools which simultaneously simulate (advance in time)  the two subsystems in which the original system is partitioned.
Once in a while the two solvers (simulation tools) synchronize to exchange data after which they proceed independently until the next synchronization time. 
This periodic data synchronization is necessary because the subsystems are coupled. For tightly coupled  subsystems the synchronization happens very often.

There are many approaches to co-simulation. The method described in this tutorial is quite simple - it is based on the following pattern of interaction between Chrono and Simulink:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_02.png)

In the schematic above, suppose that the system is defined by 
variables \f$ \mathbf{X} = \{ \mathbf{X}_{CE}, \mathbf{X}_{S}\} \f$, 
which are splitted so that Chrono deals with 
\f$ \mathbf{X}_{CE} \f$ while Simulink deals with 
\f$ \mathbf{X}_{S} \f$. 

The sequence of operations is:
- A) the Chrono simulator advances by a time step \f$ dt \f$, 
  up to the synchronization point;
- B) the \f$ \mathbf{X}_{CE} \f$ variables are sent from Chrono 
     to Simulink using a TCP/IP socket;
- C) the \f$ \mathbf{X}_{S} \f$ variables are sent from Simulink to Chrono 
     using a TCP/IP socket;
- D) the Simulink simulation advances up to the synchronization point. 
Note that Simulink could use variable-step integration, hence it 
	 could take more than one step to reach the synchronization point, see figure.

This pattern is repeated through to the end of the simulation.

In this implementation, the value of \f$ \mathbf{X}_{CE} \f$ that 
is received by Simulink is interpolated linearly during the timestep(s) 
advancement that reaches the sync time; on the other hand, the \f$ \mathbf{X}_{S} \f$
values received by Chrono are kept constant as in a Zero Order Hold (ZOH).

In this type of cosimulation the 'slow' integrator, that is Chrono, 
precedes the high-frequency integrator, that is Simulink. However, the opposite 
could be done as well. Other more sophisticated interpolation-extrapolation 
concepts could be devised - for example using extrapolation instead of ZOH 
or using higher-order interpolation for \f$ \mathbf{X}_{CE} \f$ received in Simulink. 

When Chrono precedes Simulink and using extrapolated 
\f$ \mathbf{X}_{S} \f$ values is like having a 
latency of \f$ dt \f$ in the cause-effect relationship between Chrono and Simulink. 
When the coupling between the two subsystems is tight, too large a \f$ dt \f$
 can cause _numerical instability_ problems. Unfortunately, keeping \f$ dt \f$ very small 
 to address stability issues will likely impact negatively the simulation times.


# The system

The following mechanism is considered in this tutorial: a lever 
with a revolute joint in A and a linear actuator L between points B and D:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_03.png)

Although Chrono already has methods to define linear actuators, 
Simulink and its SimHydraulics toolbox are used to produce an 
advanced model of a hydraulic actuator with valves, piping, pumps, 
feedback control, etc. The yellow actuator will be simulated in Simulink, 
the rest in Chrono.


# The Simulink model

First, open Simulink and create a model of a hydraulic actuator 
with a pump and a valve. To keep it simple, just load the 
*Power assisted steering mechanism* demo of SimHydraulics 
(sh_hydraulic_power_assisted_steering.mdl). It includes a 
ready-to use system where a piston is moved 
back and forth depending on the feedback control of the rotation 
of a steering wheel:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_04.png)

Note that the original example used a mass-spring-damper 
system modeled with SimScape Simulink blocks; this can be seen
by double clicking on the _Load_ block to expand it:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_05.png)


Replacing the _Load_ block with a Chrono mechanical load calls for the following steps:

- Delete the _Load_ block.

- Open data/cosimulation/CEcosimulation.mdl, 
  copy the _CoSimulate_ block and paste it into the model, near 
  the piston.

* Select _CoSimulate_ and use menu Edit/Link Options../Break link 
  so that you can modify it

* Create the following connections between the piston and the _CoSimulate_ 
  block:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_06.png)


The _CoSimulate_ block is an interface that 
sends its input vector of variables to Chrono and gets 
back an output vector at fixed time intervals. Specifically,
- Simulink sends the force of the piston to Chrono. 
  Chrono will apply that force to the mechanical system
  between the 'B' and 'D' hinges, so it will compute speed and displacement.
- Vice-versa: Chrono sends speed & displacement of the piston 
  to Simulink. Simulink will enforce that speed and displacement to the piston
  so that it will compute the force.

This explains the need for those two yellow blocks: one is 
the SimScape Simulink block used to get a reaction force 
(in this example in the rod of the piston) while the other is used to 
impose a motion (in this example to the rod of the piston).
The PS S and S PS grey blocks are used to convert SimScape 
and SimHydraulics signals from/to Simulink signals.

<div class="ce-info">
Note that there could be other ways to do co-simulation for 
this example. For instance, one could impose a force in Simulink and send displacement information
to Chrono, then Chrono imposes displacement with 
a rheonomic linear actuator and gets the reaction force to send to Simulink. 
</div>

Double click on the _CoSimulate_ block and set its parameters:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_07.png)

Semantics:

- _Communication step size_: controls how frequently Chrono and 
  Simulink exchange information. This example needs a very small step size 
  because this is a stiff problem (as is often the case with hydraulic problems) 
  so larger steps would cause numerical stability issues.

- _N. of scalars received from Chrono_ must match the number of variables 
  sent from the Chrono-side using C++. In this example what gets sent is 1) velocity 
  and 2) displacement of the rod.

- _TCP host name_: leave 'localhost' if you run both simulators 
  on the same computer, otherwise if you have Chrono on a different 
  machine, you provide the IP address here.

- _TCP port_: any non-firewalled, non-used port can work here. 
  For instance, use 50009.

- _TCP timeout_: if Chrono does not respond within this period, 
  the simulation is stopped.

Next, save the entire model with a proper name. This example already provides a ready-to-use model: 
data/cosimulation/test_cosim_hydraulics.mdl


# The Chrono model

You can find a ready-to-use example in 
src/demos/cosimulation/demo_COSIM_hydraulics.cpp. The steps implemented in this program are as follows:

- Use the 'try...catch' statements around socket communication operations because errors in sockets might throw exceptions.

- Create a Chsystem and populate it with a truss, a moving body, and a revolute joint.

- Now create create a 'dead' linear actuator between two points using a ChLinkTSDA with zero stiffness and damping. This will be used to apply the force between the two bodies as a cylinder with spherical ball ends.

- Create a spring-damper to have some load when moving, and configure the system's solver precision.

- Add a socket framework object (a unique copy to be used through all the program) and a cosimulation interface.


- Prepare the two column vectors of data that will be swapped back and forth between Chrono and Simulink:
  - receive one variable from Simulink (the hydraulic cylinder force);
  - send two variables to Simulink (the hydraulic cylinder velocity and displacement).

- Wait for client (Simulink) to connect. Note that in this implementation Chrono is the server and Simulink the client.

- Upon establishing a connection, the simulation is ready to begin. Note that 'dt' must be the same with the value entered in the CEcosimulation block.

- Finally, provision for exception catching in running into any trouble with the socket connection.


# Run the cosimulation

All the tools are ready for the cosimulation at this point. The next steps:

- Compile the Chrono program;

- Run the Chrono program; it will enter a wait state because it is waiting Simulink to connect;

- Open the ''Input/Output'' orange scope block in Simulink, simply to plot some results.

- Run the Simulink model by pressing the '>' button in the Simulink interface;

The two codes will run in parallel and will exchange data periodically up to the end of the simulation or up to when one presses 'stop' or aborts the Chrono program.


![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_08.png)


# Notes

<div class="ce-info">
Co-simulation can be slow if the communication timestep is small. In some cases a small timestep is unavoidable - in the example discussed here this is necessary because the system is stiff and larger steps would cause a numerical instability that will see the variables oscillating wildly.
</div>

<div class="ce-info">
One can configure the ''CoSimulate'' block with more than one input and two outputs. For instance, if you are going to simulate six PID controllers for a 6 DOF robot using Simulink, you might need to receive six rotations from Chrono as output and send six control signals (ex. torques) to Chrono, as input of CoSimulate.
</div>

<div class="ce-warning">
Only a single ''CoSimulate'' block can be used at a time. If you need to send/receive a larger number of variables, simply configure the block to have vectors with more elements as input and outputs.
</div>

<div class="ce-info">
The input and output signals of the 'CoSimulate' block accepts vectors. To build a vector from single signals use the 'Mux' block in Simulink. To split a vector in single signals use a 'De-Mux' block as shown in the example above.
</div>

<div class="ce-info">
If you have multiple options for choosing where to split a system in two subsystems for cosimulation, keep in mind this rule of thumb: split in a place where the connecting variables (forces, displacements or such) experience 'low frequencies' rather than high frequencies because the higher the frequencies that flow in the interface, the smaller the communication step must be.
</div>

<div class="ce-info">
Changing the Simulink integrator from variable time step to fixed time step might yield a warning or error about blocks having different sample rates. Use the menu Simulation/Configuration parameters..., go to the Solver tab, then set
'Tasking mode for periodic sample times: SingleTasking' (default is Auto).
</div>


# The entire code

\include demo_COSIM_hydraulics.cpp

