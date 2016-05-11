Cosimulation with Simulink (demo_cosim_hydraulics.cpp)  {#tutorial_demo_cosim_hydraulics}
==========================

Thanks to the 
[COSIMULATION module](@ref module_cosimulation_installation),
this tutorial will show you how to use co-simulation to 
simulate a hydraulic system that moves a simple mechanism, 
where the hydraulic system is managed by Simulink, 
and the mechanism with moving parts and joints is simulated by 
Chrono::Engine.

This can be used as a template for complex scenarios 
where there are mechanisms with multiple hydraulic 
cylinders that actuate moving parts: in general the 
powerful Simulink features for hydraulic system simulation 
and control systems can be used for the hydraulic subsystems, 
whereas Chrono::Engine can be used for the machinery:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_01.png)


# Prerequisites

The prerequisites for this tutorial are:

* you must use the [COSIMULATION module](@ref module_cosimulation_installation).
* you must have [Simulink](http://www.mathworks.com/products/simulink)installed.
* you must have [Instrument Control Toolbox](http://www.mathworks.com/products/instrument) for Simulink installed.
* you must have [SimHydraulics Toolbox](http://www.mathworks.com/products/simhydraulics) for Simulink installed.

Note that the SimHydraulics Toolbox is an optional 
toolbox for Simulink, and it is used for this specific demo, 
however other examples of cosimulation might not necessarily need it.


# Background 

Co-simulation is a way to split a model on two different 
software tools, so that each tool runs in parallel with the other, 
and once in a while they synchronize data and proceed with the simulation. 
This periodic data synchronization is necessary because subsystems 
might be coupled in different ways and, most often, 
it must happen with  high frequency.

There are many approaches to co-simulation. 
The method that we present in this tutorial is quite simple, 
and it is based on the following pattern of interaction 
between Chrono::Engine and Simulink:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_02.png)

In the scheme above, suppose that the system is defined by 
variables \f$ \mathbf{X} = \{ \mathbf{X}_{CE}, \mathbf{X}_{S}\} \f$ 
that are splitted so that Chrono::Engine deals with 
\f$ \mathbf{X}_{CE} \f$ and Simulink deals with 
\f$ \mathbf{X}_{S} \f$. 
So, the sequence of operations is:

- A) the Chrono::Engine simulation advances by a time step \f$ dt \f$, 
  up to the synchronization point;
- B) the \f$ \mathbf{X}_{CE} \f$ variables are sent from Chrono::Engine 
     to Simulink using a TCP/IP socket;
- C) the \f$ \mathbf{X}_{S} \f$ variables are sent from Simulink to Chrono::Engine 
     using a TCP/IP socket;
- D) the Simulink simulation advances up to the synchronization point 
     (note that Simulink here could use variable-step integrators, hence it 
	 could take more than one steps to reach the synchronization point, as in figure).

This pattern is repeated up to the end of the simulation.

Note that in our implementation, the value of \f$ \mathbf{X}_{CE} \f$ that 
is received by Simulink is interpolated linearly during the timestep(s) 
advancement that reaches the sync time; on the other hand, the \f$ \mathbf{X}_{S} \f$
values received by Chrono::Engine are kept constant as in a Zero Order Hold (ZOH).

In this type of cosimulation the 'slow' integrator, that is Chrono::Engine, 
precedes the high-frequency integrator, that is Simulink; however viceversa 
could be done as well, and other more sophisticated interpolation-extrapolation 
concepts could be devised (for example using extrapolation instead of ZOH 
or using higher-order interpolation for \f$ \mathbf{X}_{CE} \f$ received in Simulink) 
but this is enough for our example.

The fact that Chrono::Engine precedes Simulink, using extrapolated 
\f$ \mathbf{X}_{S} \f$ values, has some consequences: it is like having a 
latency of \f$ dt \f$ in the cause-effect between Chrono::Engine and Simulink. 
That is, maybe that the coupling between the two subsystems is not satisfied exactly, 
and this can cause _numerical instability_ problems. 

This problem can be cured by keeping \f$ dt \f$ as small as possible, 
although this could negatively affect the performance.


# The system

Assume we want to simulate the following mechanism, that is a lever 
with a revolute joint in A and a linear actuator L between points B and D:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_03.png)

Although in Chrono::Engine we already have methods to define linear actuators, 
here we want to use Simulink and its SimHydraulics toolbox for an 
advanced model of an hydraulic actuator with valves, piping, pumps, 
feedback control, etc. So the yellow actuator will be simulated in Simulink, 
the rest in Chrono::Engine.


# The Simulink model...

Let's open Simulink and create a model of a hydraulic actuator 
with a pump and a valve. To keep it simple, just load the 
*Power assisted steering mechanism* demo of SimHydraulics 
(sh_hydraulic_power_assisted_steering.mdl); 
it includes a ready-to use system where a piston is moved 
back and forth depending on the feedback control of the rotation 
of a steering wheel:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_04.png)

Note that in the original example, they used a mass-spring-damper 
system modeled with SimScape Simulink blocks, you can see this 
by double clicking on the _Load_ block to expand it:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_05.png)


We want to replace that _Load_ block with our Chrono::Engine mechanical load.

This requires that you do this:

- Delete the _Load_ block.

- Open ChronoEngine/bin/data/cosimulation/CEcosimulation.mdl, 
  copy the _CoSimulate_ block and paste it into the model, near 
  the piston.

* Select _CoSimulate_ and use menu Edit/Link Options../Break link 
  so that you can modify it

* Create the following connections between the piston and _CoSimulate_ 
  block:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_06.png)


What is the meaning of this? 
Simply speaking, the _CoSimulate_ block is an interface that 
send its input vector of variables to Chrono::Engine, and gets 
back an output vector, at fixed time intervals.

In our example, here, the cencept is:
- Simulink send the force of the piston to Chrono::Engine. 
  Chrono::Engine will apply that force to the mechanical system, 
  between the 'B' and 'D' hinges, so it will compute speed & displacement.
- Viceversa: Chrono::Engine sends speed & displacement of the piston 
  to Simulink. Simulink will enforce that speed and displacement to the piston, 
  so that it will compute the force.

This explains why we need those two yellow blocks: one is 
the SimScape Simulink block used to get a reaction force 
(here, in the rod of the piston) and the other is used to 
impose a motion (here, to the rod of the piston).

The PS S and S PS grey blocks are used to convert SimScape 
and SimHydraulics signals from/to Simulink signals.

Note that there could be other ways to do co-simulation for 
this example. For instance, as a specular case to the discussed approach, 
one could impose force in the Simulink side, and send displacement 
to Chrono::Engine, then Chrono::Engine imposes displacement with 
a rheonomic linear actuator and gets the reaction force to send to Simulink. 

 
Now double click on the _CoSimulate_ block and set its parameters:

![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_07.png)

The meaning here is:

- _Communication step size_: it sets how frequently Chrono::Engine and 
  Simulink will talk. In this example, we need a very small step size 
  because this is a stiff problem (as many hydraulic problems in general) 
  so larger steps would cause convergence problems and instability.

- _N. of scalars received from C::E_ must match the number of variables 
  sent from the C::E side using C++. In our example, we'll send 1) velocity 
  and 2) displacement of the rod.

- _TCP host name_: you can leave 'localhost' if you run both simulators 
  on the same computer, otherwise if you have Chrono::Engine on a different 
  machine, you could put the IP address here.

- _TCP port_: any non-firewalled, non-used port can work here. 
  We use 50009, for example.

- _TCP timeout_: if Chrono::Engine does not respond within this period, 
  just stop the simulation.

Now you can save the entire model with a proper name. 
For this example, we already provide you a ready-to-use model: 
ChronoEngine/bin/data/cosimulation/test_cosim_hydraulics.mdl


# The Chrono::Engine model...

Now let's develop the Chrono::Engine C++ program that will interact 
with Simulink. You can find a ready-to-use example in 
ChronoEngine/source/demos/cosimulation/demo_cosim_hydraulics.cpp

- Use the try..catch statements around socket communication stuff 
  because errors in sockets might throw exceptions,

- create a Chsystem and populate it with a truss, a moving body, 
  a revolute joint:

~~~{.cpp}
	try{

		ChSystem my_system; 

		 // truss
		ChSharedBodyPtr  my_body_A(new ChBody);	
		my_body_A->SetBodyFixed(true);				
		my_system.AddBody(my_body_A);

		 // moving body
		ChSharedBodyPtr  my_body_B(new ChBody);		
		my_body_B->SetMass(114);
		my_body_B->SetInertiaXX(ChVector<>(50,50,50));
		my_body_B->SetPos(ChVector<>(1,1,0));
		my_system.AddBody(my_body_B);

    		 // revolute joint
		ChSharedPtr<ChLinkLockRevolute>  my_link_BA(new ChLinkLockRevolute);
		my_link_BA->Initialize(my_body_B, my_body_A, ChCoordsys<>(ChVector<>(0,1,0)));
		my_system.AddLink(my_link_BA);
~~~

Now create create a 'dead' linear actuator between two points 
using a @ref ChLinkSpring with zero stiffness and damping. 
This will be used to apply the force between the two bodies as 
a cylinder with spherical ball ends.

~~~{.cpp}
		ChSharedPtr<ChLinkSpring>  my_link_actuator(new ChLinkSpring);
		my_link_actuator->Initialize(my_body_B, my_body_A, false, ChVector<>(1,0, 0), ChVector<>(1,1,0));
		my_link_actuator->Set_SpringK(0);
		my_link_actuator->Set_SpringR(0);
		my_link_actuator->Set_SpringRestLenght(my_link_actuator->GetDist());
		my_system.AddLink(my_link_actuator);
~~~

Create also a spring-damper to have some load when moving, 
and configure the system's solver precision:

~~~{.cpp}
		ChSharedPtr<ChLinkSpring>  my_link_springdamper(new ChLinkSpring);
		my_link_springdamper->Initialize(my_body_B, my_body_A, false, ChVector<>(1,0, 0), ChVector<>(1,1,0));
		my_link_springdamper->Set_SpringK(4450);
		my_link_springdamper->Set_SpringR(284);
		my_link_springdamper->Set_SpringRestLenght(my_link_springdamper->GetDist());
		my_system.AddLink(my_link_springdamper);


		my_system.Set_G_acc(ChVector<>(0,0,0));
		my_system.SetIterLCPmaxItersSpeed(20);
		my_system.SetLcpSolverType(ChSystem::eCh_lcpSolver::LCP_ITERATIVE_BARZILAIBORWEIN);
~~~

Add a socket framework object (an unique copy to be used through 
all the program) and a cosimulation interface:

~~~{.cpp}

		 // socket framework object
		ChSocketFramework socket_tools;

		 // cosimulation interface:
		ChCosimulation cosimul_interface(socket_tools,
										 1,	// n.input values from Simulink
										 2);// n.output values to Simulink
~~~

Prepare the two column vectors of data that will be swapped 
back and forth between C::E and Simulink. In detail we will

- receive 1 variable from Simulink (the hydraulic cylinder force)
- send 2 variables to Simulink (the hydraulic cylinder velocity and displacement)

~~~{.cpp}

		ChMatrixDynamic<double> data_in(1,1);
		ChMatrixDynamic<double> data_out(2,1);
~~~

Wait client (Simulink) to connect...
(Note that in our implementation Chrono::Engine is the server, 
and Simulink the client).

~~~{.cpp}
	
		GetLog() << " *** Waiting Simulink to start... *** \n\n";
		
		int PORTNUM = 50009;

		cosimul_interface.WaitConnection(PORTNUM);
~~~

Ok, connected? Now we are ready to run the simulation.

Note! Here the 'dt' must be the same of the sampling 
period that is entered in the CEcosimulation block!

~~~{.cpp}
		double mytime = 0;
		double histime = 0;
		
		double dt = 0.001; // same as sample period
		
		while (true)
		{

			// A) -- ADVANCE THE C::E SIMULATION 
					
			
			if (dt>0)
				my_system.DoStepDynamics(dt);
			
			mytime += dt;
			
					
			// B) -- SYNCHRONIZATION 
			
				// B.1) - SEND data 

			// - Set the multibody variables into the vector that must 
			//   be sent to Simulink at the next timestep:
			//      * the velocity of the hydraulic actuator
			//      * the displacement of the hydraulic actuator

			data_out(0) = my_link_actuator->GetDist_dt();
			data_out(1) = my_link_actuator->GetDist() - my_link_actuator->Get_SpringRestLenght(); 

			cosimul_interface.SendData(mytime, &data_out);	
			  

				// B.2) - RECEIVE data

			cosimul_interface.ReceiveData(histime, &data_in);

			// - Update the multibody system with the received force  
			//   from Simulink, using the data_in vector, that contains:
			//      * the force of the hydraulic actuator

			my_link_actuator->Set_SpringF(data_in(0));


			GetLog() << "--- time: " << mytime << "\n";		

		}

	} 
~~~

Complete the code with the exception catching, in case of 
troubles with socket connections.

~~~{.cpp}

	catch(ChExceptionSocket exception)
	{
		GetLog() << " ERRROR with socket: \n" << exception.what() << "\n";
	}
~~~


# Run the cosimulation

Now, all the tools are ready for the cosimulation.

- Compile the Chrono::Engine program;

- Run the Chrono::Engine program; it will enter a wait state, 
  because it is waiting Simulink to connect;

- Open the ''Input/Output'' orange scope block in Simulink, 
  simply to plot some results.

- Run the Simulink model by pressing the '>' button in the Simulink interface;

- Now the two codes will run in parallel and will exchange data 
  periodically up to the end of the simulation (or up to when one presses 
  'stop' or aborts the Chrono::Engine program).


![](http://projectchrono.org/assets/manual/Tutorial_cosim_hydraulics_08.png)


# Notes

<div class="ce-info">
Co-simulation can be slow if the communication step size is small. However, in this example this is necessary because the system is stiff and larger steps would cause a divergent integration (variables will start oscillating wildly).
</div>

<div class="ce-info">
You can configure the '''CoSimulate''' block with more than one input and two outputs. For instance, if you are going to simulate six PID controllers for a 6 DOF robot using Simulink, you might need to receive six rotations from Chrono::Engine as output, and send six control signals (ex. torques) to Chrono::Engine, as input of CoSimulate.
</div>

<div class="ce-warning">
Only a single '''CoSimulate''' block can be used at a time! If you want to send/receive larger number of variables, just configure the block to have vectors with more elements as input and outputs.
</div>

<div class="ce-info">
The input and output signals of the '''CoSimulate''' block accepts vectors. To build a vector from single signals use the '''Mux''' block in Simulink. To split a vector in single signals use a '''De-Mux''' block (as in the example above).
</div>

<div class="ce-info">
If you have multiple otions for choosing where to split a system in two subsystems for cosimulation, keep in mind this rule of thumb: cut it in a place where the connecting variables (forces, displacements or such) experience '''low frequencies''' rather than high frequencies, because the higher the frequencies that flow in the interface, the smaller the communication step must be.
</div>

<div class="ce-info">
If you want to change the integrator of Simulink from variable time step to fixed time step, you might get some warning or error about some blocks having different sample rates. If so, just use menu Simulation/Configuration parameters..., go to Solver tab, then set
''Tasking mode for periodic sample times: SingleTasking'' (default is Auto).
</div>


# The entire listing

For completeness, here we report the entire .cpp source:

\include demo_cosim_hydraulics.cpp

