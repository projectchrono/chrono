
Simulation system      {#simulation_system}
=================

The simulation system contains all other objects being simulated: 
[bodies](@ref rigid_bodies), [links](@ref links), etc.
It plays a fundamental role in Chrono::Engine.


# ChSystem  {#manual_ChSystem}

The simulation system is an object of class ChSystem. 
See @ref chrono::ChSystem for API details.

The following picture shows an example that explains 
how mechanisms turn into a database of bodies and links 
in a ChSystem:

![](pic_database.png)

- A @ref chrono::ChSystem contains all items of the simulation: bodies, constraints, etc.

- Use the ```Add()```, ```Remove()``` functions to populate it

- Simulation settings are contained in ChSystem:
  - integrator type
  - tolerances
  - etc.

In most cases, you have to do the following:

- create a @ref chrono::ChSystem
- add [body](@ref rigid_bodies) objects into it, see @ref chrono::ChBody
- add [link](@ref links) objects into it, see @ref chrono::ChLink
- adjust some parameters for the time integration
- run the simulation loop

Example:

~~~{.cpp}
// 1- Create a ChronoENGINE physical system: all bodies and constraints
//    will be handled by this ChSystem object.
ChSystem my_system;
 
 
// 2- Create the rigid bodies of the slider-crank mechanical system
//   (a crank, a rod, a truss), maybe setting position/mass/inertias of
//   their center of mass (COG) etc.
	
// ..the truss
auto my_body_A = std::make_shared<ChBody>();
my_system.AddBody(my_body_A);
my_body_A->SetBodyFixed(true);  // truss does not move!

// ..the crank
auto my_body_B = std::make_shared<ChBody>();
my_system.AddBody(my_body_B);
my_body_B->SetPos(ChVector<>(1, 0, 0));  // position of COG of crank
my_body_B->SetMass(2);

// ..the rod
auto my_body_C = std::make_shared<ChBody>();
my_system.AddBody(my_body_C);
my_body_C->SetPos(ChVector<>(4, 0, 0));  // position of COG of rod
my_body_C->SetMass(3);


// 3- Create constraints: the mechanical joints 
//    between the rigid bodies.

// .. a revolute joint between crank and rod
auto my_link_BC = std::make_shared<ChLinkLockRevolute>();
my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2, 0, 0)));
my_system.AddLink(my_link_BC);

// .. a slider joint between rod and truss
auto my_link_CA = std::make_shared<ChLinkLockPointLine>();
my_link_CA->Initialize(my_body_C, my_body_A, ChCoordsys<>(ChVector<>(6, 0, 0)));
my_system.AddLink(my_link_CA);

// .. an engine between crank and truss
auto my_link_AB = std::make_shared<ChLinkEngine>();
my_link_AB->Initialize(my_body_A, my_body_B, ChCoordsys<>(ChVector<>(0, 0, 0)));
my_link_AB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(my_link_AB->Get_spe_funct()))
	mfun->Set_yconst(CH_C_PI);  // speed w=3.145 rad/sec
my_system.AddLink(my_link_AB);
	
	
// 4- Adjust settings of the integrator (optional):
my_system.SetIntegrationType(ChSystem::INT_IMPLICIT_EULER)
my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
my_system.SetIterLCPmaxItersSpeed(20);
my_system.SetIterLCPmaxItersStab(20);
my_system.SetMaxPenetrationRecoverySpeed(0.2);
my_system.SetMinBounceSpeed(0.1);
		
// 5- Run the simulation (basic example)

while( my_system.GetChTime() < 10 )
{ 		
	// Here Chrono::Engine time integration is performed:
	my_system.StepDynamics(0.02);

	// Draw items on screen (lines, circles, etc.) or dump data to disk 
	[..] 
}
~~~


The default simulation settings are good for real-time 
fast simulations, with low requirements in terms of precision, 
and with balanced values of sizes and masses. 

If you want to simulate more difficult scenarios, 
you might need to adjust the settings. In most cases there 
are three areas to adjust:

- the **time stepper** (the time integration algorithm)
- the **solver** (the algorithm that computes accelerations and reaction forces at each time step)
- other settings (for instance collision tolerances etc.)

Here is a primer to the tuning of these parameters.


# Time steppers {#time_steppers}

Time steppers are used to advance the simulation. They perform numerical integration.
As such, they are also known as _time integrators_.

Technical and theoretical details on time integration are explained in PDF documents 
available at the [white papers page](@ref whitepaper_root), for example
[this PDF](http://projectchrono.org/assets/white_papers/integrator.pdf) explains how 
implicit integrators are implemented in Chrono.

Time steppers can be changed in two ways:
- using the ```my_system.SetIntegrationType(...)``` function, that has ready-to-use 
  enums that you can choose 
- using the ```my_system.SetTimestepper(...)``` function, that allows plugging custom 
  timesteppers, maybe developed by you, or in optional modules.

Using the first 'easy' method, one changes the time stepper 
as in the following example:

~~~{.cpp}
my_system.SetIntegrationType(ChSystem::INT_IMPLICIT_EULER)
~~~

Among the available timesteppers, we suggest to use one of these most useful algorithms:

- ```INT_EULER_IMPLICIT_LINEARIZED```
	- fast, no sub-iterations required
	- first order accuracy
	- works for DVI (hard contacts)
	- in case of FEA it gives first order _implicit integration_
	- constraints kept closed using stabilization
	- default time stepper in Chrono
- ```INT_HHT```
	- slow, sub-iterations required
	- second order accuracy, with adjustable numerical damping
	- at the moment it does not work for DVI (hard contacts)
	- in case of FEA it gives second order _implicit integration_
	- constraints kept closed _exactly_ because of inner iterations.
- ```INT_NEWMARK```
    - popular in the FEA community, similar properties as INT_HHT

Depending on the type of integrator, maybe there are additional 
parameters to adjust. Those custom, advanced settings
are not accessible directly from @ref chrono::ChSystem,
so you should do like in the following example:

~~~{.cpp}
if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
    mystepper->SetAlpha(-0.2);
    ...
}
~~~

See @ref chrono::ChTimestepper for API details.


# Solvers {#solvers}

Solvers are called by the above discussed time steppers, in order to compute
the unknown accelerations and unknown reaction forces at each time step.

They often require a large amount of computations and, as such, they represent 
the biggest computational bottleneck of the entire simulation.

<div class="ce-info">
Note that you can mix different solvers and time steppers as you prefer. 
However, there are some combinations that work better.
</div>

Solvers can be changed in two ways:
- using the ```my_system.SetLcpSolverType(...)``` function, that has ready-to-use 
  enums that you can choose 
- using the ```my_system.ChangeLcpSolverSpeed(...)``` function, that allows plugging custom 
  solvers, maybe developed by you, or in optional modules.
  
Using the first 'easy' method, one changes the time stepper 
as in the following example:

~~~{.cpp}
my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
~~~

Among the available solvers, we suggest to use one of these most useful algorithms:

- ```LCP_ITERATIVE_SOR``` 	
	- maximum speed: good for real-time applications, 
	- low precision: convergence might stall, especially with odd mass ratios
	- supports DVI (hard contacts)
	
- ```LCP_ITERATIVE_APGD```	
	- slow but better convergence
	- supports DVI (hard contacts)
	
- ```LCP_ITERATIVE_BARZILAIBORWEIN``` 
    - slow but better convergence
	- supports DVI (hard contacts)   
    - very similar to ```LCP_ITERATIVE_APGD``` but a bit more robust when using large mass ratios
	
- ```LCP_ITERATIVE_MINRES``` 
    - good convergence
    - supports FEA problems
    - does nor support DVI (hard contacts) for the moment.	
	
- (etc.)

Most iterative solvers have an upper limit on number of iterations. 

- The higher, the more precise, but slower.

- The lower, the faster, but it is likely that it clamps the iteration when precision is not yet reached.

The max. number of iterations can be easily changed in this way:

~~~{.cpp}
// Change the max iterations for the solver
my_system.SetIterLCPmaxItersSpeed(20);
~~~

Depending on the type of solver, maybe there are additional 
parameters to adjust. Those custom, advanced settings
are not accessible directly from @ref chrono::ChSystem,
so you should do something along the lines of the following example:

~~~{.cpp}
if (auto msolver = dynamic_cast<chrono::ChLcpIterativeMINRES*>(my_system.GetLcpSolverSpeed())) {
	msolver->SetDiagonalPreconditioning(true);
}
~~~

See @ref chrono::ChLcpSolver for API details.


# Other parameters  {#other_simulation_parameters}

Here we discuss additional settings that can affect the outcome of a simulation,
and most of them are related to collision.

First of all, remember what we said about [collision tolerances](@ref collision_tolerances), 
then read the following.

### Max. recovery speed 

Objects in contact, that interpenetrate (ex for numerical errors, 
incoherent initial conditions, etc.) will not 'escape' 
one from the other faster than this threshold. Change it as: 

~~~{.cpp}
my_system.SetMaxPenetrationRecoverySpeed(0.2);
~~~

- The higher, the faster and more precisely are recovered 
  the contact constraints errors (if any), but the risk is 
  that objects ‘pop’ out, and stackings might become unstable and noisy.
  
- The lower, the more likely the risk that objects _sink_ 
  one into the other when the integrator precision is low 
  (ex small number of iterations).

  
### Min. bounce speed 

When objects collide, if their incoming speed is lower than 
this threshold, a zero restitution coefficient is assumed. 
This helps to achieve more stable simulations of stacked objects. 

~~~{.cpp}
my_system.SetMinBounceSpeed(0.1);
~~~

- The higher, the more likely is to get stable simulations, 
  but the less realistic the physics of the collision.
  
- The lower, the more realistic is the physics of the system, 
  but you need very short time step to take advantage of this, 
  otherwise you might incur in objects that bounce erratically forever.



# Theory

You can find additional information on the theoretical aspects of time integration 
in Chrono::Engine by looking at the [whitepapers page](@ref whitepaper_root).
