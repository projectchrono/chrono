
Simulation system      {#simulation_system}
=================

The Chrono system contains all other objects being simulated, e.g.: 
[bodies](@ref rigid_bodies), [links](@ref links), etc.
The system is the cornerstone of a Chrono simulation.

\tableofcontents


# ChSystem  {#manual_ChSystem}

A Chrono simulation system is an object of class ChSystem. 
See @ref chrono::ChSystem "ChSystem" for API details.

Note that ChSystem is an abstract class: you must instantiate one of its specializations. In detail you can use of these sublclasses:

- A @ref chrono::ChSystemNSC "ChSystemNSC" for **Non Smooth Contacts** (NSC); in case of contacts a complementarity solver will take care of them using non smooth dynamics; this is very efficient even with large time steps.

- A @ref chrono::ChSystemSMC "ChSystemSMC" for **SMooth Contacts** (SMC); with this system contacts are handled using penalty methods, i.e. contacts are deformable; 

If there are no contacts or collisions in your system, it is indifferent to use ChSystemNSC or ChSystemSMC.


Example:
The following picture shows how mechanisms turn into a database of bodies 
and links in a ChSystem:

![](http://www.projectchrono.org/assets/manual/pic_database.png)

- A @ref chrono::ChSystem "ChSystem" contains all items that participate in a simulation: bodies, constraints, numerical integrator type, integration tolerances, etc.

- Use the ```Add()```, ```Remove()``` functions to add elements to a system object

Recommended way of dealing with system objects:

- Create a @ref chrono::ChSystemNSC "ChSystemNSC" or a @ref chrono::ChSystemSMC "ChSystemSMC"
- Add [body](@ref rigid_bodies) objects into it, see @ref chrono::ChBody "ChBody"
- Add [link](@ref links) objects into it, see @ref chrono::ChLink "ChLink"
- Adjust parameters for the time integration
- Run a dynamics simulation

Refer to [demo_MBS_crank](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_crank.cpp) for a basic example.


The default simulation settings are good for real-time 
fast simulations with low requirements in terms of precision 
and with similar sizes and inertia attributes. 

Several system settings may be adjusted to simulate more challenging scenarios.
In most cases there are three areas to adjust:

- The **time stepper**; i.e., the time integration algorithm
- The **solver**; i.e., the algorithm that computes accelerations and reaction forces at each time step
- Other settings, for instance, the collision detection tolerances

A primer on tuning these parameters is provided below.


# Time steppers {#time_steppers}

Time steppers, also known as _time integrators_, are used to advance the simulation. They perform numerical integration; i.e., they advance the state of the system in time.

Technical and theoretical details on time integration are explained in several PDF documents 
available on the [white papers page](http://projectchrono.org/whitepapers/). For example the
[Integrator Whitepaper](http://projectchrono.org/assets/white_papers/integrator.pdf) explains how 
implicit integrators are implemented in Chrono.

Time steppers can be changed in two ways:
- Using the ```my_system.SetTimestepperType(...)``` function, to choose a ready-to-use, pre-packaged time-stepper 
- Using the ```my_system.SetTimestepper(...)``` function, to plug in a custom time-stepper, which is user-defined

Example: changing the time stepper to an implicit numerical integrator


~~~{.cpp}
my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT)
~~~

Summary of time-steppers:


- ```EULER_IMPLICIT_LINEARIZED```
	- Default time stepper in Chrono
	- Fast, no sub-iterations required
	- First order accuracy
	- Works for DVI contacts (hard contacts)
	- Delivers first order accuracy for FEA
	- Constraints kept closed using stabilization
- ```HHT```
	- Implicit integrator, based on the Hilber-Hughes-Taylor formula
	- Typically slower than the INT_EULER_IMPLICIT_LINEARIZED choice
	- Sub-iterations required
	- Second order accuracy, with adjustable numerical damping
	- Currently can't be used for systems that handle contacts in a DVI approach; i.e., for hard contacts
	- Delivers second order accuracy for FEA
	- Constraints kept closed _exactly_ because of inner iterations.
- ```NEWMARK```
    - Popular in the FEA community, similar properties as INT_HHT
	- With the exception of one particular choice of parameters (in which it becomes the trapezoidal integration rule) it delivers first order accuracy

	In the above, the meaning of 'first order' or 'second order' accuracy is that the global integration error goes to zero as the value of the time step (or square of the time step for a second order method).
	
Depending on the type of time-stepper used, there may be different parameters to adjust.
Example:

~~~{.cpp}
if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
    mystepper->SetAlpha(-0.2);
    ...
}
~~~

See @ref chrono::ChTimestepper "ChTimestepper" API for details.


# Solvers {#solvers}

A solver is called by a time stepper to compute
the unknown accelerations and unknown reaction forces at each time step of the simulation. Most often they represent 
the biggest computational bottleneck of the entire simulation.

Solvers can be changed in two ways:
- Using the ```my_system.SetSolverType(...)``` function, to choose a ready-to-use, pre-packaged option 
- Using the ```my_system.SetSolver(...)``` function, to plug in a custom time-stepper, which is user-defined

  
Example: 

~~~{.cpp}
my_system.SetSolverType(ChSolver::Type::PSOR);
~~~

We recommend using one of the following iterative solvers:

- ```PSOR``` 	
	- low precision: convergence might stall, especially with odd mass ratios
	- supports DVI (hard contacts, with complementarity)
	- used most often for small problems, solution accuracy is not particularly important
	
- ```APGD```	
	- very good convergence, used most often for simulations in which high accuracy is desired
	- supports DVI (hard contacts, with complementarity)
	
- ```BARZILAIBORWEIN``` 
    - good convergence
	- supports DVI (hard contacts, with complementarity)   
    - similar to ```APGD```, might be more robust when using large mass ratios
	
- ```MINRES``` 
    - good convergence
    - supports FEA problems
    - does not support DVI (hard contacts, with complementarity) for the moment.

- ```ADMM + PardisoMKL``` 
    - supports FEA problems and DVI problems
    - requires an inner linear solver; best choice is ```PardisoMKL``` (requires PARDISO_MKL module) otherwise it will use \ref chrono::ChSolverSparseQR "ChSolverSparseQR".
	
While using iterative solvers it is **highly recommended**, especially in case of systems with links/constraints, to increase the number of iterations until the links do not get violated anymore. This is done by:

~~~{.cpp}
// Change the max iterations for the solver
my_system.SetSolverMaxIterations(200);
~~~

Depending on the type of solver used, there may be different parameters to adjust.
Advanced settings are not accessible directly from @ref chrono::ChSystem "ChSystem",
for instance:

~~~{.cpp}
if (auto msolver = std::dynamic_pointer_cast<ChSolverMINRES>(my_system.GetSolver())) {
	msolver->SetDiagonalPreconditioning(true);
}
~~~

See @ref chrono::ChSolver "ChSolver" API for further details.


# Other parameters  {#other_simulation_parameters}

There are many integrator/solver settings that can affect the outcome of a simulation. For instance, see [collision tolerances](@ref collision_tolerances) to gain a better understanding of the interplay between the accuracy in the collision detection and robustness of a simulation. We focus below on two important settings related to handling collisions in a simulation in which bodies collide with each other and/or with the ground.


### Max. recovery speed 

Bodies in contact that interpenetrate for various reasons, e.g., small numerical integration error, inconsistent initial conditions, etc., will not 'escape' this contact violation at a speed faster than this threshold. The recovery speed is in general problem dependent and is controlled by user as shown below. 

~~~{.cpp}
my_system.SetMaxPenetrationRecoverySpeed(0.2);
~~~

- Larger values allow a more aggressive correction of the penetration, yet this can lead to scenarios in which bodies in contact pop out fast or in stacking problems the stack becoming jittery, noisy   
- A small threshold increases the risk that objects _sink_ into one another when integrator precision is low, for instance, when the solver has a small max number of iterations

  
### Min. bounce speed 

When objects collide, if their incoming speed is lower than 
this threshold, a zero restitution coefficient is assumed. 
This helps to achieve more stable simulations of stacked objects. 

~~~{.cpp}
my_system.SetMinBounceSpeed(0.1);
~~~
- A higher value leads to more stable simulations but less physically realistic collisions

- Lower values lead to a more physically realistic time evolution but require small integration time steps otherwise objects may keep bounce erratically



# Theory

Additional information regarding the time integration strategies implemented
in Chrono can be found on the  [white papers page](http://projectchrono.org/whitepapers/).
