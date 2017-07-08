### Simulation Parameters

All of the following simulation parameters are set in `void SetupParamsH(SimParams & paramsH)` function. This function can be found in `main.cpp`.

* `paramsH.sizeScale`: Useless
* `paramsH.hsml`: Interaction Radius. (or h)
* `paramsH.MULT_INITSPACE`: Multiplier to hsml to determine the initial separation of the fluid particles and the fixed separation for the boundary particles. This means that the separation will always be a multiple of hsml. Default value = 1.0.
* `paramsH.NUM_BOUNDARY_LAYERS`: Number of particles layers that will be used in the boundary. Default value = 3.
* `paramsH.toleranceZone`: Helps determine the particles that are in the domain but are outside the boundaries, so they are not considered fluid particles and are dropped at the beginning of the simulation.
* `paramsH.NUM_BCE_LAYERS`: Number of fixed particle layers to rigid/flexible bodies which act as the boundaries. Default value = 2.
* `paramsH.BASEPRES`: Relative value of pressure applied to the whole domain.
* `paramsH.LARGE_PRES`: Artificial pressure for boundary particles. Make sure fluid particles do not go through the boundaries. Note that if time step is not small enough particles near the boundaries might build up huge pressures and will make the simulation unstable.
* `paramsH.deltaPress`: 
* `paramsH.nPeriod`: Only used in snake channel simulation. Tells you how long the channel will be.
* `paramsH.gravity`: Gravity. Applied to fluid, rigid and flexible.
* `paramsH.bodyForce3`: Constant force applied to the fluid. Flexible and rigid bodies are not affected by this force directly, but instead they are affected indirectly through the fluid. 
* `paramsH.rho0`: Density. 
* `paramsH.mu0`: Viscosity.
* `paramsH.v_Max`:  Max velocity of fluid used in equation of state. Run simulation once to be able to determine it.
* `paramsH.EPS_XSPH`: Method to modify particle velocity.
* `paramsH.multViscosity_FSI`: Multiplier that helps determine the viscosity for boundary particles. For example, if the value is 5 then the boundary particles will be 5 times more viscous than the fluid particles. Boundary particles should be more viscuous becayse they are supposed to slow down the fluid particles near the boundary.  
* `paramsH.dT`: Time step. Depending on the model this will vary and the only way to determine what time step to use is to run simulations multiple time and find which one is the largest dT that produces a stable simulation.
* `paramsH.tFinal`: Total simulation time.
* `paramsH.timePause`: Time that we let pass before applying body forces. This is done to allow the particles to stabilize first.
* `paramsH.timePauseRigidFlex`: Time before letting rigid/flex move.
* `paramsH.kdT`: Implicit integration parameter. Not very important
* `paramsH.gammaBB`: Equation of state parameter.
* `paramsH.binSize0`: Determines the length of the bin each particle occupies. Normally this would be `2*hsml` since hsml is the radius of the particle, but when we have periodic boundary condition varies a little from `2*hsml`.
* `paramsH.rigidRadius`: Radius of rigid bodies.
* `paramsH.densityReinit`:
* `paramsH.contactBoundary`: 0: straight channel, 1: serpentine
* `paramsH.straightChannelBoundaryMin`: Origin of the coordinate system (Point (0,0,0)). In this case (straigh channel) this point is where the leftmost particle of the 3rd  layer of boundary particles is located. Everything below this point is outside the tolerance zone, this means that everything below is not considered part of the system.
* `paramsH.straightChannelBoundaryMax`: Upper right most part of the system, this is `Point(2 * mm, 1 * mm, 3 * mm)` where mm is a constant defined at the beginning of `main.cpp`. This is also the rightmost particle of the 3rd layer of the top boundary particles. Everything above this point is not considered part of the system.
* `paramsH.cMin`: Lower leftmost part of the space shown in a simulation frame. This point is usually outside the tolerance zone.
* `paramsH.cMax`: Upper right most part of the space shown in a simulation frame. This point is usually outside the tolerance zone.
