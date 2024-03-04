Chrono FMI module tutorials {#tutorial_table_of_content_chrono_fmi}
===========================

- Hydraulic crane co-simulation

  This [demo](https://github.com/projectchrono/chrono/tree/main/fmu/src/demos/fmi) generates two Chrono FMUs and uses them in a co-simulation.

  The first FMU, `demo_FMI_craneFMU`, encapsulates a simple multibody system consisting of an inverted pendulum (connected through a revolute joint to the ground) representing the main mast of the crane and a second pendulum representing the payload. The crane body accepts an externally-provided actuation force applied at a point on the body (as its primary continuous input FMI variable) and provides the actuator length and length rate of change as main FMI continuous outputs.

  The second FMU, `demo_FMI_actuatorFMU`, encapsulates a hydraulic actuator consisting of a hydraulic piston, a directional valve, hoses, a pump, and an oil tank. As main FMI variables, this FMU accepts as continuous input the actuator length and length rate of change and provides as continuous output the actuator force.

  `demo_FMI_hydraulic_crane_cosim` illustrates the use of these two FMUs in an explicit force-displacement co-simulation loop. 

- Chrono::Vehicle FMUs

  If both the Chrono::FMI and the [Chrono::Vehicle](@ref vehicle) modules are enabled, several FMUs encapsulating vehicle systems are generated:
  
  - `FMU_WheeledVehicle` is a co-simulation FMU encapsulating a wheeled vehicle system with 4 wheels. The vehicle includes a powertrain and transmission, but no tires.
  
    The wrapped Chrono::Vehicle model is defined through JSON specification files for the vehicle, engine, and transmission. This vehicle FMU must be co-simulated with a driver system which provides vehicle commands and 4 tire systems which provide tire loads for each of the 4 wheels (of type TerrainForce).
  
    This vehicle FMU defines continuous output variables for:
      - vehicle reference frame (of type ChFrameMoving)
      - wheel states (of type WheelState)
    <br>

  - `FMU_ForceElementTire` is a co-simulation FMU encapsulating a "force element" (handling) tire system.

    The wrapped Chrono::Vehicle tire model is defined through a JSON specification file which is assumed to define a tire of ChForceElementTire type.

    This tire FMU must be co-simulated with a vehicle system which provides the current wheel state (of type WheelState) and a terrain system which provides local terrain information (height, normal, and coefficient of friction) at a single query point.

    This tire FMU defines continuous output variables for:
      - wheel tire/terrain load (of type TerrainForce)
      - location of the terrain query point (of type ChVector)
    <br>

  - `FMU_PathFollowerDriver` is a co-simulation FMU encapsulating a vehicle path-follower driver system. The driver model includes a lateral path-following PID controller and a longitudinal PID cruise controller.

    The wrapped Chrono::Vehicle driver model is defined through a data file specifying the path for the lateral controller.

    This driver FMU must be co-simulated with a vehicle system which provides the current vehicle reference frame (of type ChFrameMoving).
    
    This driver FMU defines continuous output variables for:
      - sterring command (in [-1,1])
      - throttle command (in [0,1]
      - braking command (in [0,1])  

To see how these FMUs can be used together in a co-simulation loop, we provide two demo programs:

  - `demo_VEH_FMI_WheeledVehicle_a`

    This program uses the WheeledVehicle and PathFollowerDriver, together with a set of 4 Chrono::Vehicle tires and a Chrono::Vehicle terrain system to co-simulate a vehicle following a prescribed path at constant speed.

  - `demo_VEH_FMI_WheeledVehicle_b`

    This program uses the WheeledVehicle, 4 instances of the ForceElementTire, and the PathFollowerDriver, in conjunction with a Chrono::Vehicle terrain system to co-simulate a vehicle following a prescribed path at constant speed.
    