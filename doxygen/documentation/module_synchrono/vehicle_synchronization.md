Wheeled and Tracked Vehicle Synchronization {#vehicle_synchronization}
=================================

\tableofcontents

SynChrono provides state synchronization for [Chrono::Vehicle](@ref manual_vehicle) model templates. Wheeled and tracked vehicle synchronization are currently supported.

## General Concepts {#veh_sync_general_concepts}

To achieve SynChrono's goal of extending Project Chrono's physics into the multi-agent realm, the first step was to enable large-scale vehicle scenarios. Single vehicle simulations was already supported through the [Chrono::Vehicle](@ref manual_vehicle) module, but a scalable solution for 10s to 100s of additional vehicles was desired. Through SynChrono's synchronization backbone, this became feasible. Synchronization within SynChrono is explained in depth [here](state_synchronization.html). 

To allow for time and space coherence between agents, state passing is performed at a predetermined heartbeat to effectively pretend all vehicles are simulated within the same world. [SynAgent's](group__synchrono__agent.html) manage both the construction and synchronization of these pretend agents (or _zombies_) and handle generating the state messages to be distributed between agents. The [SynWheeledVehicleAgent](@ref chrono::synchrono::SynWheeledVehicleAgent) and [SynTrackedVehicleAgent](@ref chrono::synchrono::SynTrackedVehicleAgent) classes are the thin wrappers used for [ChWheeledVehicle's](@ref chrono::vehicle::ChWheeledVehicle) and [ChTrackedVehicle's](@ref chrono::vehicle::ChTrackedVehicle). Over the course of a simulation, a VehicleAgent will initialize a zombie for each other VehicleAgent initialized on other nodes, distribute its own state to the other nodes and synchronize each zombie to match its corresponding agent. This cycle can be seen in the below figure.

<img src="http://www.projectchrono.org/assets/manual/synchrono_vehicle_synchronization.png" alt="DDS System" width=60%>

Wheeled and tracked vehicles will distribute individual state and description information through SynChrono. State data describes the current position and orientation of specific components of the vehicles. During the initial handshake of SynChrono, description messages are sent that describe how to reconstruct the vehicle as a zombie. See the [Wheeled Vehicle Synchronization](#wheeled-vehicle-synchronization) or the [Tracked Vehicle Synchronization](#tracked-vehicle-synchronization) sections for a detailed description about the exact data types.

### Limitations {#veh_sync_limitations}

Currently, only position and orientation of various vehicle components is shared between agents. Contact forces are not shared, hence agents are only space coherent (i.e. have knowledge of), not space interactive. 

## Wheeled Vehicle Synchronization {#veh_sync_wheeled}

A [SynWheeledVehicleAgent](@ref chrono::synchrono::SynWheeledVehicleAgent) simply wraps a pointer for a [ChWheeledVehicle's](@ref chrono::vehicle::ChWheeledVehicle).

### Description Messages {#veh_sync_wheeled_description}

To reconstruct a [ChWheeledVehicle's](@ref chrono::vehicle::ChWheeledVehicle), one must know the visual representation of its chassis, rims and tires. To enhance generality and allow for multiple vehicle configurations (i.e. more than four wheels), the number of wheels is sent, as well. Each visual file is represented as a string which is then read relative to the Chrono data directory. 

Upon initialization of the simulation, the chassis, wheels and tires are placed at the origin. The position is then updated as state messages are received.

```protobuf
table Description {
  chassis_vis_file:string;
  wheel_vis_file:string;
  tire_vis_file:string;

  num_wheels:int;
}
```

### State Messages {#veh_sync_wheeled_state}

The state information for a wheeled vehicle is rather simple. The pose (position and orientation) of the chassis and the models wheels are sent to the other agents. Wheels and tires share the same pose information.

On reception, the bodies created at initialization through the description message have their position updated.

For sending, the attached vehicle pointer is queried for position and orientation of the passed components.

```protobuf
table State {
  time:double;

  chassis:Pose;

  wheels:[Pose];
}
```

## Tracked Vehicle Synchronization {#veh_sync_tracked}

A [SynTrackedVehicleAgent](@ref chrono::synchrono::SynTrackedVehicleAgent) simply wraps a pointer for a [ChTrackedVehicle's](@ref chrono::vehicle::ChTrackedVehicle).

### Description Messages {#veh_sync_tracked_description}

To reconstruct a [ChTrackedVehicle's](@ref chrono::vehicle::ChTrackedVehicle), one must know the visual representation of its chassis, track shoes, sprockets, idlers and road wheels. Other than the chassis and track shoe, different meshes could be used for different sides, so the left and right files are sent separately. To enhance generality and allow for multiple vehicle configurations (i.e. more or less sprockets/idlers/etc.), the number of track shoes, sprockets, idlers and road_wheels are sent. Each visual file is represented as a string which is then read relative to the Chrono data directory.

```protobuf
table Description {
  chassis_vis_file:string;
  track_shoe_vis_file:string;
  left_sprocket_vis_file:string;
  right_sprocket_vis_file:string;
  left_idler_vis_file:string;
  right_idler_vis_file:string;
  left_road_wheel_vis_file:string;
  right_road_wheel_vis_file:string;

  num_track_shoes:int;
  num_sprockets:int;
  num_idlers:int;
  num_road_wheels:int;
}
```

### State Messages {#veh_sync_tracked_state}

The state information for a tracked vehicle is simply a long list of poses for each component handled by the agent. The position of each component is updated on reception and the attached vehicle pointer is queried for position and orientation on sending.

```protobuf
table State {
  time:double;

  chassis:Pose;

  track_shoes:[Pose];
  sprockets:[Pose];
  idlers:[Pose];
  road_wheels:[Pose];
}
```