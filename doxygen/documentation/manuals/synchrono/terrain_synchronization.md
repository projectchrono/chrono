Deformable Terrain Synchronization {#terrain_synchronization}
=================================

SynChrono provides state synchronization for deformable terrain through Chrono::Vehicle's [Soil Contact Model (SCM) implementation](@ref vehicle_terrain_scm). This allows for multiple vehicles to drive off-road and interact with ruts and deformation created by each other. 

<img src="http://www.projectchrono.org/assets/manual/vehicle/terrain/SCM_mesh_refinement.png" width="600" />

## SCM Terrain Background {#terrain_sync_background}

More details on the physics behind the SCM terrain can be found in the Chrono::Vehicle [terrain reference guide](@ref vehicle_terrain_scm). For the purposes of synchronization, the key characteristics of SCM terrain are that:

- Soil is represented by an integer grid, only nodes that have been deformed are maintained in a hash map
- Soil nodes are only displaced vertically
- There is no soil history - vertical displacement captures the full soil state

## Synchronizing Terrain State {#terrain_sync_mechanics}

To synchronize state between two systems, all that needs to be communicated is a list of `(int x, int y)` representing the nodes that were deformed, along with a list of `double z` indicating the current height of the corresponding nodes. During the course of a time step for the terrain's physics update, a list of which nodes were deformed and their height comes as an easy byproduct of calculations that the terrain system already does. Since this list will be reset every physics time step, but SynChrono will only send out messages every heartbeat, the deformed nodes are merged every time step into a map owned by a `SynSCMTerrainAgent` class, which is then reset only once per heartbeat. The message sent out in a FlatBuffer message is a vector of `int x, int y, double z`, each representing a node deformed during the preceding heartbeat.

When terrain state information is received on a synchronizing node, the updates are immediately applied via the [vehicle::SCMDeformableTerrrain::SetModifiedNodes](@ref chrono::vehicle::SCMDeformableTerrain::SetModifiedNodes) function and under the hood the terrain class takes care of applying these external node updates.

## Caveats {#terrain_sync_caveats}

Some additional notes about the implementation:
- Nothing in the code will prevent the user from erroneously setting different soil properties or dimensions on different ranks.
- Vehicles should not modify the same nodes during the same heartbeat -- the update that actually goes through to the terrain will be random. This should not be more restrictive than the normal requirement that vehicles do not collide, since the only nodes modified are underneath the vehicle's wheels.