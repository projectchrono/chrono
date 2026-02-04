YAML schema for Chrono::Vehicle simulation specification {#YAML_schema_vehicle_simulation}
========================================

A Chrono YAML vehicle simulation file defines the setup for a Chrono::Vehicle simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The simulation `type`, which must be `VEHICLE` here.
- [required] The vehicle `model` object which defines YAML specification of vehicle system.
- [required] The MBS `solver` object which defines YAML specification of the solver algorithms.
- [optional] The `simulation` object specifying how simulation is to be performed.
- [optional] The `output` object which specifies output options from the MBS simulation.
- [optional] The `visualization` object which specifies run-time visualization settings.

## Vehicle simulation specification

A vehicle simulation must specify the vehicle model to be simulated, integrator and solver settings, as well as optional output and run-time visualization settings.

** **TODO** ** Supported Chrono version and simulation type (VEHICLE)

#### Model and solver specification

** **TODO** **

The `model` YAML file must follow the [vehicle model schema](@ref YAML_schema_vehicle_model).

The `solver` YAML file must follow the [MBS solver schema](@ref YAML_schema_mbs_solver).


#### Simulation options

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `enforce_realtime` | Whether to enforce real-time simulation | boolean | -- | No | false |
| `end_time` | Total simulation time in seconds | double | -- | No | -1 for infinite simulation |
| `gravity` | Gravitational acceleration vector [x, y, z] | array[3] | -- | No | [0, 0, -9.8] |


#### Output options

** **TODO** **

#### Visualization options

** **TODO** **

## Example

Below is an example of a simulation configuration:

\include data/yaml/vehicle/vehicle.yaml

## YAML schema

The YAML vehicle simulation specification file must follow the ``data/yaml/schema/vehicle_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/vehicle_simulation.schema.yaml

