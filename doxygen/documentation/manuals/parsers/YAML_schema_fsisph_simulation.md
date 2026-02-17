YAML schema for Chrono::FSI-SPH simulation specification {#YAML_schema_fsisph_simulation}
=======================================

A Chrono YAML SPH simulation file defines the setup for a Chrono::FSI-SPH simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The simulation `type`, which must be `SPH` here.
- [required] The SPH `model` object which defines YAML specification of SPH problem.
- [required] The SPH `solver` object which defines YAML specification of SPH solver algorithms.
- [optional] The `output` object which specifies output options from the SPH simulation.
- [optional] The `visualization` object which specifies SPH-specific run-time visualization settings.

## FSI-SPH simulation specification

An FSI-SPH simulation must specify the SPH model to be simulated, SPH solver solver settings, as well as optional output and SPH-specific run-time visualization settings.

#### Model and solver specification

The `model` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an SPH model specification (which must follow the [SPH model schema](@ref YAML_schema_fsisph_model)).

The `solver` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with an SPH solver specification (which must follow the [SPH solver schema](@ref YAML_schema_fsisph_solver)).


#### Output options

If the `output` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Output DB type | enum | `NONE`,`ASCII`,`HDF5`  | No | `NONE` |
| `mode` | Output mode | enum | `FRAMES`,`SERIES`  | No | `FRAMES` |
| `fps` | Output frequency (FPS or Hz) | double | -- | No | 30 |

#### Visualization options

** **TODO** **  (SPH-specific visualization options)


## Example

Below is an example of an FSI-SPH simulation configuration:

\include data/yaml/fsi/sphere_decay/sph.yaml

## YAML schema

The YAML SPH simulation specification file must follow the ``data/yaml/schema/fsisph_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsisph_simulation.schema.yaml
