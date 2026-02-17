YAML schema for Chrono::FSI-TDPF simulation specification {#YAML_schema_fsitdpf_simulation}
=======================================

A Chrono YAML TDPF simulation file defines the setup for a Chrono::FSI-TDPF simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The simulation `type`, which must be `TDPF` here.
- [required] The SPH `model` object which defines YAML specification of TDPF problem.
- [required] The SPH `solver` object which defines YAML specification of TDPF solver algorithms.
- [optional] The `output` object which specifies output options from the TDPF simulation.
- [optional] The `visualization` object which specifies TDPF-specific run-time visualization settings.


## FSI-TDPF simulation specification

An FSI-TDPF simulation must specify the TDPF model to be simulated, as well as optional output and TDPF-specific run-time visualization settings.

#### Model and solver specification

The `model` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with a TDPF model specification (which must follow the [TDPF model schema](@ref YAML_schema_fsitdpf_model)).

The `solver` entry (required) must specify the path (relative to the location of this YAML simulation specification file) to
the YAML file with a TDPF solver specification (which must follow the [TDPF solver schema](@ref YAML_schema_fsitdpf_solver)).

#### Output options

If the `output` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default | 
|----------|-------------|------|------------------|----------|---------|
| `type` | Output DB type | enum | `NONE`,`ASCII`,`HDF5`  | No | `NONE` |
| `mode` | Output mode | enum | `FRAMES`,`SERIES`  | No | `FRAMES` |
| `fps` | Output frequency (FPS or Hz) | double | -- | No | 30 |

#### Visualization options

** **TODO** ** (TDPF-specific visualization options)


## Example

Below is an example of an FSI-TDPF simulation configuration:

\include data/yaml/fsi/sphere_decay/tdpf.yaml


## YAML schema

The YAML TDPF simulation specification file must follow the ``data/yaml/schema/fsitdpf_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsitdpf_simulation.schema.yaml

