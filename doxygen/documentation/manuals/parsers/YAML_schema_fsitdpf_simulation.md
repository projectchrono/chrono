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

** **TODO** **

The `model` YAML file must follow the [TDPF model schema](@ref YAML_schema_fsitdpf_model).

The `solver` YAML file must follow the [TDPF solver schema](@ref YAML_schema_fsitdpf_solver).


#### Output options

** **TODO** **

#### Visualization options

** **TODO** **


## Example

Below is an example of an FSI-TDPF simulation configuration:

\include data/yaml/fsi/sphere_decay/tdpf.yaml


## YAML schema

The YAML TDPF simulation specification file must follow the ``data/yaml/schema/fsitdpf_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsitdpf_simulation.schema.yaml

