YAML schema for Chrono::FSI problem specification {#YAML_schema_fsi_problems}
========================================

A Chrono YAML FSI problem specification file defines the multibody and fluid problems and co-simulation parameters required to run a Chrono::FSI co-simulation.
It consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The `model` object which defines YAML specification files for the multibody and fluid models and solvers.
- [required] The `simulation` object which defines the co-simulation metastep and the simulation duration.
- [optional] The `visualization` object which enables run-time visualization and defines rendering frequency.
- [optional] The `output` object which enables simulation output and defines the output frequency.

## Model specification

An FSI problem couples a Chrono MBS model and simulation to a fluid solver. Any fluid solver that implements the Chrono::FSI API can be used.

The multibody and fluid models and simulation are specified by referring to the corresponding YAML specification files. These file names must include the path to the files, relative to the location of this Chrono::FSI YAML specification.


## Simulation specification

## Visualization specification

## Output specification

## YAML schema

The YAML FSI problem specification file must follow the ``data/yaml/schema/fsi.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsi.schema.yaml

