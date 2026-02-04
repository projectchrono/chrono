YAML schema for Chrono::FSI-SPH solver specification {#YAML_schema_fsisph_solver}
=======================================

A Chrono YAML SPH simulation file defines the parameters needed to run a Chrono::SPH simulation. It consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- The `sph` object specifying SPH method parameters.
- The `kernel` object specifying the SPH kernel definition.
- The `discretization` object specifying parameters for the SPH discretization.
- The `boundary_conditions` object specifying the method and parameters for treating fluid-solid coundary conditions.
- The `integration` object specifying the type and parameters for the time integrator.
- The `proximity_search` object specifying parameters for the proximity (neighbor) search algorithm.
- The `particle_shifting` object specifying the method and parameters for the particle shifting algorithm.
- The `viscosity` object specifying the method and parameters for viscosity treatment.


## SPH method specification

** **TODO** **

## Kernel specification

** **TODO** **

## SPH discretization specification

** **TODO** **

## Boundary condition treatment

** **TODO** **

## Integrator specification

** **TODO** **

## Proximity search treatment

** **TODO** **

## Particle shifting treratment

** **TODO** **

## Viscosity treatment

** **TODO** **


## YAML schema

The YAML SPH solver specification file must follow the ``data/yaml/schema/fsisph_solver.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsisph_solver.schema.yaml
