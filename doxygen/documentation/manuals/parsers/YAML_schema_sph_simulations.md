YAML schema for Chrono::SPH fluid simulation specification {#YAML_schema_sph_simulations}
=======================================

A Chrono YAML SPH simulation file defines the parameters needed to run a Chrono::SPH simulation. It consists of three main objects:
- The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- The fluid solver type, a string identifying supported CFD solvers.
- The `simulation` object that contains simulation methods, solver and integrator settings, and visualization options.

## Simulation specification


## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/sph_simulation.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/sph_simulation.schema.yaml
