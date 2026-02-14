YAML schema for Chrono::FSI-TDPF solver specification {#YAML_schema_fsitdpf_solver}
=======================================

A Chrono YAML TDPF solver file defines the parameters needed to run a Chrono::SPH simulation. It consists of:
- The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.

TDPF simulations do not need to specify any integrator or solver settings as they only represent applied forces on FSI bodies (and therefore use the MBS solver).

## YAML schema

The YAML TDPF solver specification file must follow the ``data/yaml/schema/fsitdpf_solver.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsitdpf_solver.schema.yaml
