YAML schema for Chrono::SPH model specification {#YAML_schema_sph_models}
=======================================

A Chrono YAML SPH model file defines a fluid system for Chrono::SPH and contains two main objects:
- The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- The `model` object that lists all physics items in the Chrono model.

## Model specification

## YAML schema

The YAML model specification file must follow the ``data/yaml/schema/sph_model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/sph_model.schema.yaml
