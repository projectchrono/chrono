YAML schema for Chrono::Vehicle model specification {#YAML_schema_vehicle_models}
========================================

A Chrono::Vehicle YAML FSI problem specification file defines the vehicle models and consists of two main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fileds are verified for compatibility.
- [required] The `model` object which defines JSON specification files for the vehicle sub-systems.

## Model specification

## YAML schema

The YAML vehicle model specification file must follow the ``data/yaml/schema/vehicle_model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/vehicle_model.schema.yaml

