YAML schema for Chrono::FSI-TDPF model specification {#YAML_schema_fsitdpf_model}
=======================================

A Chrono YAML TDPF model file defines a linear potential-flow hydrodynamics model for Chrono::FSI-TDPF and contains
two main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The `model` object that identifies the hydrodynamic data file and, optionally, the wave conditions.

## Model specification

A TDPF model does not discretize the fluid. It applies hydrodynamic forces (added mass, radiation damping, and wave
excitation) to the FSI bodies of the coupled multibody model, based on pre-computed frequency-domain coefficients read
from an HDF5 file.

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `name` | Name of the model | string | -- | No | empty string |
| `angle_degrees` | Whether angles are specified in degrees (true) or radians (false) | boolean | -- | No | `true` |
| `data_path` | Location of the data files referenced below | object | -- | No | absolute paths |
| `h5_file` | Input file with hydrodynamic data (HDF5 format) | string | -- | Yes | -- |
| `waves` | Specification of wave conditions | object | -- | No | no waves |

The `data_path` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Mode for data file location | enum | `ABSOLUTE`,`RELATIVE` | Yes | `ABSOLUTE` |
| `root` | Root of data files, relative to the location of this file | string | -- | No | `.` |

#### Wave conditions

The `waves` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Wave type | enum | `NONE`,`REGULAR`,`IRREGULAR` | Yes | `NONE` |
| `height` | Wave height (twice the wave amplitude) | double | -- | Yes, for `REGULAR` | -- |
| `period` | Wave period in seconds | double | -- | Yes, for `REGULAR` | -- |
| `phase` | Wave phase | double | -- | No | 0 |
| `stretching` | Whether to apply wave stretching | boolean | -- | No | `true` |

Note that `IRREGULAR` waves are accepted by the parser but not yet implemented; no waves are generated for that type.

## Example

Below is an example of a TDPF model configuration:

\include data/yaml/fsi/sphere_decay/model_tdpf.yaml

## YAML schema

The YAML TDPF model specification file must follow the ``data/yaml/schema/fsitdpf_model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/fsitdpf_model.schema.yaml

