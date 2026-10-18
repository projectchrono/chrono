YAML schema for Chrono::Vehicle model specification {#YAML_schema_vehicle_model}
========================================

A Chrono::Vehicle YAML model specification file defines a vehicle model by referring to the Chrono::Vehicle JSON
specification files of its sub-systems. It consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML model specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The `model` object which defines JSON specification files for the vehicle sub-systems.
- [optional] The `terrain_json` entry which names a JSON specification file for a rigid terrain.
- [optional] The `initial_position` and `initial_yaw` entries which define the initial vehicle pose.
- [optional] The `chase_camera` object which defines the run-time chase-camera settings.

Note that only `model` is a nested object; `terrain_json`, `initial_position`, `initial_yaw`, and `chase_camera` are
specified at the top level of the file, as siblings of `model`.

## Model specification

The `model` object identifies the JSON files describing the vehicle and its powertrain.
The vehicle type (wheeled or tracked) is not stated in the YAML file; it is inferred from the `Template` field of the
vehicle JSON file.

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `name` | Name of the model | string | -- | No | empty string |
| `angle_degrees` | Whether angles are specified in degrees (true) or radians (false) | boolean | -- | No | `true` |
| `data_path` | Location of the JSON data files referenced below | object | -- | No | absolute paths |
| `vehicle_json` | JSON file for the vehicle model | string | -- | Yes | -- |
| `engine_json` | JSON file for the engine model | string | -- | Yes | -- |
| `transmission_json` | JSON file for the transmission model | string | -- | Yes | -- |
| `tire_json` | JSON file for the tire model | string | -- | Yes, for wheeled vehicles | -- |

The `data_path` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Mode for data file location | enum | `ABSOLUTE`,`RELATIVE` | Yes | `ABSOLUTE` |
| `root` | Root of data files, relative to the location of this file | string | -- | No | `.` |

## Terrain, initial pose, and chase camera

These entries are specified at the top level of the vehicle model file.

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `terrain_json` | JSON file for a rigid terrain. If absent, no terrain is created. | string | -- | No | no terrain |
| `initial_position` | Initial location of the vehicle reference frame | array[3] | -- | No | [0, 0, 0] |
| `initial_yaw` | Initial vehicle heading, always in degrees | double | -- | No | 0 |
| `chase_camera` | Run-time chase-camera settings | object | -- | No | -- |

Note that `initial_yaw` is always interpreted in degrees, independent of the `angle_degrees` setting in `model`.

The `chase_camera` key, if present, specifies the following properties (all of which are then required):

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `chassis_point` | Camera target point, in the chassis reference frame | array[3] | -- | Yes | -- |
| `chase_distance` | Initial horizontal distance between camera and target point | double | -- | Yes | -- |
| `chase_height` | Initial vertical distance between camera and target point | double | -- | Yes | -- |

## Example

Below is an example of a wheeled vehicle model configuration:

\include data/yaml/vehicle/polaris.yaml

## YAML schema

The YAML vehicle model specification file must follow the ``data/yaml/schema/vehicle_model.schema.yaml`` provided in the Chrono data directory: 

\include data/yaml/schema/vehicle_model.schema.yaml

