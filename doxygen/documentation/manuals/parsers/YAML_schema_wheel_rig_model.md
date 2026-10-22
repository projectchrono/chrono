YAML schema for Chrono::Vehicle single-wheel test rig model specification {#YAML_schema_wheel_rig_model}
========================================

A Chrono YAML wheel test rig model file defines the rig mechanism of a
[wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation): the wheel assembly under test, the rig
properties, and the multibody solver. It is the "multibody" side of a wheel test rig simulation, independent of the
terrain and of the test scenario. It consists of the following objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The `model` object, which defines the wheel assembly.
- [optional] The `rig` object, which defines the test rig properties.
- [required] The `solver` entry, the path to the MBS solver specification file.

## Wheel assembly specification

The wheel assembly is one of the following types:
- `VEHICLE`: a Chrono::Vehicle wheel and tire, optionally attached to a Chrono::Vehicle suspension, all specified
  through Chrono::Vehicle JSON specification files.
- `CUSTOM`: a wheel assembly provided by the application, for example a wheel or wheel-suspension assembly of a robot
  or vehicle model built outside Chrono::Vehicle. The assembly is created by a factory function that the application
  registers with the YAML parser.

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Type of wheel assembly | enum | `VEHICLE`,`CUSTOM` | No | `VEHICLE` |
| `name` | Name of the model | string | -- | No | empty string |
| `angle_degrees` | Whether angles in this file (`rig.camber_angle`) are specified in degrees (true) or radians (false) | boolean | -- | No | `true` |
| `data_path` | Location of the JSON data files referenced below | object | -- | No | absolute paths |
| `wheel_json` | JSON file for the wheel subsystem | string | -- | Yes, `VEHICLE` type | -- |
| `tire_json` | JSON file for the tire subsystem | string | -- | Yes, `VEHICLE` type | -- |
| `suspension_json` | JSON file for a suspension subsystem to which the wheel is attached | string | -- | No | no suspension |
| `tire` | Tire settings, `VEHICLE` type only | object | -- | No | -- |
| `assembly` | Name of the registered factory, `CUSTOM` type only | string | -- | Yes, `CUSTOM` type | -- |
| `parameters` | Parameters passed to the factory, `CUSTOM` type only | object | -- | No | -- |

The model name is also used to name the output files.

The `data_path` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Mode for data file location | enum | `ABSOLUTE`,`RELATIVE` | Yes | `ABSOLUTE` |
| `root` | Root of data files, relative to the location of this file | string | -- | No | `.` |

#### Chrono::Vehicle wheel assemblies

A `VEHICLE` wheel assembly consists of a wheel and a tire and, optionally, a suspension.

Any Chrono::Vehicle tire with a collision geometry (rigid tires, including rigid mesh tires) can be used on any
terrain type. Handling (force-element) tires, such as TMeasy or Pacejka tires, have no collision geometry and can only
be used on rigid terrain. FEA (deformable) tires are not supported.

The `tire` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `collision_type` | Tire-terrain collision detection type (handling tires only) | enum | `SINGLE_POINT`,`FOUR_POINTS`,`ENVELOPE` | No | `SINGLE_POINT` |

With a suspension (`suspension_json`), the wheel is attached to the spindle of the suspension and the suspension to
the rig chassis. The suspension geometry then dictates the camber and slip angles, so `rig.camber_angle` and the test
`slip_angle` are ignored. A steerable suspension can be steered through the test `steering` input (see
[slip angle and steering](@ref YAML_wheel_rig_steering)); the rig creates a steering rack to which the
suspension tierods are attached. For an independent suspension, the vertical travel of the other side (which is not
supported by the terrain) is locked. The mass of the suspension is included in the mass
supported by the wheel.

#### Custom wheel assemblies

A `CUSTOM` wheel assembly is created by a factory function registered by the application with the
@ref chrono::parsers::ChParserWheelRigYAML "ChParserWheelRigYAML" parser, under the name given by `assembly`.
The factory receives the Chrono system and the `parameters` object of this file, unchanged; the content of
`parameters` is defined by the application. The factory must return a wheel assembly derived from
@ref chrono::vehicle::ChWheelTestRig::WheelAssembly "ChWheelTestRig::WheelAssembly"; see its documentation for the
requirements on a wheel assembly implementation.

For example, a program could register a factory for a custom wheel as follows (where `MyWheel` is a class derived
from `ChWheelTestRig::WheelAssembly`):

~~~{.cpp}
parsers::ChParserWheelRigYAML parser(yaml_filename);

parser.RegisterWheelAssembly("My_wheel", [](ChSystem& sys, const YAML::Node& params) {
    auto wheel = chrono_types::make_shared<MyWheel>(sys);
    if (params["radius"])
        wheel->SetRadius(params["radius"].as<double>());
    return wheel;
});

auto sys = parser.CreateSystem();
auto rig = parser.CreateRig(*sys);
~~~

The factory must be registered before the rig is created. A rig model with a `CUSTOM` wheel assembly therefore cannot
be used with the `run_chrono` program, which does not register any factories.

## Rig properties

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `normal_load` | Normal load on the wheel (N) | double | -- | No | 1000 |
| `camber_angle` | Constant wheel camber angle (ignored with a suspension) | double | -- | No | 0 |

The normal load is typically the static wheel load of the vehicle the wheel belongs to. It can be overridden for a
given test through `test.normal_load` in the simulation file. The rig mass is set so that the total weight supported
by the wheel equals this load. With a suspension, the suspension is held at its static
equilibrium configuration under this load during the drop phase of a test, and released once the tire carries the
normal load. This *loaded configuration* is also the reference for steering angles (see
[steering and slip angle reference](@ref YAML_wheel_rig_steering_reference)).

## Solver specification

The `solver` entry (required) must specify the path (relative to the location of this YAML file) to the YAML file
with an MBS solver specification, which must follow the [MBS solver schema](@ref YAML_schema_mbs_solver). Its
integrator time step is the step size for the multibody system and the wheel assembly dynamics.

## Examples

Below is an example of a rig model with a Chrono::Vehicle wheel, tire, and steerable suspension:

\include data/yaml/vehicle/wheel_rig/rig_model_hmmwv_front_tmeasy.yaml

Below is an example of a rig model with a custom wheel assembly:

\include data/yaml/vehicle/wheel_rig/rig_model_viper.yaml

## YAML schema

The YAML wheel test rig model specification file must follow the ``data/yaml/schema/wheel_rig_model.schema.yaml`` provided in the Chrono data directory:

\include data/yaml/schema/wheel_rig_model.schema.yaml
