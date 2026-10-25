YAML schema for Chrono::Vehicle static tire test rig simulation specification {#YAML_schema_tire_static_rig_simulation}
========================================

A Chrono YAML static tire test rig simulation file defines a simulation of a Chrono::Vehicle wheel and tire on a static
tire test rig (@ref chrono::vehicle::ChTireStaticTestRig "ChTireStaticTestRig"). The wheel spindle is lowered onto a
rigid plate until the tire carries a nominal radial load and, depending on the test mode, the plate is then translated
or rotated to load the tire longitudinally, laterally, or torsionally. Such tests characterize the static stiffness of
a tire in each direction.

A static tire test rig simulation uses the following files:

| File | Content | Schema |
|------|---------|--------|
| simulation | rig plate, test scenario, simulation, output, plots, and visualization settings | this page |
| rig model | wheel and tire, and a reference to the MBS solver file | [wheel test rig model schema](@ref YAML_schema_wheel_rig_model) |
| MBS solver | multibody integrator and solver | [MBS solver schema](@ref YAML_schema_mbs_solver) |

The rig model file follows the same schema as the rig model of a
[single-wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation), so that the same wheel and tire
specification can be used with both test rigs.

The simulation file consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The simulation `type`, which must be `TIRE_STATIC_RIG` here.
- [required] The `rig` entry, the path to the rig model specification file.
- [optional] The `plate` object, which defines the contact material of the rig plate.
- [required] The `test` object, which defines the test scenario.
- [required] The `simulation` object, which defines the simulation settings.
- [optional] The `output` object, which defines the output settings.
- [optional] The `plots` object, which defines plots of the output channels.
- [optional] The `visualization` object, which defines the run-time visualization settings.

## Rig model specification

The `rig` entry (required) must specify the path (relative to the location of this YAML file) to a rig model file,
which must follow the [wheel test rig model schema](@ref YAML_schema_wheel_rig_model), with the following restrictions:
- the model must be of `VEHICLE` type (application-provided wheel assemblies are not supported);
- a suspension (`suspension_json`) and the rig camber angle (`rig.camber_angle`) are ignored;
- the rig normal load (`rig.normal_load`) is the default nominal radial load (see `test.radial_load` below);
- handling (force-element) tires cannot be used, as they have no collision geometry.

Rigid tires and deformable tires (FEA and MB tires) can be tested. Deformable tires require the `SMC` contact method;
their contact surface is specified through the `tire.contact_surface` object of the rig model file.

The integrator time step of the MBS solver is the step size for the multibody system and the tire dynamics.

## Rig plate

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `friction` | Coefficient of friction | double | -- | No | 0.8 |
| `restitution` | Coefficient of restitution | double | -- | No | 0 |
| `Young_modulus` | Young's modulus (Pa), used with an `SMC` contact method | double | -- | No | 2e7 |

## Test scenario

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `mode` | Test mode | enum | `SUSPEND`,`TEST_R`,`TEST_X`,`TEST_Y`,`TEST_Z` | Yes | -- |
| `radial_load` | Nominal radial load (N); overrides `rig.normal_load` from the rig model file | double | -- | No | rig normal load |
| `drop_speed` | Spindle speed (m/s) before the tire contacts the plate | double | -- | No | 1 |
| `radial_speed` | Spindle speed (m/s) during the radial loading phase | double | -- | No | 0.001 |
| `longitudinal_speed` | Plate speed (m/s) in a longitudinal test | double | -- | No | 0.001 |
| `lateral_speed` | Plate speed (m/s) in a lateral test | double | -- | No | 0.001 |
| `torsional_speed` | Plate angular speed (rad/s) in a torsional test | double | -- | No | 0.01 |
| `transition_delay` | Time delay (s) after a change of test phase | double | -- | No | 0.1 |

The test modes are:
- `SUSPEND`: the wheel spindle is fixed above the plate (no test).
- `TEST_R`: radial (vertical compression) test.
- `TEST_X`: longitudinal test, with the plate translated in the x direction.
- `TEST_Y`: lateral test, with the plate translated in the y direction.
- `TEST_Z`: torsional test, with the plate rotated about the vertical axis.

In all test modes other than `SUSPEND`, the wheel spindle is first lowered at the drop speed until the tire contacts
the plate. The spindle is then lowered at the radial loading speed until the radial load reaches the nominal radial
load. A `TEST_R` test ends at that point. In the other test modes, the spindle is then held in place and the plate is
translated or rotated at the corresponding loading speed until the end of the simulation.

After each change of test phase, the rig waits for `transition_delay` before checking for the next phase change. The
*measuring phase* of a test is the radial loading phase for a `TEST_R` test and the plate displacement phase for the
other tests, in both cases after the transition delay.

## Simulation options

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `end_time` | Total simulation time in seconds | double | -- | No | -1 for infinite simulation |
| `enforce_realtime` | Whether to enforce real-time simulation | boolean | -- | No | false |
| `gravity` | Gravitational acceleration vector [0, 0, -g] | array[3] | -- | No | [0, 0, -9.8] |
| `num_threads` | Number of threads used by the parallel components | object | -- | No | -- |

A simulation ends at `end_time` or at the end of a `TEST_R` test, whichever comes first.
The test rig assumes a Z-up world frame: only gravity vectors of the form [0, 0, -g], with g >= 0, are accepted.
The `num_threads` object is the same as for a [single-wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation).

## Output options

The `output` object is the same as for a [single-wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation).
Output is written in a subdirectory of the output directory, named after the simulation specification file.
The output database contains the rig output channels listed below, each with its name and units.

#### Output channels

All quantities are in SI units, with angles in radians. Deflections are measured from the configuration at first
tire-plate contact and are zero before that.

| Channel | Description | Units |
|---------|-------------|-------|
| `measuring` | 1 during the measuring phase of the test, 0 otherwise | -- |
| `state` | Rig state (0: fixed, 1: dropping, 2: compressing, 3: displacing, 4: done) | -- |
| `deflection` | Tire deflection for the current test mode | m (rad for `TEST_Z`) |
| `load` | Tire load for the current test mode | N (N.m for `TEST_Z`) |
| `radial_deflection`, `radial_load` | Radial deflection and load | m, N |
| `longitudinal_deflection`, `longitudinal_load` | Plate displacement and load in the x direction | m, N |
| `lateral_deflection`, `lateral_load` | Plate displacement and load in the y direction | m, N |
| `torsional_deflection`, `torsional_load` | Plate rotation and torque about the vertical axis | rad, N.m |
| `spindle_height` | Height of the wheel spindle | m |

## Plots

The `plots` object is the same as for a [single-wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation),
using the output channels listed above. With `measuring_only` enabled (the default), only data recorded during the
measuring phase of the test is plotted. A plot of `load` as a function of `deflection` shows the tire stiffness
characteristic for the current test mode.

## Visualization options

The `visualization` object is the same as for a [single-wheel test rig simulation](@ref YAML_schema_wheel_rig_simulation),
except that `type` specifies the tire visualization shapes and that the camera follows the wheel, with a default
`offset` of [1.0, 2.5, 0.5] and a default `target_offset` of [0, 0, 0].

## Running a static tire test rig simulation

A static tire test rig simulation can be run with the `run_chrono` program (see the
[YAML parser overview](@ref YAML_parser_overview)), for example:

`./run_chrono -s ../data/yaml/vehicle/tire_static_rig/tire_static_rig_hmmwv_mbtire.yaml`

The `demo_YAML_tire_static_rig` demo program allows selecting one of several tire models at run time.

## Example

Below is an example of a static tire test rig simulation configuration, for a radial test of an MB tire:

\include data/yaml/vehicle/tire_static_rig/tire_static_rig_hmmwv_mbtire.yaml

with the following rig model:

\include data/yaml/vehicle/tire_static_rig/rig_model_hmmwv_mbtire.yaml

## YAML schema

The YAML static tire test rig simulation specification file must follow the ``data/yaml/schema/tire_static_rig_simulation.schema.yaml`` provided in the Chrono data directory:

\include data/yaml/schema/tire_static_rig_simulation.schema.yaml
