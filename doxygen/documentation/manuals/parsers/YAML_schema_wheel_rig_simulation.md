YAML schema for Chrono::Vehicle single-wheel test rig simulation specification {#YAML_schema_wheel_rig_simulation}
========================================

A Chrono YAML wheel test rig simulation file defines a simulation of a single wheel assembly on a test rig
(@ref chrono::vehicle::ChWheelTestRig "ChWheelTestRig"), running on a patch of rigid or deformable terrain.
The wheel assembly is either a Chrono::Vehicle wheel and tire (optionally attached to a suspension) or a wheel assembly
provided by the application.

Paralleling a Chrono FSI simulation, a wheel test rig simulation combines independent specifications of the rig
mechanism (the "multibody" side) and of the terrain, so that the same rig can be tested on different terrains and
with different test scenarios:

| File | Content | Schema |
|------|---------|--------|
| simulation | test scenario, coupling, simulation, output, plots, and visualization settings | this page |
| rig model | wheel assembly, rig properties, and a reference to the MBS solver file | [wheel test rig model schema](@ref YAML_schema_wheel_rig_model) |
| MBS solver | multibody integrator and solver | [MBS solver schema](@ref YAML_schema_mbs_solver) |
| terrain | terrain type, patch, and parameters; for CRM terrain, a reference to the SPH solver file | [wheel test rig terrain schema](@ref YAML_schema_wheel_rig_terrain) |
| SPH solver | SPH solver, CRM terrain only | [FSI-SPH solver schema](@ref YAML_schema_fsisph_solver) |

The simulation file consists of the following main objects:
- [required] The Chrono version (`chrono-version`) that is compatible with the YAML specification.
  This is a string of the form `M.m` (major.minor) or `M.m.p` (major-minor-patch), although only the two fields are verified for compatibility.
- [required] The simulation `type`, which must be `WHEEL_RIG` here.
- [required] The `rig` entry, the path to the rig model specification file.
- [required] The `terrain` entry, the path to the terrain specification file.
- [required] The `test` object, which specifies the test scenario.
- [required] The `simulation` object, which specifies how the simulation is to be performed.
- [optional] The `coupling` object, which specifies wheel-terrain coupling settings (CRM terrain only).
- [optional] The `output` object, which specifies output options.
- [optional] The `plots` object, which specifies Gnuplot plots generated at the end of the simulation.
- [optional] The `visualization` object, which specifies run-time visualization settings.

## The single-wheel test rig

The test rig carries the wheel assembly on a rig chassis that is free to move vertically relative to a carrier body,
which in turn moves along the global X axis. The rig mass is sized so that, under gravity, the total weight supported
by the wheel equals the specified normal load (see the [rig model schema](@ref YAML_schema_wheel_rig_model)).
The test rig assumes a Z-up world frame. The motion of the wheel is controlled through:
- a linear motor driving the carrier at the prescribed longitudinal speed,
- a rotational motor driving the wheel at the prescribed angular speed,
- for a wheel without a suspension, a rotation of the wheel about the vertical axis, imposing the prescribed slip angle,
- for a wheel with a steerable suspension, a steering actuator acting on the steering mechanism of the assembly.

A motor whose function is not specified is not actuated; for example, a driven wheel is obtained by specifying only
the angular speed, and a pulled (towed) wheel by specifying only the longitudinal speed.

The terrain patch is placed by the rig relative to the wheel: it starts 2 wheel radii behind the initial wheel
location and extends along the positive X axis.

## Rig and terrain specification

The `rig` entry (required) must specify the path (relative to the location of this YAML simulation specification
file) to the YAML file with a rig model specification, which must follow the
[wheel test rig model schema](@ref YAML_schema_wheel_rig_model).

The `terrain` entry (required) must specify the path (relative to the location of this YAML simulation specification
file) to the YAML file with a terrain specification, which must follow the
[wheel test rig terrain schema](@ref YAML_schema_wheel_rig_terrain).

## Test scenario

The `test` object specifies the operation mode of the rig and the motion functions. Motion functions are functions
of time (with the same specification as functions in the [MBS model schema](@ref YAML_schema_mbs_model)), applied
after the time delay.

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `mode` | Rig operation mode | enum | `SUSPEND`,`DROP`,`TEST` | Yes | -- |
| `angle_degrees` | Whether angles are specified in degrees (true) or radians (false) | boolean | -- | No | `true` |
| `drop_speed` | Wheel lowering speed (m/s), `TEST` mode only | double | -- | No | 0.1 |
| `time_delay` | Settling time (s) before the motion functions are applied, measured from the end of the drop phase | double | -- | No | 0 |
| `longitudinal_speed` | Carrier longitudinal speed (m/s) as a function of time | function | -- | No | not actuated |
| `angular_speed` | Wheel angular speed (rad/s) as a function of time | function | -- | No | not actuated |
| `slip_angle` | Wheel slip angle as a function of time, wheel without a suspension only | function | -- | No | 0 |
| `steering` | Steering input, wheel assembly with a steering actuator only | object | -- | No | -- |
| `longitudinal_slip` | Constant longitudinal slip (overrides `longitudinal_speed` and `angular_speed`) | object | -- | No | -- |

The `angle_degrees` setting applies to the values of the `slip_angle` function and of the `steering` function with an
`ANGLE` input.

@anchor YAML_wheel_rig_modes
#### Operation modes

- `SUSPEND`: the wheel is suspended above the terrain, with a locked spindle.
- `DROP`: the wheel is released and falls freely onto the terrain.
- `TEST`: the wheel is lowered onto the terrain at `drop_speed`, then the motion functions are applied after
  `time_delay`. Without a suspension, the drop phase ends when the wheel reaches the terrain. With a suspension, the
  suspension travel is held at its static equilibrium under the normal load and lowering continues until the tire
  carries the normal load; the suspension is then released, so that releasing it does not change the load on the
  wheel. A slow drop speed avoids impacting the terrain, which matters in particular for deformable terrain.

Rig measurements (slip, camber, and steering angles, terrain forces, drawbar pull, drive torque) are enabled in `TEST`
mode once the time delay has elapsed; until then, they are reported as zero.

#### Test scenarios

Typical test scenarios are obtained through the following combinations of motion functions:

| Scenario | Specification |
|----------|---------------|
| driven wheel | `angular_speed` only |
| pulled wheel | `longitudinal_speed` only |
| immobilized wheel | `longitudinal_speed` and `angular_speed`, both constant 0 |
| fully prescribed | `longitudinal_speed`, `angular_speed`, and `slip_angle` (or `steering`, with a suspension) |
| constant slip | `longitudinal_slip` |

The `longitudinal_slip` object, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `value` | Longitudinal slip | double | -- | Yes | -- |
| `base_speed` | Base longitudinal speed (m/s) used to enforce the slip value | double | -- | No | 1.0 |

A positive slip value indicates a spinning wheel, a negative value a sliding (skidding) wheel, and a value of -1
a wheel sliding without rotation.

@anchor YAML_wheel_rig_steering
#### Slip angle and steering

For a wheel without a suspension, the slip angle is imposed directly through the `slip_angle` function, by rotating
the wheel about the vertical axis.

With a suspension, the suspension geometry dictates the wheel orientation, so `slip_angle` is ignored. A steerable
suspension can instead be steered through the `steering` object, which specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `input` | Type of steering input | enum | `ANGLE`,`ACTUATOR` | Yes | -- |
| `function` | Steering angle or actuator coordinate as a function of time | function | -- | Yes | -- |
| `feedback` | Feedback correction of the actuator coordinate, `ANGLE` input only | object | -- | No | -- |

The wheel is steered at the correct place, about the kingpin axis, through the steering mechanism of the wheel
assembly. For a Chrono::Vehicle steerable suspension, the rig creates a steering rack to which the suspension tierods
are attached; for an application-provided wheel assembly, the assembly provides its own steering actuator.

- `ACTUATOR`: the function specifies the steering actuator coordinate, which is specific to the steering mechanism.
  For a Chrono::Vehicle steerable suspension, this is the rack displacement (m) along the rig lateral axis.
- `ANGLE`: the function specifies the wheel steering angle, positive to the left. The steering angle is converted to
  the actuator coordinate through a kinematic steering map, computed at initialization by sweeping the steering
  mechanism over its range at the loaded configuration of the suspension. The resulting actuator motion is a
  feedforward input, so that bump and compliance steer are retained.

The `feedback` object, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `enabled` | Whether to enable the feedback correction | boolean | -- | No | `false` |
| `time_constant` | Time constant of the feedback correction (s) | double | -- | No | 0.05 |

With feedback enabled, an integral correction of the actuator coordinate drives the actual steering angle to the
specified one. Note that this cancels bump and compliance steer.

@anchor YAML_wheel_rig_steering_reference
#### Steering and slip angle reference

Steering angles are measured relative to the *loaded configuration* of the suspension: the configuration at static
equilibrium under the normal load, with the steering actuator at zero. A zero steering angle therefore retains the
static toe of the suspension in that configuration. This toe angle is in general different from the design toe of
the suspension, because of bump steer: the loaded configuration is usually at a different ride height than the
design configuration. As a consequence, a wheel with a suspension may run at a non-zero slip angle even with a zero
steering input.

The two angles reported by the rig differ accordingly:
- the slip angle is the wheel heading relative to the direction of travel and includes the loaded toe;
- the steering angle is relative to the loaded configuration and does not include it.

For a steerable suspension, the slip angle is the negative of the sum of the steering angle and the loaded toe angle.

During a test, the ride height can change further, for example as lateral tire forces load the suspension through
its geometry (jacking). Without feedback, the resulting bump steer changes the actual steering angle; feedback
cancels this bump steer, but not the static toe of the loaded configuration.

## Wheel-terrain coupling

The optional `coupling` object specifies settings for the coupling between the wheel and the terrain. Currently, it
only applies to CRM terrain, through its `crm` object:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `active_domain` | CRM active domain attached to the wheel hub | object | -- | No | -- |

The `active_domain` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Active domain type | enum | `NONE`,`AUTO`,`BOX` | Yes | `NONE` |
| `size` | Box dimensions, `BOX` type only | array[3] | -- | No | -- |

With `NONE`, the entire SPH domain is always active. With `AUTO`, the active domain is a box of estimated size
(2.5 times the bounding box of the wheel assembly). With `BOX`, the active domain is a box of the specified size,
centered at the hub. These settings are ignored for other terrain types.

## Simulation options

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `time_step` | Rig step (s), the interval over which the rig is advanced at each call | double | -- | No | MBS solver step |
| `end_time` | Total simulation time in seconds | double | -- | No | -1 for infinite simulation |
| `stop_at_patch_end` | Whether to end the simulation when the wheel reaches either end of the terrain patch | boolean | -- | No | `false` |
| `enforce_realtime` | Whether to enforce real-time simulation | boolean | -- | No | false |
| `gravity` | Gravitational acceleration vector [0, 0, -g] | array[3] | -- | No | [0, 0, -9.8] |
| `num_threads` | Number of threads used by the parallel components | object | -- | No | -- |

The rig step `time_step` is distinct from the step sizes of the solvers:
- With CRM terrain, it is the FSI coupling step. Over each rig step, the multibody system is advanced with the step
  from the MBS solver file and the SPH system with the step from the SPH solver file; the two exchange data at the end
  of the rig step.
- For other terrain types, the multibody system is advanced over each rig step with the step from the MBS solver file.

With `stop_at_patch_end` enabled, the simulation ends (before `end_time`) once the wheel hub is within one wheel
radius of either end of the terrain patch. This setting is ignored for granular terrain, which uses a moving patch.

The test rig assumes a Z-up world frame: only gravity vectors of the form [0, 0, -g], with g >= 0, are accepted.
The magnitude g is used, together with the normal load, to size the rig mass.

The `num_threads` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `chrono` | Number of threads used by Chrono (e.g., SCM ray casting) | integer | -- | No | 1 |
| `collision` | Number of threads used by the collision detection system | integer | -- | No | 1 |
| `eigen` | Number of threads used by Eigen | integer | -- | No | 1 |
| `pardiso` | Number of threads used by the PARDISO solver | integer | -- | No | 1 |

The `pardiso` entry is used only when the MBS solver file selects the `PARDISO` solver type.

## Output options

If the `output` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `format` | Output DB format | enum | `NONE`,`ASCII`,`HDF5`  | Yes | -- |
| `mode` | Output mode (one file per output frame, or a single time-series file) | enum | `FRAMES`,`SERIES`  | No | `FRAMES` |
| `fps` | Output frequency (FPS or Hz); also the sampling frequency of the data recorded for plots | double | -- | No | 100 |

Note that `format` is required whenever the `output` key is present.
`HDF5` output silently falls back to `NONE` in a build without HDF5 support.

Output is written in a subdirectory of the output directory, named after the simulation specification file.
The output database contains the rig output channels listed below, each with its name and units.

#### Output channels

All quantities are in SI units, with angles in radians. Forces and moments are expressed in the global frame and
reported at the wheel hub. Quantities marked as measurements are zero until rig measurements are enabled (see
[operation modes](@ref YAML_wheel_rig_modes)).

| Channel | Description | Units |
|---------|-------------|-------|
| `measuring` | 1 if rig measurements are enabled, 0 otherwise | -- |
| `long_slip` | Longitudinal slip, (r ω - v) / abs(v), with v the horizontal hub speed (measurement) | -- |
| `slip_angle` | Slip angle, from the hub orientation (measurement) | rad |
| `camber_angle` | Camber angle, from the hub orientation (measurement) | rad |
| `steering_angle` | Steering angle, relative to the loaded configuration (measurement) | rad |
| `steering_actuator` | Steering actuator coordinate (0 without a steering actuator) | actuator specific |
| `wheel_force_x`, `wheel_force_y`, `wheel_force_z` | Terrain force on the wheel (measurement) | N |
| `wheel_moment_x`, `wheel_moment_y`, `wheel_moment_z` | Terrain moment on the wheel (measurement) | N.m |
| `drawbar_pull` | Drawbar pull, from the carrier motor force (measurement; 0 if the carrier is not actuated) | N |
| `drive_torque` | Wheel drive torque, from the wheel motor (measurement; 0 if the wheel is not actuated) | N.m |
| `wheel_angular_speed` | Wheel angular speed about its spin axis | rad/s |
| `carrier_speed` | Carrier longitudinal speed | m/s |
| `carrier_position` | Carrier longitudinal position | m |
| `hub_position_x`, `hub_position_y`, `hub_position_z` | Wheel hub position | m |
| `sinkage` | Wheel sinkage below the initial terrain surface, from the hub height | m |
| `suspension_travel` | Vertical hub displacement relative to the rig chassis, measured from the design configuration (wheel assemblies with a suspension only) | m |

An application-provided wheel assembly may append its own channels to this list.

## Plots

If the `plots` key is present, Gnuplot plots of the output channels are generated at the end of the simulation.
Plot generation requires the Chrono::Postprocess module and a Gnuplot installation.

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `output` | Plot output: interactive Gnuplot windows, PNG images, or PDF files | enum | `WINDOW`,`PNG`,`PDF` | No | `PNG` |
| `measuring_only` | Whether to plot only data recorded while rig measurements are enabled | boolean | -- | No | `true` |
| `figures` | List of figures | array | -- | Yes | -- |

Each entry in `figures` is either a channel name, which plots that channel as a function of time, or an object with
the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `y` | Ordinate channel or list of channels | string or array | -- | Yes | -- |
| `x` | Abscissa channel | string | -- | No | time |
| `title` | Figure title | string | -- | No | -- |
| `y_range` | Ordinate range [min, max] | array[2] | -- | No | automatic |

Channels are recorded at the output frequency (`output.fps`), even if no output database is written
(`output.format: NONE`). For each figure, a Gnuplot script (`plot_<n>.gpl`) and, for PNG or PDF output, the plot
(`plot_<n>.png` or `plot_<n>.pdf`) are written to the output directory. Unknown channel names are reported and skipped.

## Visualization options

If the `visualization` key is present, it must specify a YAML object with the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `type` | Type of wheel assembly visualization shapes | enum | `NONE`,`PRIMITIVES`,`MODEL_FILE`,`COLLISION`  | No | `NONE` |
| `render_fps` | Rendering frequency (FPS or Hz) | double | -- | No | 120 |
| `enable_shadows` | Turn on shadow rendering | boolean | -- | No | `true` |
| `camera` | Camera settings | object | -- | No | -- |
| `output` | Image output settings | object | -- | No | -- |

The camera follows the wheel hub. The `camera` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `vertical` | Vertical direction (camera "up"); the rig requires Z | enum | `Z`  | No | `Z` |
| `offset` | Camera location relative to the wheel hub position (in the world frame) | array[3] | -- | No | [2.0, 2.5, 1.0] |
| `target_offset` | Camera look-at point relative to the wheel hub position (in the world frame) | array[3] | -- | No | [0, 0.25, -0.25] |

The `output` key, if present, specifies the following properties:

| Property | Description | Type | Available Values | Required | Default |
|----------|-------------|------|------------------|----------|---------|
| `save_images` | Whether to save rendered frames as image files | boolean | -- | No | `false` |
| `image_type` | Image file type, specified through the file extension | string | -- | No | `bmp` |

Run-time visualization is enabled only if the `visualization` key is present *and* specifies a `type` other than
`NONE`. Rendering of SPH particles and BCE markers (CRM terrain) is specified in the terrain file.
For deformable terrain, `COLLISION` shows the tire geometry that interacts with the terrain.

## Running a wheel test rig simulation

A wheel test rig simulation can be run with the `run_chrono` program (see the
[YAML parser overview](@ref YAML_parser_overview)), for example:

`./run_chrono -s ../data/yaml/vehicle/wheel_rig/wheel_suspension_rig_rigid.yaml`

`run_chrono` can only create wheel assemblies specified through Chrono::Vehicle JSON files. A rig model with an
application-provided (`CUSTOM`) wheel assembly requires a program that registers the corresponding factory with the
@ref chrono::parsers::ChParserWheelRigYAML "ChParserWheelRigYAML" parser before creating the rig (see the
[rig model schema](@ref YAML_schema_wheel_rig_model)). The `demo_YAML_wheel_rig` demo program illustrates this and
allows selecting one of several wheel test rig specification files at run time.

## Example

Below is an example of a wheel test rig simulation configuration, for a wheel with a steerable suspension on rigid terrain:

\include data/yaml/vehicle/wheel_rig/wheel_suspension_rig_rigid.yaml

## YAML schema

The YAML wheel test rig simulation specification file must follow the ``data/yaml/schema/wheel_rig_simulation.schema.yaml`` provided in the Chrono data directory:

\include data/yaml/schema/wheel_rig_simulation.schema.yaml
