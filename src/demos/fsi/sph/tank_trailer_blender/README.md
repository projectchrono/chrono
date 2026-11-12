# Tank trailer sloshing demo and Blender rendering pipeline

This directory documents `demo_FSI-SPH_TankTrailer` (source in the parent directory) and holds the scripts that turn its output into rendered videos with [splashsurf](https://github.com/InteractiveComputerGraphics/splashsurf), [Blender](https://www.blender.org) (Cycles) and [ffmpeg](https://ffmpeg.org).

## The demo

`demo_FSI-SPH_TankTrailer` simulates water sloshing in a partially filled horizontal cylindrical tank, with Chrono::FSI-SPH for the fluid and Chrono::Vehicle for the vehicle. The tank (inner radius 0.33, inner length 1.40, filled to a height of 0.33) is welded to a UTV trailer towed by a Polaris UTV.

### Requirements

- Chrono::FSI with the SPH fluid solver (requires CUDA) and Chrono::Vehicle. The demo is built only when both modules are enabled.
- Chrono::VSG (optional) for run-time visualization.
- Chrono::PostProcess (optional) for the `--blender` output.

### Modes

- **vehicle** (default): the Polaris, starting at (-40, -15), holds its brakes for the first second while the system settles, then accelerates toward the target speed (35 km/h by default) under a path-follower driver. After a straight run of 30, it performs a double lane change to the left (lane offset 3, transitions of length 10, length 5 in the offset lane), returns to the original lane and brakes hard once its reference point passes x = 25. The vehicle comes to rest at about t = 11 and the fluid keeps sloshing until the end of the simulation (13 by default). The terrain is a rigid flat patch of 300 x 60.
- **prescribed**: the tank alone follows a prescribed motion with the same phases (acceleration, lane change, hard braking), with no vehicle or terrain. The simulation ends at 7 by default.

### Solution method

The SPH problem is solved in a frame that translates with the tank, so the computational domain stays small and fixed however far the vehicle travels:

- The tank body (A) is part of the vehicle multibody system.
- A mirror body (B) carries the BCE markers of the tank walls (5 layers). It stays at the origin with the orientation, angular velocity and angular acceleration of the tank and no translational motion.
- In this non-inertial frame, the fluid is subject to the effective gravity g - a, where a is the acceleration of the tank. It is updated every step.
- The fluid force and torque on the mirror body are applied to the tank body.

### Command-line options

| Option | Default | Description |
|---|---|---|
| `--mode` | `vehicle` | `vehicle` or `prescribed` |
| `--spacing` | `0.02` | initial particle spacing |
| `--step_size` | `1e-4` | integration step size (fluid and multibody) |
| `--t_end` | 13 (vehicle), 7 (prescribed) | simulation end time |
| `--speed` | `35` | vehicle target speed, in km/h (vehicle mode) |
| `--output_fps` | `100` | frequency of the `results.txt` output |
| `--quiet` | off | disable the terminal output |
| `--blender` | off | save data for rendering in Blender (see below) |
| `--blender_fps` | `50` | frequency of the Blender output |
| `--no_vis` | off | disable run-time visualization |
| `--view` | `tank` | run-time view: `tank` (camera following the tank) or `vehicle` (vehicle chase camera, vehicle mode only) |
| `--render_fps` | `100` | run-time rendering frequency |
| `--snapshots` | off | save the run-time rendered images |

The step size must satisfy the CFL condition for the chosen spacing: `1e-4` works with the default spacing of `0.02`, while a spacing of `0.015` requires `5e-5`.

### Default run

Run with no arguments, the demo simulates the vehicle mode with a spacing of 0.02 (26,910 SPH particles) and a step size of 1e-4 for 13. On an RTX 2060 SUPER, it runs in about 15 minutes without run-time visualization (real-time factor about 66). No fluid particles leave the tank: at most a few of them move into the tank wall, less than one spacing deep, and none goes past the BCE layers.

### Run-time visualization

With Chrono::VSG, the demo opens a VSG window showing the vehicle, the trailer and the tank, with the SPH particles drawn in the tank and colored by their speed relative to the tank. The particles, which live in the frame translating with the tank, are displayed at the tank location through `ChSphVisualizationVSG::SetMarkersFrame`. The rigid BCE markers can be shown from the GUI; their outer layers lie outside the drawn tank shell.

### Output

The output goes to `DEMO_OUTPUT/FSI_Tank_Trailer/<mode>_d<spacing>_h<step_size>/` (relative to the working directory):

- `results.txt`: one line per output frame with the columns time, tank position (x, y, z), tank acceleration (ax, ay, az), tank speed, fluid force on the tank (Fx, Fy, Fz) and fluid center of mass in the tank frame (com_x, com_y, com_z).
- `snapshots/img_NNNNN.png` with `--snapshots`.
- With `--blender`:
  - `blender/`: the scene (vehicle, trailer, tank, terrain) written by `ChBlender`, with `exported.assets.py` and one `output/stateNNNNN.py` per frame;
  - `particles/fluid_NNNNN.json`: positions of the SPH particles in the frame translating with the tank;
  - `fluid_offset.txt`: for each frame, the frame index, the time and the translation of that frame (tank position minus mirror location), which places the fluid in the world.

## Rendering with Blender

The pipeline has three steps: reconstruct the fluid surface for each frame, render the frames in Blender, and assemble the videos.

### Requirements

- Python 3 with numpy (for `make_fluid_surfaces.py`).
- splashsurf, installed with `cargo install splashsurf` (tested with version 0.11). It must be on the `PATH`, or its executable given in the `SPLASHSURF` environment variable.
- Blender 5.x (tested with 5.1). The render script loads the Chrono::Blender add-on from `src/importer_blender/for_blender_5.0/chrono_import.py` in the same source tree; set `CHRONO_BLENDER_IMPORTER` to use another copy.
- ffmpeg.
- A GPU supported by Cycles (OptiX or CUDA) is strongly recommended; the script falls back to the CPU otherwise.

### 1. Run the demo with Blender output

```
demo_FSI-SPH_TankTrailer --blender --no_vis --spacing 0.015 --step_size 5e-5
```

The finer spacing gives a better fluid surface. At this resolution the run takes about 42 minutes on an RTX 2060 SUPER and writes about 1.5 GB (650 frames at the default `--blender_fps 50`).

### 2. Reconstruct the fluid surfaces

```
python make_fluid_surfaces.py <run_dir> --spacing 0.015
```

where `<run_dir>` is the demo output directory (e.g. `DEMO_OUTPUT/FSI_Tank_Trailer/vehicle_d0.015_h5e-05`) and `--spacing` is the spacing of the run. For each frame, the script drops the few particles lodged in the tank walls (more than half a spacing outside the inner surface, using the tank orientation from the Blender state file), writes the remaining ones to `particles_filtered/`, and runs splashsurf to write the surface mesh to `surfaces/fluid_NNNNN.ply`. Options: `--first`, `--last` (frame range), `--jobs` (frames processed concurrently, default 4), `--threads` (splashsurf threads per frame, default 3), and the splashsurf parameters `--smoothing-length`, `--cube-size`, `--threshold` and `--smoothing-iters`.

### 3. Render the frames

```
blender -b -P render_tank_trailer.py -- <run_dir> --camera chase
blender -b -P render_tank_trailer.py -- <run_dir> --camera side
```

The script imports the scene with the Chrono::Blender add-on, replaces the run-time materials (thin glass for the tank, water with absorption for the fluid, asphalt for the terrain), and loads the fluid surface of each frame into the tank. Two cameras follow the tank along a smoothed path: `chase` (behind and to the right of the trailer) and `side`. Unless `--no-cues` is given, lane markings and traffic cones along the double lane change are added, so that the motion of the vehicle is visible. Frames are written to `<run_dir>/render/<camera>_NNNNN.png`.

| Option | Default | Description |
|---|---|---|
| `--camera` | `chase` | `chase` or `side` |
| `--first`, `--last`, `--step` | all frames | frame range |
| `--frames` | | render only the listed frames (e.g. stills for a quick check) |
| `--samples` | `128` | Cycles samples per pixel |
| `--res` | `1920 1080` | image resolution |
| `--out-dir` | `<run_dir>/render` | output directory |
| `--no-cues` | off | no lane markings and cones |
| `--save-blend` | off | also save the scene (at the first frame) to `<run_dir>/tank_trailer.blend` |

At the default settings, a frame takes about 13 s on an RTX 2060 SUPER (about 2.5 hours per camera for 650 frames). A rendering interrupted by a crash or a reboot can be resumed with `--first`.

### 4. Assemble the videos

```
ffmpeg -framerate 50 -i <run_dir>/render/chase_%05d.png -c:v libx264 -crf 18 -pix_fmt yuv420p tank_trailer_chase.mp4
```

and similarly for the side camera. The frame rate should match `--blender_fps` of the run for real-time playback.

### Limitations

- The rendering pipeline supports the vehicle mode only: the prescribed mode has no terrain, and the render script stops with an error.
- The scripts repeat values from the demo source and must be updated if those change: the tank dimensions and the mirror location (`make_fluid_surfaces.py`), and the path geometry used to place the lane markings and cones (`render_tank_trailer.py`). The scripts also rely on the names of the `tank` body and of the terrain patch (`patch_1`) in the Blender output.
- `make_fluid_surfaces.py --spacing` must match the spacing of the run.
