Coupling of a Chrono FEA cable with Chrono::FSI-SPH or OpenFOAM in a shaking tank
================================================================================

A flexible cable, modeled with 8 ANCF cable elements and anchored at the floor of a box of fluid, is excited by a
horizontal oscillation of the box. The box is periodic in x (the direction of the oscillation) and y, with a floor
and an open top (the fluid surface). With the Chrono::FSI-SPH fluid participant, this is the preCICE counterpart of
the shaking tank YAML model of `demo_YAML_fsi` (see `data/yaml/fsi/flexible_cable_shaking_tank`). The same problem
is also solved with an OpenFOAM (interFoam) fluid participant.

All combinations are run with the program `demo_PRECICE_flexible_cable_shaking_tank` (solid participant, and
Chrono::FSI-SPH fluid participant) and, for OpenFOAM, the OpenFOAM case in `fluid_openfoam`.

Problem
-------

- **Box motion:** X(t) = A (1 - cos(w t)) along x, with A = 0.08 and a frequency of 0.5 Hz (w = pi). The box
  velocity A w sin(w t) has zero mean, so the fluid does not drift relative to the box.
- **Frame of the box:** all participants are simulated in the (non-inertial) frame of the box. The box acceleration
  a(t) = A w^2 cos(w t) enters as a prescribed frame acceleration: each participant uses the effective gravity
  g - a(t) instead of the gravity g. For the cable, this is the base excitation; in the fluid, it drives the flow
  relative to the box.
- **Box:** 0.8 x 0.4 x 1.0 (x, y, z), filled with fluid up to its top; the cable (length 0.795) is at the center
  of the floor. With periodic sides and a uniform forcing, the fluid surface stays flat.
- **Fluid:** density 1000, kinematic viscosity 5e-3. The flow around the cable is laminar (Reynolds number of a few
  units), which both fluid participants resolve.
- **Duration:** 16 s (8 periods).

Participants
------------

- **Solid** (`solid_chrono`): a Chrono MBS model containing the FEA mesh of the cable (`mbs_model.yaml`):
  diameter 0.06, density 8000, Young's modulus 5e7 (first natural frequency of about 1 Hz in the fluid). The cable
  diameter is the width of the cable as seen by the SPH fluid (3 particle spacings for the STAR pattern of BCE
  markers), also used by OpenFOAM, so that both fluid participants and the solid participant see the same cable.
  - `mbs_participant_sph.yaml`: coupling with Chrono::FSI-SPH (sends node positions, velocities, and
    accelerations; receives nodal forces).
  - `mbs_participant_openfoam.yaml`: coupling with OpenFOAM (sends node displacements; receives nodal forces).
- **Fluid, Chrono::FSI-SPH** (`fluid_sph`): particle spacing 0.02, physical (laminar) viscosity, free surface at
  the top. The cable is represented with BCE markers placed around the cable nodes received from the solid
  participant.
- **Fluid, OpenFOAM** (`fluid_openfoam`): interFoam with the box filled with the fluid (the second phase required by
  interFoam does not appear) and a pressure boundary at the top (`top`), through which the fluid may enter or leave
  (the cable motion does not conserve the fluid volume exactly). The cable is a cylindrical wall patch (`cable`) of
  a body-fitted mesh, moved with the cable by mesh motion (`displacementLaplacian`); the box sides are cyclic
  patches. The frame acceleration is applied as the momentum source -rho a(t) (a coded source in
  `constant/fvOptions`), with the gravity kept vertical: interFoam folds gravity into the pressure, and a horizontal
  gravity component is incompatible with the periodic sides of the box. In a periodic box, both forms drive the same
  flow.

The frame acceleration (`frame_acceleration` in the `simulation` settings of the Chrono participant files, and the
source in `constant/fvOptions` of the OpenFOAM case) must be the same in all participants: the participants are
separate processes and nothing reconciles these settings for you.

preCICE configurations
----------------------

- `precice_config_sph.xml` (Chrono::FSI-SPH): serial explicit coupling with the fluid participant first and a time
  window equal to the SPH integration step size (1e-4). The solid participant runs second and reads the fluid
  forces at the end of the window. The participants exchange data through the current directory.
- `precice_config_openfoam.xml` (OpenFOAM): serial explicit coupling with a time window of 1e-3 (four steps of the
  solid participant), the OpenFOAM time step. The solid participant runs first in each time window, with the fluid
  forces of the previous window; OpenFOAM then follows the displacements of the current window, interpolated
  linearly in time, so that each OpenFOAM time step receives its own share of the cable motion. The participants run
  from `solid_chrono` and `fluid_openfoam` and exchange data through this directory.

Running with Chrono::FSI-SPH
----------------------------

Run the two participants in two different processes, from the same directory (e.g., the directory of the Chrono
executables):

```
demo_PRECICE_flexible_cable_shaking_tank -p Solid -f SPH
demo_PRECICE_flexible_cable_shaking_tank -p Fluid_SPH -f SPH
```

Other options: `--no_vis` (no run-time visualization), `--no_output` (no output), `--quiet` (no verbose
terminal output), `-c <file>` (another preCICE configuration file).

With output enabled (the default), the solid participant writes the trajectory of the top cable node (in the frame
of the box) to `DEMO_OUTPUT/PRECICE_Flexible_Cable_Shaking_Tank/SPH/results.txt` (time, x, y, z, displacement).

Running with OpenFOAM
---------------------

OpenFOAM runs on Linux only. Requirements:

- Chrono, built with the PRECICE and FSI modules enabled (the VSG module, optional, enables run-time visualization
  of the solid participant).
- OpenFOAM (openfoam.com distribution) and the OpenFOAM-preCICE adapter; see the preCICE
  [documentation](https://precice.org/adapter-openfoam-overview.html).

Run the two participants from two different terminals, using the `run.sh` script in each participant directory of
the Chrono data directory of the build tree (here, `<chrono_data_dir>`):

```
cd <chrono_data_dir>/precice/flexible_cable_shaking_tank/solid_chrono
./run.sh
```

```
cd <chrono_data_dir>/precice/flexible_cable_shaking_tank/fluid_openfoam
./run.sh             # or: ./run.sh -parallel (see system/decomposeParDict)
```

Do not run `demo_PRECICE_flexible_cable_shaking_tank -f OpenFOAM` from another directory: it locates its data files
relative to `solid_chrono`. Options given to the solid script are passed on (e.g., `./run.sh --no_vis`). The fluid
script generates the mesh (`blockMesh`) and runs interFoam.

Output:

- solid participant: trajectory of the top cable node in
  `solid_chrono/DEMO_OUTPUT/PRECICE_Flexible_Cable_Shaking_Tank/OPENFOAM/results.txt` (same format as above);
- fluid participant: OpenFOAM time directories in `fluid_openfoam` (at the first time step after every multiple of
  0.1 s).

To remove all generated files, run `./clean.sh` in this directory (or in each participant directory).
