Coupling of a Chrono FEA cable with Chrono::FSI-SPH or OpenFOAM
===============================================================

A flexible cable, modeled with 8 ANCF cable elements and anchored at the bottom of a container, is impacted by
a collapsing granular (CRM) or water (CFD) column. With the Chrono::FSI-SPH fluid participant, this is the preCICE
counterpart of `demo_FSI-SPH_Flexible_Cable` (single cable, no rigid bodies) and of the flexible cable models of
`demo_YAML_fsi` (see `data/yaml/fsi/flexible_cable`). The water column problem can also be solved with an
OpenFOAM (interFoam) fluid participant.

All combinations are run with the program `demo_PRECICE_flexible_cable` (solid participant, and Chrono::FSI-SPH
fluid participant) and, for OpenFOAM, the OpenFOAM case in `fluid_openfoam`.

Participants
------------

- **Solid** (`solid_chrono`): a Chrono MBS model containing the FEA mesh of the cable, with two cable models:
  - *soft cable* (`mbs_model.yaml`, `mbs_participant.yaml`): Young's modulus 6e8, the cable of
    `demo_FSI-SPH_Flexible_Cable`;
  - *stiff cable* (`mbs_model_stiff.yaml`, `mbs_participant_stiff.yaml`): identical except for its Young's modulus
    (1.2e10).

  The stiff cable coupled with OpenFOAM has its own participant file (`mbs_participant_openfoam.yaml`), since that
  coupling uses different data and a different order of the participants (see below). The cable nodes are
  exchanged through a `FEA_MESH_NODES` coupling mesh, whose edges are the cable elements. The solid participant
  sends node positions, velocities, and accelerations (Chrono::FSI-SPH) or node displacements (OpenFOAM), and
  receives nodal forces.
- **Fluid, Chrono::FSI-SPH** (`fluid_sph`): granular (CRM) or water (CFD) column. The fluid participant represents
  the cable with a proxy mesh, built from the node positions and segments given in its own specification file,
  which is used to generate the BCE markers of the cable (STAR pattern). It receives node positions, velocities,
  and accelerations, and does not depend on the cable model.
- **Fluid, OpenFOAM** (`fluid_openfoam`): water column only, solved with interFoam (volume of fluid, laminar). The
  cable is a cylindrical wall patch (`cable`) of a body-fitted mesh, moved with the cable by mesh motion
  (`displacementLaplacian`). Its diameter is 0.06, the effective width of the SPH cable (3 particle spacings for
  the STAR pattern of BCE markers), so that both fluid participants see the same cable; the cable diameter in
  the solid participant (0.02) only defines its stiffness and mass. The patch receives displacements and sends
  forces; preCICE maps both with `nearest-projection` onto the cable elements.

Combinations and limitations
----------------------------

| Fluid problem | Soft cable | Stiff cable |
|---|---|---|
| Chrono::FSI-SPH, granular (`-f CRM`) | yes | yes |
| Chrono::FSI-SPH, water (`-f CFD`) | yes | yes |
| OpenFOAM, water (`-f OpenFOAM`) | **no** | yes |

- **OpenFOAM requires the stiff cable.** The collapsing water column bends the soft cable flat onto the floor of
  the container, and the wave reflected from the right wall then lifts it past vertical. A body-fitted mesh that
  deforms with the cable cannot follow this motion: the cells between the cable and the floor collapse. The
  stiff cable deflects moderately under the same flow (tip rotation below 20 degrees), which the mesh motion
  supports. `demo_PRECICE_flexible_cable` rejects `-f OpenFOAM` with the soft cable.
- **OpenFOAM solves the water column only.** There is no OpenFOAM counterpart of the granular (CRM) problem.
- **OpenFOAM runs on Linux only**, since OpenFOAM has no Windows-native installation. The Chrono::FSI-SPH
  combinations run on any platform supported by Chrono::FSI-SPH (CUDA).
- **Duration.** The OpenFOAM configuration ends at 2 s, which covers the impact on the cable, the run-out, and the
  start of the wave reflected from the right wall; compare the two fluid participants over these first 2 s.
- **The two fluid participants do not agree quantitatively.** With the stiff cable, both predict the same
  response in shape and timing (peak deflection at about 0.4 s, then a decay), but the Chrono::FSI-SPH
  participant predicts a larger deflection: a peak tip displacement of 0.16, against 0.084 with OpenFOAM. On a
  fixed cable, the water force is 2 to 3.5 times larger in SPH. This is largely a resolution effect: at the
  particle spacing used here (0.02), the cable is only three spacings wide. Halving the spacing (with 6 BCE
  layers, which keeps the cable width) brings the force at impact and the onset of the deflection close to the
  OpenFOAM results, and lowers the peak deflection to 0.12, still about 1.5 times larger. The two
  participants also model the water differently: SPH uses an artificial (numerical) viscosity, much larger than
  that of water, and has no air phase.

preCICE configurations
----------------------

- `precice_config_crm.xml`, `precice_config_cfd.xml` (Chrono::FSI-SPH): the solid participant sends node
  positions, velocities, and accelerations. Serial explicit coupling with the fluid participant first and a time
  window equal to the integration step size, which matches the (default) sequential coupling scheme of a
  non-preCICE Chrono::FSI simulation. The solid participant runs second in each time window and reads the fluid
  forces at the end of the window. The participants exchange data through the current directory.
- `precice_config_openfoam.xml` (OpenFOAM): the solid participant sends node displacements. Serial explicit
  coupling with a time window of 5e-4 (two steps of the solid participant). The solid participant runs first in
  each time window, with the fluid forces of the previous window; OpenFOAM then follows the displacements of the
  current window, interpolated linearly in time. OpenFOAM adapts its time step to the Courant number and may take
  several steps (substeps) per window: the time interpolation gives each substep its own share of the cable
  motion. With the fluid participant first, OpenFOAM would only have the displacements of the previous window,
  the whole cable motion of a window would be applied in its first substep, and the resulting mesh velocities
  near the cable top would make the simulation unstable. The participants run from `solid_chrono` and
  `fluid_openfoam` and exchange data through this directory.

Running with Chrono::FSI-SPH
----------------------------

Run the two participants in two different processes, from the same directory (e.g., the directory of the Chrono
executables), with the same fluid problem (`-f CRM` or `-f CFD`) and cable model (`-m soft_cable`, the default,
or `-m stiff_cable`):

```
demo_PRECICE_flexible_cable -p Solid -f CFD -m stiff_cable
demo_PRECICE_flexible_cable -p Fluid_SPH -f CFD -m stiff_cable
```

Other options: `--no_vis` (no run-time visualization), `--no_output` (no output), `--quiet` (no verbose
terminal output), `-c <file>` (another preCICE configuration file).

With output enabled (the default), the solid participant writes the trajectory of the top cable node to
`DEMO_OUTPUT/PRECICE_Flexible_Cable/<CRM|CFD>_<SOFT_CABLE|STIFF_CABLE>/results.txt`, in the same format as
`demo_FSI-SPH_Flexible_Cable`.

Running with OpenFOAM
---------------------

Requirements (Linux):

- Chrono, built with the PRECICE and FSI modules enabled (the VSG module, optional, enables run-time visualization
  of the solid participant).
- OpenFOAM (openfoam.com distribution) and the OpenFOAM-preCICE adapter; see the preCICE
  [documentation](https://precice.org/adapter-openfoam-overview.html).

Run the two participants from two different terminals, using the `run.sh` script in each participant directory of
the Chrono data directory of the build tree (here, `<chrono_data_dir>`):

```
cd <chrono_data_dir>/precice/flexible_cable/solid_chrono
./run.sh
```

```
cd <chrono_data_dir>/precice/flexible_cable/fluid_openfoam
./run.sh             # or: ./run.sh -parallel (see system/decomposeParDict)
```

Do not run `demo_PRECICE_flexible_cable -f OpenFOAM` from another directory: it locates its data files relative to
`solid_chrono`. The solid script runs `demo_PRECICE_flexible_cable -p Solid -f OpenFOAM -m stiff_cable`; options
given to the script are passed on (e.g., `./run.sh --no_vis`). The fluid script generates the mesh
(`blockMesh`), initializes the water column (`setFields`), and runs interFoam.

Output:

- solid participant: trajectory of the top cable node in
  `solid_chrono/DEMO_OUTPUT/PRECICE_Flexible_Cable/OPENFOAM_STIFF_CABLE/results.txt` (same format as above);
- fluid participant: OpenFOAM time directories in `fluid_openfoam` (at the first time step after every multiple of
  0.02 s).

To remove all generated files, run `./clean.sh` in this directory (or in each participant directory).

The OpenFOAM case is adapted from the `breaking-dam-2d` case of the preCICE
[tutorials](https://github.com/precice/tutorials/tree/develop/breaking-dam-2d).
