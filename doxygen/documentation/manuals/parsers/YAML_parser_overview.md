Overview of the YAML parser for Chrono models and simulations {#YAML_parser_overview}
===============================================================

## General structure of the YAML parsers

The Chrono YAML parsers allow running Chrono simulations fully specified through YAML files without the need for writing and building user code.
To this end, a complete Chrono simulation via YAML files requires specification of both Chrono models and Chrono solvers.
Currently, the Chrono YAML parsers support rigid multibody, vehicle, and FSI simulations.  For any type of Chrono simulation, the user is expected to provide:
1. a top-level YAML *simulation* file which specifies the model to be simulated and the solvers used for the simulation, as well as optional outpout and run-time visualization setrtings,
2. a YAML *model* file with a description of the respective Chrono model (MBS, vehicle, or FSI), and
3. a YAML *solver* file with a description of the solver type and parameters appropriate for the desired type of simulation.

These 3 YAML files provide the top of a hierarchy of YAML specification files that fully describe the desired Chropno simulation.
Note that FSI problems, which couple MBS and fluid simulations have a deeper hierarchy of YAML specification files, as the top-level FSI *simulation* file refers to an MBS *simulation* file and afluid *simulation* file.

The various types of Chrono simulations that can be conducted through YAML files and the corresponding required YAML files are shown in the schematic below.

<img src="http://www.projectchrono.org/assets/manual/YAML_schemas.png" width="600">


## Reference frames {#YAML_parser_frames}

A Chrono model is specified with respect to an implied model reference frame (M). You can think of this reference frame as a global frame for the purpose of defining the model.  In other words, all body positions (location and orientation, <sub>M</sub>X<sub>B</sub>), joint frames (if joints are specified through an absolute joint frame), TSDA end point locations (if specified through absolute locations), etc. are assumed to be given relative to this model frame.

During instantiation of a YAML-specified Chrono model, the caller has the option of specifying the frame transform from the global reference frame (G) to the model frame of that instance, <sub>G</sub>X<sub>M</sub>.  This allows placing multiuple instances of the same YAML-specified Chrono model in the global scene, at different global locations and with different orientations.

The YAML parser assumes that rigid bodies are specified in the most general (and flexible) manner, namely with an arbitrary body reference frame (B). The body centroidal frame (C) is then specified through a transform (location and orientation, <sub>B</sub>X<sub>C</sub>) relative to the body reference frame. The body inertia tensor (composed of inertia moments and, optionally, inertia products) is assumed to be specified relative to the body centroidal frame. Similarly, any geometry (collision and/or visualization) attached to a rigid body is assumed to be specified relative to the body reference frame. For example, a cylindrical shape is fully specified through its radius and length, as well as the location of its center (l<sub>cyl</sub>) in the body reference frame and the direction of the cylinder axis (a<sub>cyl</sub>) expressed in the body reference frame.

<img src="http://www.projectchrono.org/assets/manual/YAML_frames.png" width="600" />


## Chrono runner program

If the Chrono::Parser module, with YAML support, is enabled, the `run_chrono` application can be used to run simulations specified completelyy through YAML files:

```
Copyright (c) 2026 projectchrono.org
Chrono version: 9.0.1

Usage:
  E:\Build\chrono\bin\Release\run_chrono.exe [OPTION...]

  -h, --help              Print usage
  -s, --sim_file arg      Simulation specification file (YAML format)
  -o, --out_dir arg       Output directory (default: DEMO_OUTPUT/YAML_CHRONO/)
      --no_output         Disable output
      --no_visualization  Disable run-time visualization
```

The optional argument `out_dir` specifies the top-level location of all Chrono simulation outputs (if enabled in the YAML simulation specification). The output directory itself is created if it does not exist, but all its parents must already exists.
Optionally, all output can be supressed (using `--no_output`), overwriting settings in the YAML files.  Similarly, run-time visualization can be disabled (`--no_visualization`), regardless of the settings in the AML specification.

The Chrono distribution includes (in the `data\yaml\` directory) several YAML specification files for illustrating the use of the Chrono YAML parsers.  Some examples can be executed as follows:
- MBS simulation (defaults to a slider-crank mechanism):<br>
  `./run_chrono -s ../data/yaml/mbs/mbs.yaml`<br>
  (modify the mbs.yaml file to point to a different MBS model)
- Vehicle simulation (defaults to a wheeled vehicle model):<br>
  `./run_chrono -s ../data/yaml/vehicle/vehicle.yaml`<br>
  (modify the mbs.yaml file to point to a different vehicle model)
- FSI-SPH problem (defaults to a CFD simulation of a cylinder dropped in water):<br>
  `./run_chrono -s ../data/yaml/fsi/objectdrop/fsi_objectdrop.yaml`<br>
  (use a different FSI specification file for other examples)
- FSI-TDPF problem <br>
  `./run_chrono -s ../data/yaml/fsi/sphere_decay/fsi_sphere_decay.yaml`
