Coupling of Chrono MBS with OpenFOAM (3D)
=========================================

This demo is a 3D version of the 2D flap demo in `../flap_openfoam_2D`, itself adapted from the preCICE quickstart [tutorial](https://precice.org/quickstart.html).
The OpenFOAM case in the subdirectory `fluid_openfoam` extends the 2D case of the [precice/tutorial](https://github.com/precice/tutorials/tree/develop/quickstart/fluid-openfoam) GitHub repository to 3D: the flap is a plate spanning part of the channel width, with no-slip walls replacing the `empty` front and back patches of the 2D case. See the [preCICE license](fluid_openfoam/LICENSE.txt).

As in the 2D case, the flap is placed slightly off the channel center line: the gaps to the top and bottom walls differ, and so do the gaps to the front and back walls.
The resulting flow asymmetry loads the plate in both pitch (about z) and roll (about x).

The multibody system contains a single body (the _flap_) connected to ground with a spherical joint.
Two rotational springs, about the x and z axes, are attached between the flap and ground at the joint location.
The flap starts with a small angular velocity about these two axes and has an oscillatory motion due to the force exerted by the fluid in the channel.
To stabilize the coupled simulation and control the body oscillations, the spring constants are increased after 1.5 seconds.

The preCICE interface (coupling mesh) in this demo is 3D: on the Chrono side, it consists of the points on the flap surface listed in `solid_chrono/flap.txt`.

Requirements
------------

**NOTE:** Because OpenFOAM does not provide a Windows-native installation, this coupled simulation can only be run on a Linux machine.

- Chrono MBS<br>
  Chrono must be built with the PRECICE module enabled. Optionally, the VSG module may also be enabled to allow run-time visualization of the mechanical system.

- OpenFOAM<br>
  OpenFOAM and the OpenFOAM-preCICE adapter must be installed. See the preCICE quickstart [tutorial](https://precice.org/quickstart.html) for details.

Running the coupled simulation
------------------------------

The two simulations (MBS and CFD) must be run from two different terminal windows (two different processes).

- Chrono MBS<br>
  To run the Chrono participant, execute the `run.sh` script from its location in the Chrono data directory (here, `<chrono_data_dir>`):
  ```
  cd <chrono_data_dir>/precice/flap_openfoam_3D/solid_chrono
  ./run.sh
  ```

  **NOTE:** do _not_ run directly the `demo_PRECICE_flap_openfoam` executable from its location in the build or install tree.
  The same executable runs the 2D and 3D versions of this demo; the `run.sh` script selects the 3D version with the `-3D` argument.

- OpenFOAM<br>
  To run the CFD participant, execute the `run.sh` script from its location in the Chrono data directory (here, `<chrono_data_dir>`):
  ```
  cd <chrono_data_dir>/precice/flap_openfoam_3D/fluid_openfoam
  ./run.sh
  ```
