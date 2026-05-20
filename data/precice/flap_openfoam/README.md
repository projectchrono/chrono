Coupling of Chrono MBS with OpenFOAM
====================================

This demo is adapted from the preCICE quickstart [tutorial](https://precice.org/quickstart.html). 
The data files in the subdirectory `fluid_openfoam` are copied from the [precice/tutorial](https://github.com/precice/tutorials/tree/develop/quickstart/fluid-openfoam) GitHub repository, with a simplified `run.sh` bsh script file. See the [preCICE license](fluid_openfoam/LICENSE.txt).  

The multibody system contains a single body (the _flap_) connected to ground with a revolute joint, with the single degree of freedom representing the deflection angle of the flap in the channel.
A rotational spring is attached between the flap and ground at the joint location.
The flap has an oscillatory motion due to the force exerted by the fluid in a channel.
To stabilize the coupled simulation and control the body oscillations, the spring constant is increased after 1.5 seconds.

While Chrono MBS is always 3D, the preCICE interface (coupling mesh) in this demo is 2D: the simulation occurs in the (x-y) plane.

Requirements
------------

**NOTE:** Because OpenFOAM does not provide a Windows-native installation, this coupled simulation can only be run on a Linux machine.

- Chrono MBS<br>
  Chrono must be built with the PRECICE module enabled. Optionally, the VSG module may also be enabled to allow run-time visualization of the mechanical system.

- OpenFOAM<br>
  OpenFOAM and the OpenFOAM-preCICE adapter must be installed. See the preCICE quickstart [tutorial](https://precice.org/quickstart.html) for details.

Running the coupled simulation
------------------------------

The two simulations (MBS and CFD) must be run from two different terminal windows.

- Chrono MBS<br>
  The multibody participant is executed like any other Chrono demo from the build tree (here, `<chrono_build_dir>`):
  ```
  cd <chrono_build_dir>
  ./demo_PRECICE_flap_openfoam
  ```

- OpenFOAM<br>
  To run the CFD participant, execute the `run.sh` script from its locatioono in the Chrono data directory (here, `<chrono_data_dir>`):
  ```
  cd <chrono_data_dir>/precice/flap_openfoam/fluid_openfoam
  ./run.sh
  ```
