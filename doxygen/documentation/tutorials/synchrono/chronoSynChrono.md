Chrono SYNCHRONO module tutorials {#tutorial_table_of_content_chrono_synchrono}
===============================

The Chrono distribution contains several demos for modeling and simulating autonomous vehicles and robots with the [SYNCHRONO module](@ref manual_synchrono).

SynChrono supports synchronization of distributed simulations of multiple vehicles or robots in a single time and space coherent Chrono world. SynChrono also provides out of the box wrapping of Chrono::Vehicle models, Chrono::Irrlicht and Chrono::Sensor visualizations, modeling of intelligent intersection systems, and synchronization of deformable (SCM) terrain.

SynChrono demos:

* Basic SynChrono Demos (demo_vehicles/)
    * demo_SYN_wheeled - Demo with driveable wheeled vehicles, on flat, rigid terrain, using MPI for synchronization.
    * demo_SYN_tracked - Demo with driveable tracked vehicles, on flat, rigid terrain, using MPI for synchronization.

* Feature-oriented SynChrono demos
    * demo_SYN_scm(_tracked) - Demo showing support for synchronized deformable (SCM) terrain
    * demo_highway - Demo showing simple model for lane-changing and a highway mesh

* FastDDS SynChrono demos
    * demo_SYN_DDS_wheeled - Demo with driveable wheeled vehicles, on flat, rigid terrain using DDS
    * demo_SYN_DDS_tracked - Demo with driveable tracked vehicles, on flat, rigid terrain using DDS
    * demo_SYN_DDS_distributed - Demo with driveable wheeled vehicles. Setup to allow for DDS nodes spread across multiple physical computing nodes via IP.

All SynChrono demos use command-line arguments to control various options at run-time rather than needing a re-compile. You can view the options for a particular demo by running `./path/to/demo_SYN_somedemo --help`. To run an MPI-based demo with `n` nodes, run the command:
````
mpirun -n <num nodes> path/to/demo_SYN_MPI_demo
````
For more complex MPI setups, please consult the documentation for your particular MPI distribution.

While the `mpirun` command handles initialization of each node, to run DDS-based demos, you will need to start each node manually in a separate process, perhaps with a script or simply by opening multiple terminal windows. To launch a DDS-based demo with `n` nodes, run this command:
````
./path/to/demo_SYN_DDS_demo -n <num nodes> -d <this node num>
````
where the argument to `d` is a different digit for each launched DDS node.