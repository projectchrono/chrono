Chrono SYNCHRONO module tutorials {#tutorial_table_of_content_chrono_synchrono}
===============================

The Chrono distribution contains several demos for modeling and simulating autonomous vehicles and robots with the [SYNCHRONO module](@ref manual_synchrono).

SynChrono supports synchronization of distributed simulations of multiple vehicles or robots in a single time and space coherent Chrono world. SynChrono also provides out of the box wrapping of Chrono::Vehicle models, Chrono::Irrlicht and Chrono::Sensor visualizations, modeling of intelligent intersection systems, and synchronization of deformable (SCM) terrain.

SynChrono demos:

* Basic SynChrono Demos (demo_vehicles/)
    * demo_SYN_vehicles - Demo with driveable wheeled vehicles, on flat, rigid terrain
    * demo_SYN_tracked - Demo with driveable tracked vehicles, on flat, rigid terrain

* Feature-oriented SynChrono demos
    * demo_SYN_scm(_tracked) - Demo showing support for synchronized deformable (SCM) terrain
    * demo_highway - Demo showing simple model for lane-changing and a highway mesh
