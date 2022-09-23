Gear constraint example (demo_MBS_gears.cpp)  {#tutorial_demo_gears}
==========================

Create a transmission made of pulleys, 
spur gears, bevel gears. 

Custom simplified constraints are available, 
to avoid the simulation of contacts between gear teeth. 

This tutorial shows how to:

- use ChLinkGear constraints to represent simplified gear interaction in 3D space
- the ChLinkPulley constraint is used to model simplified pulleys in 3D space.
- manage rotations of references using ChCoordsys and ChFrame classes.
- create a motor between two parts.  
 
 
\include demo_MBS_gears.cpp

