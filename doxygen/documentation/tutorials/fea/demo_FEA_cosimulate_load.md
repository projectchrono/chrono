Apply cosimulated loads to amesh  (demo_FEA_cosimulate_load.cpp)     {#tutorial_demo_FEA_cosimulate_load}
================================


Tutorial that teaches how to use the 
[FEA module](group__fea__module.html)
to import an .INP Abaqus mesh with a 3D tetrahedral mesh and apply loads to the surface coming from an external process.

The external process here is simply simulated using a function in the same .cpp unit, but in a context of cosimulation it could be an external process/program like, say, a CFD software, that receives the 3D mesh from Chrono and gives back the fluid forces to Chrono.

\include demo_FEA_cosimulate_load.cpp