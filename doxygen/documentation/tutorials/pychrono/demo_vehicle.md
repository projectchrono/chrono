Simulate vehicle dynamics in Python using PyChrono  {#tutorial_pychrono_demo_vehicle}
==========================

Model a HMMWV military truck and drive it in simulation. 
Uses [PyChrono](@ref pychrono_introduction), Irrlicht and Vehicle submodules.

You can:

- use one of the predefined vehicle model in PyChrono (HMMWV, Sedan or Citybus)
- Define your own model in a JSON file

Remember to change data paths in the code.
	
HMMWV predefined model:

\include demo_vehicle_HMMWV.py

HMMWV imported from json file:

\include demo_vehicle_HMMWV_JSON.py
