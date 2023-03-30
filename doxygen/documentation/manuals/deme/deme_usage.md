DEM-Engine usage {#deme_usage}
=================================

### Description

DEM-Engine, nicknamed _DEME_, does Discrete Element Method simulations:

- Using up to two GPUs at the same time (works great on consumer _and_ data center GPUs).
- With the particles having complex shapes represented by clumped spheres.
- With support for customizable contact force models (want to add a non-standard cohesive force, or an electrostatic repulsive force? You got this).
- With an emphasis on computational efficiency.
- With support for co-simulation with other C/C++ packages, such as [Chrono](https://github.com/projectchrono/chrono).

<p>
  <img width="380" src="https://i.imgur.com/mLMjuTc.jpg">
  <img width="380" src="https://i.imgur.com/PRbd0nJ.jpg">
</p>

Currently _DEME_ is a C++ package with an API design similar to Chrono's, and should be easy to learn for existing Chrono users. We are building a Python wrapper for _DEME_.

### Examples

After installing _DEME_, you can start trying out the demos.

- An all-rounder beginner example featuring a bladed mixer interacting with complex shaped particles: `./src/demo/DEMdemo_Mixer`.
- A place to learn how prescribed motions work in this package, using either analytical boundaries or particle-represented boundaries: `./src/demo/DEMdemo_Centrifuge` and `./src/demo/DEMdemo_Sieve`.
- A fun game-of-life simulator built with the package, showing the flexibility in terms of how you can use this tool: `./src/demo/DEMdemo_GameOfLife`.
- A few representative engineering experiments reproduced in DEM simulations, which potentially serve as starting points for your own DEM scripts: `/src/demo/DEMdemo_BallDrop`, `./src/demo/DEMdemo_ConePenetration`, `/src/demo/DEMdemo_Sieve`, `./src/demo/DEMdemo_Repose`.
- `./src/demo/DEMdemo_WheelDP` shows how to load a checkpointed configuration file to instantly generate a settled granular terrain, then run a drawbar-pull test on it. This demo therefore requires you to first finish the two GRCPrep demos to obtain the terrain checkpoint file. The granular terrain in these demos features DEM particles with a variety of sizes and shapes.
- More advanced examples showing the usage of the custom additional properties (called _wildcards_) that you can associate with the simulation entities, and use them in the force model and/or change them in simulation then deposit them into the output files: `./src/demo/DEMdemo_Indentation`.
- It is a good idea to read the comment lines at the top of the demo files to understand what they each does.

Some additional troubleshooting tips for running the demos:

- If errors similar to `CUDA_ERROR_UNSUPPORTED_PTX_VERSION` are encountered while you run the demos, or (rarely) the simulations proceed without detecting any contacts, then please make sure the CUDA installation is the same version as when the code is compiled.

### Limitations

_DEME_ is designed to simulate the interaction among clump-represented particles, the interaction between particles and mesh-represented bodies, as well as the interaction between particles and analytical boundaries.

- It is able to handle mesh-represented bodies with relatively simple physics, for example a meshed plow moving through granular materials with a prescribed velocity, or several meshed projectiles flying and hitting the granular ground. 
- However, if the bodies' physics are complex multibody problems, say it is a vehicle that has joint-connected parts and a motor with certain driving policies, or the meshed bodies have collisions among themselves that needs to be simulated, then _DEME_ alone does not have the infrastructure to handle them. But you can install _DEME_ as a library and do coupled simulations with other tools such as [Chrono](https://github.com/projectchrono/chrono), where _DEME_ is exclusively tasked with handling the granular materials and the influence they exert on the outside world (with high efficiency, of course). See the following section.


