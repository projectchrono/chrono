About Chrono {#introduction_chrono}
==========================

Chrono is split into multiple **Modules** which are are additional libraries that can be *optionally* used to expand the features of Chrono::Engine. In this sense, the Chrono::Engine framework is a modular concept that can be expanded or simplified, depending on user's needs.

There are various motivations for modularity:

-   the compilation of module, usually, depends on additional software
    components that require some installation and configuration from the
    user (for instance, the unit for GPU computation requires that the
    user has the CUDA SDK installed in his system, the Matlab unit
    requires the Matlab API to be installed, etc.) We do not want to
    force the user to have all these prerequisites - one can compile and
    use only the module that fit into his system;

-   splitting the project in smaller components avoid dealing with a
    monolithic, huge dll;

-   further module could be developed in future without changing the core
    library of the project.

For these and other reasons, we decided to make the compilation of the
module *conditional*: the user can enable their compilation -if
interested-, otherwise disable them to make the build process easier and
with minimum requirements.

<!-- In the following picture one can get an idea of how module can depend on
external libraries, whereas the core system of Chrono::Engine just
depends on the plain operating system.

![](/images/module.png "module.png") -->

Features
==========================

The following is a short list of main features of Chrono

### Physical modeling

-   Rigid body support
-   Flexible body support via Chrono::FEA module - both for ANCF and co-rotational nonlinear finite element analysis
-   Support for fluid-solid interaction problems, via Chrono::FSI module
-   Coulomb friction model capturing stick-slip phenomena.
-   Support for rolling friction and spinning friction.
-   Support for handling frictional contact via two approaches: a complementarity approach and a penalty approach.
-   Springs and dampers with non-linear response. Can be user defined.
-   Large collection of joints and constraints: spherical, revolute, prismatic, universal, glyph, screw, bevel and spur gears, pulleys, etc.
-   Unilateral constraints.
-   Constraints to impose trajectories, or to force motion on splines, curves, surfaces, etc.
-   Constraints can have limits (ex. elbow). 
-   Constraints can be rheonomic or holonomic
-   Custom constraint for linear motors.
-   Custom constraint for pneumatic cylinders.
-   Custom constraint for motors, with reducers, learning mode, etc.
-   On the fly constraint activation/deactivation.
-   Simplified 1D dynamic models. Examples: powertrain, clutches, brakes, etc. For more sophisticated models see companion Chrono::Vehicle module.
-   All physical items can have an arbitrary number of 'assets' used for defining visualization shapes, custom properties, etc.

### Solver
-   HHT solver for index 3 differential algebraic equations
-   Symplectic first order half-implicit Euler solver for large frictional contact problems
-   Speed-impulse level solver for handling large frictional contact problems.
-   Handling of redundant/ill posed constraints.
-   Stabilization or projection methods to avoid constraint drifting.
-   Static analysis solver.
-   Inverse kinematics and interactive manipulation.

### Collision features

-   Supports compounds of spheres, cubes, convex geometries, triangle meshes, etc.
-   Additional collision support provided by the Bullet collision detection engine, which is wrapped inside Chrono::Engine.
-   Broad phase collision detection: sweep-and-prune SAT.
-   Narrow phase collision detection: AABB and/or OBB binary volume trees, to handle geometries with thousands of details.
-   Detail phase with custom primitive-to-primitive fallbacks.
-   Safety 'envelope' around objects.
-   Report penetration depth, distance, etc.
-   Bodies can be activated/deactivated, and can selectively enter collision detection.

### Implementation details

-   ANSI-compliant C++ syntax.
-   Optimized custom classes for vectors, quaternions, matrices.
-   Optimized custom classes for coordinate systems and coordinate transformations, featuring a custom compact algebra via operator overloading.
-   All operations on points/speeds/accelerations are based on quaternion algebra and have been profiled for fastest execution.
-   Custom sparse matrix class.
-   Custom redirectable stream classes, featuring platform independent file archiving and modern syntax.
-   Special archive engine, with easy and reliable persistent/transient serialization. Includes versioning and deep pointers storage.
-   Expandable run-time class factory.
-   Custom pseudo-'run-time-type-information', to allow persistence even in case of name-mangling with different C++ compilers.
-   High resolution timer, platform independent.
-   Class to create PostScript(tm) output.


### Other

- Interface with MATLAB
- Cosimulation with Simulink
- Import STEP cad files to define complex geometries
- Online/offline visualization with Irrlicht and POV-Ray, respectively.
-   Classes for genetic & classical optimization.
-   Classes for Newton-Raphson solution of non-linear equations.
-   Classes for interfacing external geometric data (NURBS, splines).
-   Scripting via Python.
-   Makefile system based on CMake (cross-platform, on Windows 32/64 bit, Linux, OSX).


