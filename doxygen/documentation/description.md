Brief Introduction {#introduction_chrono}
==========================


The following is a short list of main attributes of Chrono.

### Physics modeling

-   Rigid body support.
-   Flexible body support - both for ANCF and co-rotational nonlinear finite element analysis.
-   Support for fluid-solid interaction problems, via Chrono::FSI module.
-   Coulomb friction model capturing stick-slip phenomena.
-   Support for rolling friction and spinning friction.
-   Support for handling frictional contact via two approaches: a complementarity approach and a penalty approach.
-   Springs and dampers with non-linear response. Can be user defined.
-   Large collection of joints and constraints: spherical, revolute, prismatic, universal, glyph, screw, bevel and spur gears, pulleys, etc.
-   Unilateral constraints.
-   Constraints to impose trajectories, or to force motion on splines, curves, surfaces, etc.
-   Constraints can have limits (ex. elbow).
-   Constraints can be rheonomic or holonomic.
-   Custom constraint for linear motors.
-   Custom constraint for pneumatic cylinders.
-   Custom constraint for motors, with reducers, learning mode, etc.
-   On the fly constraint activation/deactivation.
-   Simplified 1D dynamic models. Examples: powertrain, clutches, brakes, etc. For more sophisticated models see the Chrono::Vehicle module.
-   All physical items can have an arbitrary number of 'assets' used for defining visualization shapes, custom properties, etc.

### Solver
-   HHT solver for index 3 differential algebraic equations.
-   Symplectic first order half-implicit Euler solver for large frictional contact problems.
-   Speed-impulse level solver for handling large frictional contact problems.
-   Handling of redundant/ill posed constraints.
-   Stabilization or projection methods to avoid constraint drifting.
-   Static analysis solver.
-   Inverse kinematics and interactive manipulation.

### Collision detection features

-   Supports compounds of spheres, cubes, convex geometries, triangle meshes, etc.
-   Additional collision support provided by the Bullet collision detection engine, which is wrapped inside Chrono::Engine.
-   Broad phase collision detection: sweep-and-prune SAT.
-   Narrow phase collision detection: AABB and/or OBB binary volume trees.
-   Detail phase with custom primitive-to-primitive fallbacks.
-   Safety 'envelope' around objects.
-   Report penetration depth, distance, etc.
-   Bodies can be activated/deactivated, and can selectively enter collision detection.

### Implementation details

-   Eigen-based matrix and vector operations.
-   Optimized custom classes for vectors, quaternions, matrices.
-   Optimized custom classes for coordinate systems and coordinate transformations.
-   All operations on points/speeds/accelerations are based on quaternion algebra.
-   Special archive engine, with easy and reliable persistent/transient serialization.
-   Expandable run-time class factory.

### Other

- Template-based vehicle modeling through Chrono::Vehicle
- Scripting via Python (Chrono::Python)
- Import STEP cad files to define complex geometries
- Run-time visualization with Irrlicht or VSG
- Post-processing visualization with POV-Ray or Blender
- Build system based on CMake (cross-platform, on Windows 64 bit, Linux, MacOS).
