---
layout: default
title: Chrono::Engine Features
permalink: /chrono_engine/features/
---

The following is a short list of main features of Chrono::Engine.

### Core features

-   Fully ANSI-compliant C++ syntax.
-   Optimized custom classes for vectors, quaternions, matrices. All
    math classes implement operator overloading and type templates.
-   Optimized custom classes for coordinate systems and coordinate
    transformations, featuring a custom compact algebra via
    operator overloading.
-   All operations on points/speeds/accelerations are based on
    quaternion algebra and have been profiled for fastest execution.
-   Custom sparse matrix class.
-   Linear algebra functions for LU decomposition, products, Choleski,
    Von Kauffmann, LDLt and SVD decompositions, etc.
-   Custom redirectable stream classes, featuring platform independent
    file archiving and modern syntax.
-   Special archive engine, with easy and reliable
    persistent/transient serialization. Includes versioning and deep
    pointers storage.
-   Expandable run-time class factory.
-   Custom pseudo-'run-time-type-information', to allow persistence even
    in case of name-mangling with different C++ compilers.
-   High resolution timer, platform independent.
-   Class to create PostScript(tm) files easily.

### Physical modeling

-   Rigid bodies, markers, forces, torques
-   Bodies can be activated/deactivated, and can selectively enter
    collision detection.
-   If markers or bodies are moved by external routines, a BDF method
    will update kinematic data.
-   Speed and angular speed of rigid bodies can be clamped in order to
    increase stability (for VR simulations).
-   Resting rigid bodies can automatically enter the 'frozen' mode, to
    allow real-time simulations of complex scenarios.
-   Exact Coloumb friction model, for precise stick-slip of bodies.
-   Parts can collide and rebounce, depending on
    restitution coefficients.
-   Springs and dampers, even with non-linear features
-   Wide set of joints (spherical, revolute joint, prismatic, universal
    joint, glyph, etc.)
-   Unilateral constraints.
-   Constraints to impose trajectories, or to force motion on splines,
    curves, surfaces, etc.
-   Special joints for modeling screws.
-   Constraints for bevel or spur gears.
-   Constraints can have limits (ex. elbow) and can be rheonomic,
    motorized
-   Custom constraint for linear motors.
-   Custom constraint for pneumatic cylinders.
-   Custom constraint for motors, with reducers, learning mode, etc.
-   All joints can report the reaction force.
-   Constraints can be activated/deactivated.
-   Brakes and clutches, with precise stick-slip effect.
-   Lot of non-linear properties of items (ex. time-dependant force) can
    be set with modular 'function' objects, with GUI.
-   Polymorphic interface to solver, to include particles, SPH and FEM
    (bricks, tetrahedrons, etc.).
-   Monodimensional dynamic items (ex.for powertrains, with clutches,
    brakes, torques, torsional stiffness, etc.)
-   Constraints for pulleys.
-   Rolling friction and spinning friction.
-   All physical items can have an arbitrary number of 'assets' attached
    to them.
-   Attached assets can be used for defining visualization shapes,
    custom properties, etc.

### Solver

-   Custom HyperOCTANT technology for efficient real-time solution of
    large LCP problems, even with critical cases of friction, collision
    and stacking.
-   A special iterative solver can handle real-time massive simulations,
    with more than one million of constraints.
-   Handling of redundant/ill posed constraints.
-   Modern 'real-time' integration exploiting differential inclusions,
    with speed-impulse LCP.
-   Stabilization or projection methods to avoid constraint drifting.
-   Static solution, even with strong geometric nonlinearities.
-   Inverse kinematics and interactive manipulation.

### Collision features

-   Supports compounds of spheres, cubes, convex geometries, triangle
    meshes, etc.
-   Advanced collision methods are available thank to the Bullet
    collision detection engine, which is wrapped inside Chrono::Engine.
-   Broad phase collision detection: sweep-and-prune SAT.
-   Narrow phase collision detection: AABB and/or OBB binary volume
    trees, to handle geometries with thousands of details.
-   Detail phase with custom primitive-to-primitive fallbacks.
-   Safety'envelope' around objects.
-   Report penetration depht, distance, etc.
-   Conveyor belts.

### Other

-   Modular libraries, based on [units](ChronoEngine:Units "wikilink"):
    -   interface with Matlab
    -   cosimulation with Simulink
    -   import STEP cad files
    -   use the Python language
    -   perform postprocessing and visualize animations in POV,
        Irrlicht, etc.
    -   etc.
-   Classes for genetic & local optimization.
-   Classes for Newton-Raphson solution of non-linear equations.
-   Classes for interfacing foreign geometric data (NURBS, splines).
-   The multibody engine can be scripted via Python.
-   Makefile system based on CMake (cross-platform, on Windows 32/64
    bit, Linux,etc.).
