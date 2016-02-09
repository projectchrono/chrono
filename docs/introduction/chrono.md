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


Frequently Asked Questions
==========================

### Where can I start to read the documentation?

If you are a new user, we suggest you to start from [the main page of
this WIKI](Main_Page "wikilink"), or directly go to [this quick
installation guide](Installation "wikilink").

### Does CHRONO::ENGINE have a graphical user interface?

No, it is a C++ library and you must be a software programmer in order
to take advantage of it. However, if you need a CAD-like interface, you
may give a look at
[Chrono::SolidWorks](ChronoSolidWorks:Introduction "wikilink"), our
add-in for the SolidWorks CAD; this allows you to create the assets for
Chrono::Engine simulations using powerful graphical-user interface
tools.

### Is CHRONO::ENGINE free, or should I pay a fee?

Since 2013, Chrono::Engine got a permissive license of BSD style, so you
can use it freely in your projects (read the details in the license.txt
file in the repository).

### I want to use CHRONO::ENGINE, but I have few knowledge about C++ language..

Using CHRONO::ENGINE requires adequate knowledge about the C++ language.
If you don't know what's 'templating', 'polimorphism' or such, please
learn some basic lessons about C++ (for example see [these
links](http://www.deltaknowledge.com/chronoengine/links.html)).

### How can I contribute to the development of Chrono::Engine?

Look at the instructions in the page about the [GIT
repository](ChronoEngine:GIT_repository "wikilink"). If you feel like
contributing to the development, just fork the chrono GIT, then submit
us a pull request when you think you have interesting contributions.

##Build

### Can CHRONO::ENGINE be compiled on platform XXX with compiler YYY?

Yes, probably it can, but currently we test it under Windows (32 and 64
bit, XP and Vista, with MingW GNU compiler and Microsoft VisualC++
compilers) and Linux (32 and 64 bit, GNU compiler)

### Should I need to build all sub units?

No, to keep dependencies as few as possible, Chrono::Engine is designed
in a way so that advanced features, that might depend on additional
libraries or tools, are grouped into
[units](ChronoEngine:Units "wikilink") that can be selected in the CMake
interface when setting up the build.

##Programming

### I see that CHRONO::ENGINE implements a 'fake RTTI' mechanism, mostly used by the class factory. Why doesn't you use the ready-to-use typeid mechanism of default C++ RTTI?

Well, though C symbol decoration is always the same, different compilers
implement different schemes for name decoration of C++ class members.
Therefore, if you use the RTTI name decoration for a polimorphic
serialization mechanism, your files won't be platform-independent. Since
our class factory is based on a custom RTTI, the class factory of the
persistent/transient deserialization will always work, regardless of
compiler/platform.

##Chrono Engine Workflow

The ProjectChrono ecosystem includes tools, plugins and libraries, most
of them revolving around the Chrono::Engine middleware.

The Chrono::Engine middleware is a modular set of libraries for physical
simulations, organized in [units](Units "wikilink"). Some of those units
allow interfacing Chrono::Engine to external software, for example for
pre-processing and post-processing data, or for displaying real-time
simulations with OpenGL, or for parsing Python commands.

This means that there are many options for the assets workflow. In a
broad context, the following picture shows some of these possibilities:

![](/images/Workflow.png "workflow.png")
