Load a STEP file and create joints   {#tutorial_pychrono_demo_cascade_step_robot}
==================================

Use pychrono.cascade to load a STEP file and create constraints between the bodies.
We provide the .step file in the data/ directory for this example.

Uses [PyChrono](@ref pychrono_introduction).

Learn how to:

- load a STEP file, saved from a 3D CAD.
- fetch parts and recerences from the STEP document, and create joints between them.
- assign a ChLinkTrajectory to a part

\include demo_cascade_step_robot.py