Use mesh shapes {#tutorial_pychrono_demo_mesh}
===========================

Create complex rigid body shapes based on .obj meshes.
Uses [PyChrono](@ref pychrono_introduction).

Learn how to:

- load a .obj mesh file and use it for visualization of the shape
- load a .obj mesh file and use it for collision
- adjust position of center of mass respect to reference in ChBodyAuxRef
- change inertia properties.
	
<div class="ce-info">
Collision detection with generic concave meshes is slower and less
robust than any other options for collision shapes, so use it only if defining 
the collision shape via primitives like spheres boxes cylinders or their
clusters is too complex. (In fact, a good trade-off often is the following: use a detailed
mesh for visualization, and few simple primitives for collision.)
</div>

<div class="ce-info">
The mesh shape is a .obj file in Wavefront file format,
you can generate it from 3D modelers such as Blender, Maya, etc., as well as from some CAD.
</div>

<div class="ce-info">
For collision purposes, the .obj mesh must be "watertight", i.e. having
no gaps in edges, no repeated vertexes, etc. If you are not sure about this,
the free tool MeshLab, for example, has tools to check the topological correctness of the mesh.
</div>

<div class="ce-info">
For visualization purposes only, i.e. if you do not use the mesh also for 
collision, the mesh does not need to be watertight. (btw. If the visualization does not look good, 
check if the normals are correct in your .obj file.)
</div>



\include demo_mesh.py