
Finite Elements      {#manual_fea}
===============

This part discusses the Chrono [FEA module](group__chrono__fea.html).

Finite elements can be used in Chrono to model flexible parts. 

Different types of finite elements are supported:

- **solid volume** (e.g. tetrahedrons): can be used to model parts with arbitrary geometry, such as metal shapes, plastic molds, biomechanical tissues like muscles, etc.

- **surface volume** i.e. shells: can be used to model thin parts, like tires, airbags, etc.

- **beam** elements: can be used to model cables, wires, thin shafts, blades of wind turbines, etc.

Large displacements are supported for the majority of the finite elements. This allows, for instance, the simulation of problems like rubber structures undergoing 
large deflections, airplane taking complex manouvers in space, blades of wind turbines with large rotations and other geometric non-linearities. 
Of course linearized small-displacement analysis is supported too, as a subcase.


Detailed documentation on nodes and elements can be found in these pages:
* @subpage manual_fea_nodes
* @subpage manual_fea_elements




# Finite element data structures

Data structures for a FEA model are organized in this way:

![](http://www.projectchrono.org/assets/manual/fea_data_1a.png) ![](http://www.projectchrono.org/assets/manual/fea_data_1b.png)

Any Chrono \ref chrono::ChSystem "ChSystem" holds, among lists of bodies, links, loads and other physic items, also a list of @ref chrono::fea::ChMesh "fea::ChMesh" objects. These objects are themselves containers for finite elements (derived from @ref chrono::fea::ChElementBase "ChElementBase") and nodes (derived from @ref chrono::fea::ChNodeFEAbase "ChNodeFEAbase").

A single @ref chrono::fea::ChMesh "fea::ChMesh" can actually contain multiple elements.

A full set of [loads](@ref loads) and [links](@ref links) are available to apply loads and link nodes (either to other nodes or to \ref chrono::ChBody "ChBody").


# How to create a FEA model 

In the following example we show the data structures for a case with two elements, a constraint, a load:

![](http://www.projectchrono.org/assets/manual/fea_data_2a.png) ![](http://www.projectchrono.org/assets/manual/fea_data_2b.png)


In general:

- a **mesh** is a container for nodes and elements
  - add a ChMesh to the system using \ref chrono::ChSystem::Add() "ChSystem::Add()"
  - multiple meshes are allowed in a single system

- a **node** has degrees of freedom (xyz, rotations, etc.). 
  - add nodes to a mesh using \ref chrono::fea::ChMesh::AddNode() "ChMesh::AddNode()"
  - look at the [list of nodes](@ref manual_fea_nodes).

- an **element** connects N nodes. 
  - add elements to a mesh using \ref chrono::fea::ChMesh::AddElement() "ChMesh::AddElement()"
  - initialize the elements by telling which node are connected with `SetNodes()`
  - set a material property to the element by using `SetMaterial()`
  - look at the [list of elements](@ref manual_fea_elements).


The following example illustrates it.

## 1) Create a ChMesh 

- create the mesh (do this using `chrono_types::make_shared`, so it will be handled by a shared pointer, and you will not worry about deleting it)
- add the mesh to a physical system:

~~~{.cpp}
    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);
~~~

## 2) Create some nodes 

- create nodes
- note: usually node positions are offered as parameters in their constructors
- add nodes to the mesh

~~~{.cpp}
    // Create some point-like nodes with x,y,z degrees of freedom
    // While creating them, also set X0 undeformed positions.
    auto mnode1 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 1));
    auto mnode3 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnode4 = chrono_types::make_shared<ChNodeFEAxyz>(ChVector<>(1, 0, 0));
    
    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);
~~~

- set node properties, if needed:
  - e.g. most node classes provide a way to apply a local force (or even a torque if they have rotational DOFs) by using `SetForce()`, an easier alternative to using ChLoad classes, if the force is constant. See [loads](@ref loads) for additional info.
  - it is possible to attach an optional local point-mass using `SetMass()` for the nodes; otherwise the default mass for FEA nodes is zero as mass is mostly added by finite elements.

~~~{.cpp}
    // For example, set some non-zero mass concentrated at the nodes
    mnode1->SetMass(0.01); 
    mnode2->SetMass(0.01); 

    // For example, set an applied force to a node:
    mnode2->SetForce(ChVector<>(0, 5, 0));
~~~

## 3) Create a material

- create the material
- note that not all elements need a material, for instance ChElementSpring has no material.
- a single material can be shared between multiple elements.

~~~{.cpp}
	// Create a material, that will be assigned to each element,
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();

    // …and set its parameters
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
~~~

## 4) Create FEA elements

- create the elements
- add the elements to the mesh.
- assign nodes (which you created before) to the element.
- assign material(s) to the elements.

~~~{.cpp}
	// Create the tetrahedron element, 
    auto melement1 = chrono_types::make_shared<ChElementTetraCorot_4>();
    
    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // assign nodes
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);

    // assign material
    melement1->SetMaterial(mmaterial);
~~~


# See also


- [whitepapers page](http://projectchrono.org/whitepapers/) 
  Theory and additional information regarding the implementation of finite elements in Chrono 

- [tutorials](@ref tutorial_table_of_content_chrono_fea) 
  Demos and examples
  
- [API documentation](group__chrono__fea.html) 
  The C++ classes and functions of this module.





