
Finite Elements      {#manual_fea}
===============

This part of Chrono requires that the optional [FEA module](group__fea__module.html) is correctly installed and built. 
Go to the [installation guide](@ref module_fea_installation).

Finite elements can be used in Chrono to model flexible parts. 

Different types of finite elements are supported.

- solid volume elements like tetrahedrons can be used to model parts with arbitrary geometry, such as metal shapes, plastic molds, biomechanical tissues like muscles, etc.

- surface volume elements like shells can be used to model thin parts, like tires, airbags, etc.

- beam elements can be used to model cables, wires, thin shafts, blades of wind turbines, etc.

For most of these elements we support large displacements. This allows, for instance, the simulation of problems like rubber structures undergoing 
large deflections, airplane taking complex manouvers in space, blades of wind turbines with large rotations and other geometric non-linearities. 
Of course linearized small-displacement analysis is supported too, as a subcase.


Detailed documentation on nodes and elements can be found in these pages:


* @subpage manual_fea_nodes.


* @subpage manual_fea_elements.




# Finite element data structures

Data structures for a FEA model are organized in this way:

![](http://www.projectchrono.org/assets/manual/fea_data_1a.png) ![](http://www.projectchrono.org/assets/manual/fea_data_1b.png)

One can see that the main ingredients are

- a @ref chrono::fea::ChMesh object that contains elements and nodes

- some nodes, from the classes *derived* from @ref chrono::fea::ChNodeFEAbase. 

- the elements, from the classes *derived* from @ref chrono::fea::ChElementBase. 


Note, however, that one can also use some of the @ref chrono::ChLink objects to constraint nodes to other nodes or to chrono::ChBody objects.
Also, some @ref chrono::ChLoad objects like forces, torques, pressures, can be added. In the following example we show the data structures for
a case with two elements, a constraint, a load:

![](http://www.projectchrono.org/assets/manual/fea_data_2a.png) ![](http://www.projectchrono.org/assets/manual/fea_data_2b.png)


In general:

- A **mesh** is a container for nodes and elements
  - Add a ChMesh to the system using ChSystem::Add()
  - Multiple meshes are allowed in a single system

- A **node** has degrees of freedom (xyz, rotations, etc.). 
  - Add nodes to a mesh using ChMesh::AddNode()
  - Look at the [list of nodes](@ref manual_fea_nodes).

- An **element** connects N nodes. 
  - Add elements to a mesh using ChMesh::AddElement()
  - Initialize the elements by telling which node are connected with SetNodes()
  - Set a material property to the element by using SetMaterial()
  - Look at the [list of elements](@ref manual_fea_elements).


  
# How to create a FEA model 

Creating/Setting up a finite element mesh typically involves the following steps:

1. Create the @ref chrono::fea::ChMesh, and add it to a @ref chrono::ChSystem
2. Create some nodes
3. Create a material for elements, if needed
4. Create some elements between nodes

The following example illustrates it.

## 1) Create a ChMesh 

- Create the mesh (do this using std::make_shared, so it will be handled by a shared pointer, and you will not worry about deleting it)
- Add the mesh to a physical system:

~~~{.cpp}
    // The physical system: it contains all physical objects.
    ChSystem my_system;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);
~~~

## 2) Create some nodes 

- Create nodes, using shared pointers.
- Note: usually node positions are set as parameters in their constructors
- Add nodes to the mesh

~~~{.cpp}
    // Create some point-like nodes with x,y,z degrees of freedom
    // While creating them, also set X0 undeformed positions.
    auto mnode1 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 0));
    auto mnode2 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0, 1));
    auto mnode3 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 1, 0));
    auto mnode4 = std::make_shared<ChNodeFEAxyz>(ChVector<>(1, 0, 0));
    
    // Remember to add nodes and elements to the mesh!
    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);
~~~

- Set node properties, if you need. 
  - Ex. most node classes provide a way to apply a local force (or even a torque if they have rotational DOFs) by using SetForce() , an easier alternative to using ChLoad classes, if the force is constant.
  - Here you may want also to attach an optional local point-mass using SetMass() for the nodes;  otherwise the default mass for FEA nodes is zero, as mass is mostly added by finite elements.

~~~{.cpp}
    // For example, set some non-zero mass concentrated at the nodes
    mnode1->SetMass(0.01); 
    mnode2->SetMass(0.01); 

    // For example, set an applied force to a node:
    mnode2->SetForce(ChVector<>(0, 5, 0));
~~~

## 3) Create a material

- Create the material, using shared pointers.
- Note that not all elements need a material, for instance ChElementSpring has no material.
- A single material can be shared between multiple elements.

~~~{.cpp}
	// Create a material, that will be assigned to each element,
    auto mmaterial = std::make_shared<ChContinuumElastic>();

    // …and set its parameters
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
~~~

## 4) Create FEA elements

- Create the elements, using shared pointers.
- Add the elements to the mesh.
- Assign nodes (which you created before) to the element.
- Assign material(s) to the elements.

~~~{.cpp}
	// Create the tetrahedron element, 
    auto melement1 = std::make_shared<ChElementTetra_4>();
    
    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // assign nodes
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);

    // assign material
    melement1->SetMaterial(mmaterial);
~~~


# See also


- [whitepapers page](@ref whitepaper_root) 
  Theory and additional information regarding the implementation of finite elements in Chrono 

- [tutorials](@ref tutorial_table_of_content_chrono_fea) 
  Demos and examples
  
- [API documentation](group__fea__module.html) 
  The C++ classes and functions of this module.

- [installation guide](@ref module_fea_installation)
  How to install and build this module.




