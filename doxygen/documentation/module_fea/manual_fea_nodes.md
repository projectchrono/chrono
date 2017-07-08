
List of FEA nodes         {#manual_fea_nodes}
=======================

\tableofcontents


Different types of nodes can be used in the [FEA module](@ref manual_fea).
In this page you can find a description of their properties. 

- A **node** in an object with degrees of freedom (xyz, rotations, etc.). 
- There are different types of nodes, some elements require specific types of nodes.
- Nodes are handled via std::shared_ptr shared pointers: you do not have to worry about deleting them.
- Add nodes to a mesh using ChMesh::AddNode()
  


# ChNodeFEAxyz  {#manual_ChNodeFEAxyz}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyz.png)

- 3 coordinates (p, ie. x y z translation in 3D)
- Ex. Used by solid elements:
  - ChElementTetra_4 
  - ChElementTetra_10 
  - ChElementHexa_8
  - ChElementHexa_20  etc. 



# ChNodeFEAxyzrot  {#manual_ChNodeFEAxyzrot}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzrot.png)

- 6 coordinates (translation p and rotation in 3D)
- Note: rotation expressed by quaternions q
- Ex. used by these elements:
  - ChElementBeamEuler 
  - ChElementShellReissner


# ChNodeFEAxyzD  {#manual_ChNodeFEAxyzD}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzD.png)

- 6 coordinates (p translation and Dx Dy Dz direction)
- Useful for defining simple beams of cable type, where information about torsion is not useful
- Ex. used by these elements:
  - ChElementCableANCF 
  - ChElementShellANCF

 
# ChNodeFEAxyzDD {#manual_ChNodeFEAxyzDD}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzDD.png)

- 9 coordinates (x y z translations and two directions)
- Ex. used by these elements:
  - ChElementBeamANCF 

  
# ChNodeFEAxyzP   {#manual_ChNodeFEAxyzP}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzP.png)

- 1 coordinates (a scalar P, in a 3D space)
- Used for thermal and electrostatic analysis
- Ex. used by these elements:
  - ChElementTetra_4_P 

  
  
# Theory

Additional information regarding the implementation of finite elements
in Chrono can be found at the  
[whitepapers page](http://projectchrono.org/whitepapers/).


# Examples

See demos and examples at the 
[tutorials](@ref tutorial_table_of_content_chrono_fea) page.





