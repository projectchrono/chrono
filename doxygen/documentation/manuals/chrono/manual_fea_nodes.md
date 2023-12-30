
FEA Nodes        {#manual_fea_nodes}
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
  - ChElementTetraCorot_4 
  - ChElementTetraCorot_10 
  - ChElementHexaCorot_8
  - ChElementHexaCorot_20  etc. 



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
  - ChElementShellANCF_3423

 
# ChNodeFEAxyzDD {#manual_ChNodeFEAxyzDD}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzDD.png)

- 9 coordinates (x y z translations and two directions)
- Ex. used by these elements:
  - ChElementBeamANCF_3333 
  - ChElementShellANCF_3833


# ChNodeFEAxyzDDD {#manual_ChNodeFEAxyzDDD}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzDDD.png)

- 12 coordinates (x y z translations and three directions)
- Ex. used by these elements:
  - ChElementBeamANCF_3243 
  - ChElementShellANCF_3443
  - ChElementHexaANCF_3843


# ChNodeFEAxyzP   {#manual_ChNodeFEAxyzP}

![](http://www.projectchrono.org/assets/manual/fea_ChNodeFEAxyzP.png)

- 1 coordinates (a scalar P, in a 3D space)
- Used for thermal and electrostatic analysis
- Ex. used by these elements:
  - ChElementTetraCorot_4_P 


# ChNodeFEAcurv   {#manual_ChNodeFEAcurv}

- 9 coordinates (3 2nd order partial derivatives of the position vector, which represent curvatures)
  - 2nd order partial derivative of position with respect to just x
  - 2nd order partial derivative of position with respect to just y
  - 2nd order partial derivative of position with respect to just z
- Ex. used by these elements:
  - ChElementHexaANCF_3813_9 
  
  
# Theory

Additional information regarding the implementation of finite elements
in Chrono can be found at the  
[whitepapers page](http://projectchrono.org/whitepapers/).


# Examples

See demos and examples at the 
[tutorials](@ref tutorial_table_of_content_chrono_fea) page.





