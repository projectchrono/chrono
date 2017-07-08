
List of FEA elements      {#manual_fea_elements}
=======================

\tableofcontents


Different types of finite elements can be used in the [FEA module](@ref manual_fea).
In this page you can find a description of their properties. 

- An **element** connects N nodes. 
- Nodes are handled via std::shared_ptr shared pointers: you do not have to worry about deleting them.
- Add elements to a mesh using ChMesh::AddElement()
- Initialize the elements by telling which node are connected with SetNodes()
- Set a material property to the element by using SetMaterial()

  
# ChElementSpring    {#manual_ChElementSpring}

![](http://www.projectchrono.org/assets/manual/fea_ChElementSpring.png)

- 2 nodes of ChNodeFEAxyz type
- Large displacements allowed
- Zero mass element
- Parameters: 
  - rest length L, 
  - stiffness k, 
  - damping r
- Note: geometric stiffness not (yet) implemented
- The simpliest element: a starting point to learn how to implement finite elements


# ChElementBar    {#manual_ChElementBar}

![](http://www.projectchrono.org/assets/manual/fea_ChElementBar.png)

- 2 nodes of ChNodeFEAxyz type
- Very similar to ChElementSpring, except it has a mass
- No torque at the ends (like two spherical joints)
- Large displacements allowed
- Parameters: 
  - rest length L, 
  - Section area A, 
  - Young modulus E, 
  - damping 
- Note: geometric stiffness not (yet) implemented




# ChElementTetra_4    {#manual_ChElementTetra_4}

![](http://www.projectchrono.org/assets/manual/fea_ChElementTetra_4.png)

- 4 nodes of ChNodeFEAxyz type
- Linear interpolation, constant stress
- 1 integration point
- Corotational formulation for large displacements
- Uses polar decomposition for corotated frame
- Useful for solids
- Fastest element for solids


  
  
# ChElementTetra_10    {#manual_ChElementTetra_10}

![](http://www.projectchrono.org/assets/manual/fea_ChElementTetra_10.png)

- 10 nodes of ChNodeFEAxyz type
- Quadratic interpolation, linear stress 
- 4 integration points
- Corotational formulation for large displacements
- Uses polar decomposition for corotated frame
- Note: initial position assuming nodes n>4 exactly at mid-length of edges
- Useful for solids 


  
  
# ChElementHexa_8    {#manual_ChElementHexa_8}

![](http://www.projectchrono.org/assets/manual/fea_ChElementHexa_8.png)

- 8 nodes of ChNodeFEAxyz type
- Linear interpolation
- 8 integration points
- Corotational formulation for large displacements
- Useful for solids, with structured grids


  
# ChElementHexa_20   {#manual_ChElementHexa_20}

![](http://www.projectchrono.org/assets/manual/fea_ChElementHexa_20.png)

- 20 nodes of ChNodeFEAxyz type 
- 8 at vertexes, 12 at edges midpoints
- Quadratic interpolation
- 27 integration points
- Corotational formulation for large displacements
- Useful for solids, with structured grids


# ChElementBrick    {#manual_ChElementBrick}

![](http://www.projectchrono.org/assets/manual/fea_ChElementHexa_8.png)

- 8 nodes of ChNodeFEAxyz type
- Linear interpolation
- 8 integration points
- Use EAS Enhanced Assumed Strain
- Large strains 
- Can use Mooney-Rivlin model for hyperelastic materials
- Useful for solids, with structured grids


# ChElementBrick_9    {#manual_ChElementBrick_9}

- 9 nodes of ChNodeFEAxyz type (8 at the corners, 1 at the center)
- Linear interpolation
- 8 integration points
- Strain formulations for large strains:
  - Green-Lagrange
  - Hencky      
- Plasticity:
  - J2 (metals)
  - DruckerPrager (soil, plastics)
  - DruckerPrager_Cap (soil, plastics)
- Useful for solids, with structured grids


  
# ChElementCableANCF   {#manual_ChElementCableANCF}

![](http://www.projectchrono.org/assets/manual/fea_ChElementCableANCF.png)

- 2 nodes of @ref chrono::fea::ChNodeFEAxyzD type
- 3 integration point (stiffness), 4 (mass)
- ANCF formulation for large displacements
- Thin beam (no shear)
- Does not model torsional stiffness (useful for wires, cables)
- Section property: A, I, E, density, damping



# ChElementBeamEuler   {#manual_ChElementBeamEuler}

![](http://www.projectchrono.org/assets/manual/fea_ChElementBeamEuler.png)
![](http://www.projectchrono.org/assets/manual/fea_ChElementBeamEuler_section.png)

- 2 nodes of ChNodeFEAxyzrot type
- Linear interpolation
- 1 integration point (default)
- Corotational formulation for large displacements
- Thin beam (no shear), based on the Euler-Bernoulli thin beam theory 
- Section property: 
  - A, Iyy, Izz, E, density, damping
  - G, J   for torsional stiffness, plus optional:
  - αe , ze , ye ,  for offset/rotated section
  - zs , ys          for offset shear center



# ChElementBeamANCF   {#manual_ChElementBeamANCF}

![](http://www.projectchrono.org/assets/manual/fea_ChElementBeamANCF.png)

- 2 nodes of ChNodeFEAxyzDD type
- ANCF formulation for large displacements
- [recent feature – beta testing]


  
# ChElementShellReissner   {#manual_ChElementShellReissner}

![](http://www.projectchrono.org/assets/manual/fea_ChElementShellReissner.png)

- 4 nodes of ChNodeFEAxyzrot type
- Bi-linear interpolation
- 4 integration points (default)
- Allows large displacements, exponential map used for SO3
- Thick shells allowed
- Based on the Reissner 6-field shell theory (w. drilling stiffness)
- Can have multi-layered materials, using CLT thory
- ANS, shear-lock free
- Nodes need not to be aligned to shell (rotation offsets auto-computed in initialization)



  
# ChElementShellANCF   {#manual_ChElementShellANCF}

![](http://www.projectchrono.org/assets/manual/fea_ChElementShellANCF.png)

- 4 nodes of ChNodeFEAxyzD type
- Bi-linear interpolation
- 4 integration points (default)
- Allows large displacements, using ANCF formulation
- Thick shells allowed
- Can have multi-layered materials
- ANS-EAS, shear-lock free
- Nodes D must be aligned to shell normal at initialization








# Theory

Additional information regarding the implementation of finite elements
in Chrono can be found at the  
[whitepapers page](http://projectchrono.org/whitepapers/).


# Examples

See demos and examples at the 
[tutorials](@ref tutorial_table_of_content_chrono_fea) page.





