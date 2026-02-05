Chrono FEA tutorials {#tutorial_table_of_content_chrono_fea}
===========================

The following demos, included with the Chrono distribution, illustrate the use of the [FEA module](group__chrono__fea.html).

- [demo_FEA_basic](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_basic.cpp)  
  Learn the very basics: create mesh container, add nodes and elements, and perform linear static analysis (on single elements).
- [demo_IRR_visualize_FEA](https://github.com/projectchrono/chrono/blob/main/src/demos/irrlicht/demo_IRR_visualize_FEA.cpp) and [demo_VSG_visualize_FEA](https://github.com/projectchrono/chrono/blob/main/src/demos/vsg/demo_VSG_visualize_FEA.cpp)  
  Learn how to visualize FEA in the realtime view, also learn to use tetrahedrons and hexahedrons for nonlinear dynamics.
- [demo_FEA_cables](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_cables.cpp)  
  Learn how to create cables using ChElementCableANCF, and connect some bodies 
- [demo_FEA_beamsEuler](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_beamsEuler.cpp)  
  Learn how to create beams using ChElementBeamEuler 
- [demo_FEA_loads_dynamic](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_loads_dynamic.cpp) and [demo_FEA_loads_static](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_loads_static.cpp)  
  Here you can learn how to apply loads to elements, or how to create your custom loads.
- [demo_FEA_shellsANCF_3423_3443](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_shellsANCF_3423_3443.cpp)  
  Learn to use ANCF shell elements
- [demo_FEA_shellsReissner](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_shellsReissner.cpp)  
  Learn to use Reissner 6-field shell elements
- [demo_FEA_hexaANCF_3813](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_hexaANCF_3813.cpp) and [demo_FEA_hexaANCF_3813_9](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_hexaANCF_3813_9.cpp)  
  Learn to use ANCF brick elements
- [demo_FEA_contacts_SMC](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_contacts_SMC.cpp) and [demo_FEA_contacts_SMC](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_contacts_SMC.cpp)  
  How to perform contact between FEA meshes in three dimensional space, with automatic collision detection
- [demo_FEA_abaqus_wheel](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_abaqus_wheel.cpp)  
  Import an .INP Abaqus mesh with a 3D tetrahedral mesh
- [demo_FEA_cosimulate_load](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_cosimulate_load.cpp)  
  Import an .INP Abaqus mesh with a 3D tetrahedral mesh and apply loads to the surface coming from an external process.
- [demo_FEA_thermal](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_thermal.cpp)  
  Compute a simple solution for the temperature field in a solid, given boundary conditions (Poisson problem).
- [demo_FEA_electrostatics](https://github.com/projectchrono/chrono/blob/main/src/demos/fea/demo_FEA_electrostatics.cpp)  
  Compute the electrostatic solution for a volume given boundary conditions (high voltage electrodes in air).