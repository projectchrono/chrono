Install the MODAL module {#module_modal_installation}
==========================

[TOC]

This module allows Chrono to perform modal analysis and related functionalities.
For example you can use finite elements to model a beam and you can use this module to compute the natural frequencies of the structure. 
The module uses the iterative methods in the SPECTRA library in order to compute the lower modes 
in case the structure has too many DOFs: in fact the default methods offered by the EIGEN linear 
algebra library (default in Chrono) are not able to work with large sparse matrices and limited n of required modes.

Read [the introduction to modules](modularity.html) for a technical 
background on the modularity of the Chrono project.


## Features

The new **modal module** provides functionalities to do modal analysis and modal reduction. In detail, the class `ChModalAssembly` offer three main functionalities:

- **undamped modal analysis** of all the system being created within the sub assembly will be obtained. The modes and frequencies can be also displayed interactively if using the Irrlicht visualization system. 
	- Structures can contain both elastic parts (ex. Chrono finite elements) and constraints (joints, revolute pairs, etc.)
	- Rigid modes (for free-free structures) are supported
	- A custom genaralized, sparse, constrained eigenvalue solver of Krylov-Schur type allows the computation of only the n lower modes. This allows handling large FEA systems. 
	
- **damped (complex) modal analysis** of the subsystem: this is like the previous case, but damping matrix is used too, hence:
	- Obtain complex eigenvalues/eigenvectors. 
	- Return also the damping factor, damped and undamped frequencies.
	- Damping factors for the modes are output too, indicating stability or instability. 

- **modal reduction** of the subassembly. Example of a scenario where this is useful: you have a tower modeled with thousands of finite elements, but you are just interested in the small oscillations of its tip, because you will mount a windmill on its tip. If you simulate thousands of finite elements just for this purpose, you waste CPU time, hence a modal reduction of the tower will discard all the DOFs of the finite elements and represent the overall behaviour of the tower using just few modal shapes (ex. fore aft bending, lateral bending, etc.), with extreme CPU performance at the cost of a small reduction of fidelity.
	- Bodies and FEA nodes can be added to the subassebly as *internal*  or *boundary* interface nodes. Later one can call `ChModalAssembly::SwitchModalReductionON(int n_modes)` to replace the complexity of the internal nodes with few `n_modes` modal coordinates.
	- Boundary interface nodes can be connected to the rest of the multibody system as usual, using constraints, forces, etc.
	- Internal constraints can be used between internal nodes. Their effect too will be condensed in the modal reduction.
	- *NOTE: at the moment only linear dynamics is supported for the subassembly, in the sense that the subassembly cannot withstand large rotations, ex. in a helicopter blade. Future developments will address this*


## Requirements

- To **run** applications based on this module there are no requirements

- To **build** applications based on this module:
	- the [Spectra](https://spectralib.org/) library is needed



## Building instructions

1. Download the [Spectra](https://spectralib.org/) library from its GIThub page. This can be done directly by pressing *Code/Download Zip* in the GIThub page, 
   or by using a client like Sourcetree to pull from the GIThub repository. 
   **NOTE! we need the development version of the Spectra libray, because the 1.0 version in the master branch do not yet include the Krylov-Schur solver. Therefore, use this address: [https://github.com/yixuan/spectra/tree/develop](https://github.com/yixuan/spectra/tree/develop)**
   
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
  
3. Set the `ENABLE_MODULE_MODAL` as 'on', then press 'Configure' (to refresh the variable list)

4. Set the `SPECTRA_INCLUDE_DIR` to the path where you downloaded your Spectra library (in detail, its include/ directory).
   For example, it could be `C:/engines/spectra/include`
 
5. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

<div class="ce-info">
The `develop` branch of Spectra is under development. You might get some warnings during the build process. 
</div>

<div class="ce-info">
While we wait that future versions of Spectra will enable complex eigenvalues in the Krylov-Schur solver, a more conventional solver is used for the case of damped complex modal analysis. The more conventional solver is not sparse, so expect more CPU time and memory usage. The usual modal analysis, instead, already uses Krylov-Schur so it can support large problems.
</div>

## How to use it

- Look at the [API section](group__modal__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_table_of_content_chrono_matlab) to learn how to use the functions of this module.
