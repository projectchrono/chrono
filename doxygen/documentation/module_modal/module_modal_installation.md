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

The **MODAL module** is used to perform modal analysis for various needs, for example:

- compute the eigenvalues and eigenvectors of a structure
	- the method can return only the lower n modes, in case the structure has too many DOFs
	- an iterative method is used, exploiting sparse matrices
	- structures can contain both elastic parts (ex. Chrono finite elements) and constraints (joints, revolute pairs, etc.)
- compute complex eigenvalues and eigenvectors, with damped structures
	- return also the damping factor, damped and undamped frequencies
	- stability of the structure, from the sign of the damping factor 
	- often needed in aeroelasticity
- simplify a complex subassembly into a *modal assembly*, by performing linearization of the sub assembly and by selecting a limited number of modes 
	- use Harting modal reduction
	- large rotations and rotations are allowed for modal subassemblies
	- unlimited number of modal assemblies


## Requirements

- To **run** applications based on this module yo need no special library 

- To **build** applications applications based on this module you must have the SPECTRA library.




## Building instructions

The SPECTRA library is a header-only c++ library, so it does not need complex building steps. Just put its source on your computer and inform CMAKE where it is.
   
1. **download** [SPECTRA from https://github.com/yixuan/spectra/tree/develop](https://github.com/yixuan/spectra/tree/develop) . 
   - You can use the Download .zip button in the GIThub page and unzip somewhere, 
   - or you can use a GIT client such as Sourcetree, to **clone it**.
   NOTE! here we refer to the *develop* branch (from October 2021 onward) because SPECTRA v.1.0 does not contain the Krylov-Shur solver that we use in our module.

2. For example, here we suppose that you unzipped it in `C:/engine_demos/spectra`.

3. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
   
4. Set the `ENABLE_MODULE_MODAL` as 'on', then press 'Configure' (to refresh the variable list) 
 
5. Set the directory in `SPECTRA_INCLUDE_DIR`: it must contain the path to your SPECTRA directory.  
   In our example, browse to `C:/engine_demos/spectra/include`
   
7. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.


## How to use it

- Look at the [API section](group__modal__module.html) of this module for documentation about classes and functions.

- Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the functions of this module.
