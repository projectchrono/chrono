Install the MODAL module {#module_modal_installation}
==========================

[TOC]

This module allows Chrono to perform **modal analysis** and **modal reduction** also on large Chrono systems thanks to iterative shift-and-invert eigenvalue solvers leveraging sparse matrices.

An extended description of the module can be found in the [Chrono::Modal](@ref manual_modal) user manual.

Read [the introduction to modules](modularity.html) for a technical background on the modularity of the Chrono project.


## Requirements

- To **build** applications based on this module:
	- the [Spectra](https://spectralib.org/) library is needed

- To **run** applications based on this module there are no requirements


## Building instructions

1. Download the [Spectra](https://spectralib.org/) library from its GIThub page. This can be done directly by pressing *Code/Download Zip* in the GIThub page, 
   or by using a client like Sourcetree to pull from the GIThub repository. 
   **NOTE! we need the development version of the Spectra libray, because the 1.0 version in the master branch does not include the Krylov-Schur solver yet. Therefore, use this link: [https://github.com/yixuan/spectra/tree/develop](https://github.com/yixuan/spectra/tree/develop)**
   
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:
  
3. Set the `ENABLE_MODULE_MODAL` as 'ON', then press 'Configure' (to refresh the variable list)

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

- Look at the C++ source of [demos](https://github.com/projectchrono/chrono/tree/main/src/demos/modal) to learn how to use the functions of this module.
