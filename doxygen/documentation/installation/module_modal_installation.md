Install the MODAL module {#module_modal_installation}
==========================

[TOC]

Chrono::Modal enables **modal analysis** and **modal reduction** on large Chrono systems thanks to iterative shift-and-invert eigenvalue solvers leveraging sparse matrices.

An extended description of the module can be found in the [Chrono::Modal](@ref manual_modal) user manual.


## Requirements

- To **build** applications based on this module:
	- the [Spectra](https://spectralib.org/) library is needed

- To **run** applications based on this module there are no requirements


## Building instructions

1. Download the [Spectra](https://spectralib.org/) library from its GitHub page. You can either download the source code as a zip file, or else clone the git repository.
   **NOTE** we need the development version of the Spectra libray, as the only one that includes the Krylov-Schur solver Therefore, use this link: [https://github.com/yixuan/spectra/tree/develop](https://github.com/yixuan/spectra/tree/develop)
   
2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono).
  
3. Set `CH_ENABLE_MODULE_MODAL` to 'ON'.

4. Set the `SpectraINCLUDE_DIR` to the path to the include directory in the Spectra install.  For example, it could be `C:/Packages/spectra/include`.
 
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
