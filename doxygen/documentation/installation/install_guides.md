Installation Guides {#install_guides}
==========================

### Installing Chrono (C++)

-   [Core Chrono module](@ref tutorial_install_chrono)

Additional Chrono functionalities are provided through optional _modules_, conditionally enabled during CMake configuration. Modules often rely on third-party libraries that might require additional installation steps, as described in each module installation page.

-   [CASCADE module](@ref module_cascade_installation)

-   [COSIMULATION module](@ref module_cosimulation_installation)

-   [CSHARP module](@ref module_csharp_installation)

-   [FMI module](@ref module_fmi_installation)

-   [FSI module](@ref module_fsi_installation)

-   [GPU module](@ref module_gpu_installation)

-   [IRRLICHT module](@ref module_irrlicht_installation)

-   [MATLAB module](@ref module_matlab_installation)

-   [MODAL module](@ref module_modal_installation)

-   [MULTICORE module](@ref module_multicore_installation)

-   [MUMPS module](@ref module_mumps_installation)

-   [OPENGL module](@ref module_opengl_installation)

-   [Pardiso MKL module](@ref module_mkl_installation)

-   [Pardiso Project module](@ref module_pardisoproject_installation)

-   [PARSERS module](@ref module_parsers_installation)

-   [POSTPROCESS module](@ref module_postprocess_installation)

-   [PYTHON module](@ref module_python_installation)

-   [SENSOR module](@ref module_sensor_installation)	

-   [SYNCHRONO module](@ref module_synchrono_installation)

-   [VEHICLE module](@ref module_vehicle_installation)

-   [VSG module](@ref module_vsg_installation)


For some of these dependencies, we provide utility scripts that will download, configure, build, and install versions that are known to work with the current Chrono distribution. In each case, we provide both batch scripts (for Windows users) and bash scripts (for Linux/Mac users). Currently, utility scripts for the following sets of dependencies are available (under the `contrib/build-scripts` subdirectory of the Chrono source tree):

- VSG libraries required for the Chrono::VSG module
- URDF libraries required by the URDF parser in the Chrono::Parsers module
- GL utility libraries required for the Chrono::OpenGL and (optionally) Chrono::Sensor modules
- OpenCRG library required for the optional OpenCRG support in the Chrono::Vehicle module

The directory `contrib/build-scripts` also includes sample scripts (`buildChrono.bat` and `buildChrono.sh`) for configuring Chrono with CMake which can be used as examples of satisfying the dependencies for the various optional Chrono modules (assuming these dependencies were installed with the utility scripts described above).

### Building a project that uses Chrono

User projects should start from the template offered in the repository under: [_chrono/template_project_](https://github.com/projectchrono/chrono/tree/main/template_project).

Full instructions can be found in: [Linking to Chrono](@ref tutorial_install_project)

### Building Chrono for WebAssembly

-   [Build Chrono for WASM](@ref tutorial_install_chrono_emscripten)


### Installing PyChrono

- @subpage pychrono_installation


### Chrono::Solidworks add-in

An add-in to enable the export of models from SolidWorks to Chrono

- @subpage tutorial_install_chrono_solidworks
