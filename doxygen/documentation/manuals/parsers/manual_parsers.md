Chrono::Parsers {#manual_parsers}
=================================

This module enables:
- specifying Chrono models and simulations in YAML
- importing models from 3rd-party specification files
  + URDF (requires external library)
  + OpenSim
  + Adams
- interacting with Python language (requires Python interpreter)

Note that import of finite element models (meshes) can be done using the @ref chrono::fea::ChMeshFileLoader "ChMeshFileLoader" class, available in the main Chrono module (so no need for the PARSERS module). This class enables the import of Abaqus, TetGen, GMF (LS-DYNA) or generic OBJ files, but only for very specific elements and is not a full-fledged FE import tool.

[Installation guide for Chrono::Parsers](@ref module_parsers_installation)

## YAML parsers

The various YAML parser classes allows parsing YAML specification files for Chrono models and simulations. The parsers cache model information and simulation settings from the corresponding YAML input files and then allows populating a Chrono, Chrono::Vehicle, or Chrono::FSI system and setting solver and simulation parameters.

* @subpage YAML_parser_overview
* @subpage YAML_schema_mbs_simulation
  * @subpage YAML_schema_mbs_model
  * @subpage YAML_schema_mbs_solver
* @subpage YAML_schema_vehicle_simulation
  * @subpage YAML_schema_vehicle_model
* @subpage YAML_schema_fsi_simulation
  * @subpage YAML_schema_fsisph_simulation
    * @subpage YAML_schema_fsisph_model
    * @subpage YAML_schema_fsisph_solver
  * @subpage YAML_schema_fsitdpf_simulation
    * @subpage YAML_schema_fsitdpf_model
    * @subpage YAML_schema_fsitdpf_solver


## Python Engine (not PyChrono)
The @ref chrono::parsers::ChPythonEngine "ChPythonEngine" class allows running Python code directly from C++ and to interact with it by exchanging data from/to variables.  
It should not be confused with [PyChrono](@ref pychrono_introduction) that is a completely stand-alone Python library that wraps Chrono C++ code:

+ @ref chrono::parsers::ChPythonEngine "ChPythonEngine" allows to run generic Python code from C++
+ [PyChrono](@ref pychrono_introduction) allows to use Chrono from Python

Through this class the user can also import models exported from SolidWorks to Python, by means of the method @ref chrono::parsers::ChPythonEngine::ImportSolidWorksSystem() "ChPythonEngine::ImportSolidWorksSystem()". Please refer to the [Chrono::SolidWorks reference manual](@ref manual_chrono_solidworks) for additional information. Please just be aware that this is **not the only way** to import SolidWorks models.

Demo about generic Python interaction: [demo_PARSER_Python.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/parsers/demo_PARSER_Python.cpp)



[Install guide](@ref module_parsers_installation)





