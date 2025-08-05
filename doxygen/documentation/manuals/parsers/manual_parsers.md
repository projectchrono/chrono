Chrono::Parsers {#manual_parsers}
=================================

This module enables:
- specifying Chrono models and simulations in YAML
- importing models from 3rd-party specification files
  + URDF (requires external library)
  + OpenSim
  + Adams
- interacting with Python language (requires Python interpreter)

Note that import of fnite element models (meshes) can be done using the @ref chrono::fea::ChMeshFileLoader "ChMeshFileLoader" class, available in the main Chrono module (so no need for the PARSERS module). This class enables the import of Abaqus, TetGen, GMF (LS-DYNA) or generic OBJ files, but only for very specific elements and is not a full-fledged FE import tool.

[Installation guide for Chrono::Parsers](@ref module_parsers_installation)

## YAML parser

The @ref chrono::parsers::ChParserYAML class allows parsing YAML specification files for Chrono models and simulations. The parser caches model information and simulation settings from the corresponding YAML input files and then allows populating a Chrono system and setting solver and simulation parameters.

* @subpage YAML_parser_overview
* @subpage YAML_schema_models
* @subpage YAML_schema_simulations


## Python Engine (is not PyChrono)
The @ref chrono::parsers::ChPythonEngine "ChPythonEngine" class allows to run Python code directly from C++ and to interact with it, also exchanging data from/to variables.  
It should not be confused with [PyChrono](@ref pychrono_introduction) that is a completely stand-alone Python library that wraps Chrono C++ code.

+ @ref chrono::parsers::ChPythonEngine "ChPythonEngine" allows to run generic Python code from C++
+ [PyChrono](@ref pychrono_introduction) allows to use Chrono from Python

Through this class the user can also import models exported from SolidWorks to Python, by means of the method @ref chrono::parsers::ChPythonEngine::ImportSolidWorksSystem() "ChPythonEngine::ImportSolidWorksSystem()". Please refer to the [Chrono::SolidWorks reference manual](@ref manual_chrono_solidworks) for additional information. Please just be aware that this is **not the only way** to import SolidWorks models.

Demo about generic Python interaction: [demo_PARSER_Python.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/parsers/demo_PARSER_Python.cpp)



[Install guide](@ref module_parsers_installation)





