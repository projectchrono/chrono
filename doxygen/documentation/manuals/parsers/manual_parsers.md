Chrono::Parsers {#manual_parsers}
=================================

This module enables:
- importing models from description files
  + URDF (requires external library)
  + OpenSim
  + Adams
  + custom input for robots
- the interaction with Python language (requires Python interpreter)

On the contrary, the import of **finite elements** comes through the @ref chrono::fea::ChMeshFileLoader "ChMeshFileLoader" class, available in the main Chrono library (so no need for the PARSERS module).  
This class enables the import of Abaqus, TetGen, GMF (LS-DYNA) or generic OBJ files, but only for very specific elements and shall not be considered a full-fledged FE import tool.

## Python Engine (is not PyChrono)
The @ref chrono::parsers::ChPythonEngine "ChPythonEngine" class allows to run Python code directly from C++ and to interact with it, also exchanging data from/to variables.  
It should not be confused with [PyChrono](@ref pychrono_introduction) that is a completely stand-alone Python library that wraps Chrono C++ code.

+ @ref chrono::parsers::ChPythonEngine "ChPythonEngine" allows to run generic Python code from C++
+ [PyChrono](@ref pychrono_introduction) allows to use Chrono from Python

Through this class the user can also import models exported from SolidWorks to Python, by means of the method @ref chrono::parsers::ChPythonEngine::ImportSolidWorksSystem() "ChPythonEngine::ImportSolidWorksSystem()". Please refer to the [Chrono::SolidWorks reference manual](@ref manual_chrono_solidworks) for additional information. Please just be aware that this is **not the only way** to import SolidWorks models.

Demo about generic Python interaction: [demo_PARSER_Python.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/parsers/demo_PARSER_Python.cpp)



[Install guide](@ref module_parsers_installation)





