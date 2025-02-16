Install the PYTHON module {#module_python_installation}
==========================

[TOC]

This is an optional module that adds Python support in Chrono.

## Features

The **PYTHON module** allows users to use Chrono modeling, simulation, and visualization capabilities using [Python](http://www.python.org).

This module consists of *two* build targets:

- The Python modules for [PyChrono](@ref pychrono_introduction). <br>
  Python wrappers are generated for the following Chrono modules (if they are enabled during CMake configuration):
    - *pychrono*, which wraps most Chrono classes, equivalent to the chrono namespace
    - *pychrono.fea*, which wraps FEA classes, equivalent to the chrono::fea namespace.
    - *pychrono.vehicle*, which wraps the Chrono::Vehicle classes and ground vehicle models in the chrono::vehicle namespace.
    - *pychrono.robot*, which wraps the various models in the Chrono robotics models library.
    - *pychrono.sensor*, which wraps the Chrono::Sensor classes.
    - *pychrono.postprocess*, which wraps the Chrono::Postprocess module.
    - *pychrono.irrlicht*, which wraps the Chrono::Irrlicht module for run-time visualizatino.
    - *pychrono.pardisomkl*, which wraps the Chrono::PardisoMKL module (not available on the Mac Apple Silicon).
    - *pychrono.cascade*, which wraps the Chrono::Cascade module.
<br><br>
- A *PYPARSER module*, which is a C++ module for parsing / executing / interpreting 
  Python instructions from C++ programs.

<div class="ce-info">
**NOTE**: If you are only interested in using PyChrono, an alternative to building the Chrono Python module is to install the precompiled [PyChrono conda package](@ref pychrono_installation).
</div>

## Requirements

- To **run** applications based on this module:
    - you must have [Python3](http://www.python.org) installed. On the Mac load the actual Python distribution from https://www.python.org and install it. Set the appropriate environment variable. Don't use the python installed by homebrew.
    - to use the *pychrono.cascade* module you must also build and install the [pythonocc-core](https://github.com/tpaviot/pythonocc-core) package. For consistency with OpenCASCADE 7.4.0 (required to build Chrono::Cascade), make sure to use **pythonocc-core version 7.4.1**.
    
- To **build** this module:
    - you must have [Python3](http://www.python.org) installed.
    - you must have the [SWIG](http://www.swig.org/) wrapper generator installed.

<div class="ce-warning">
Building PyChrono requires SWIG version **4.0.0** or newer.
- On Windows, use a [SWIGWIN](https://sourceforge.net/projects/swig/files/swigwin) distribution which includes a pre-built executable. SWIG versions 4.0.2, 4.1.0, and 4.2.1 have been tested.
- On Linux, many distributions include packages of SWIG. Consult your package management application.
- On MacOS, SWIG can be installed with homebrew (<tt>brew install swig</tt>).
</div>

## Building instructions

1. Install SWIG on your system. Version 4.0.0 or newer is required. (on Windows, just unzip where you want).

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:

3. Set `CH_ENABLE_MODULE_PYTHON` as 'on', then press 'Configure' (to refresh the variable list) 

4. The Python package should be detected by CMake automatically.
If prompted, set the `CH_PYTHONDIR` variable to the directory where you have your copy of Python. 
   For example, it could be <tt>C:/Python33</tt>

5. If prompted, set the CMake <tt>SWIG_EXECUTABLE</tt> variable to specify the SWIG executable. 
   On Linux, this should be detected automatically. On Windows, depending on how SWIG was installed, you may need to set this manually. 

6. Press 'Configure' again, then 'Generate', and proceed as usual in the installation instructions.

If you have multiple Python installations on the same machine, you may need to explicitly specify which one to use in the call to CMake.  For example, under Linux (Mac is similar to Linux):

<pre>
% ccmake -DPYTHON_EXECUTABLE:FILEPATH=/usr/local/python/3.6.0/bin/python3
  -DPYTHON_LIBRARY=/usr/local/python/3.6.0/lib/libpython3.so
  -DPYTHON_INCLUDE_DIR=/usr/local/python/3.6.0/include
  ../../chrono
On the Mac:
% ccmake -DPYTHON_EXECUTABLE:FILEPATH=$(which python3) ../../chrono
</pre>

After successful compilation, the [PyChrono](@ref pychrono_introduction) modules can be used either from the *BUILD* tree or, after installation, from the *INSTALL* tree.  In order for the generated Python modules to be accessible, you must set/append to the <tt>PYTHONPATH</tt> environment variable.  During configuration, the Chrono CMake script will output the proper paths to be used in setting the PYTHONPATH environment variables; for example:

- Windows:<br>
<img src="http://www.projectchrono.org/assets/manual/ChronoPython_config.png" width="500">

- Linux:<br>
<img src="http://www.projectchrono.org/assets/manual/ChronoPython_config_linux.png" width="600">

Setting/changing environment variables is platform-specific.

- On Windows, you can (globally) set environment variables in 'Control Panel -> System -> Advanced system settings':<br>
  <img src="http://www.projectchrono.org/assets/manual/Windows_env.png" width="500"><br>

- On Linux, using bash shell:<br>
  <img src="http://www.projectchrono.org/assets/manual/Linux_env.png" width="600"><br>
  To permanently set <tt>PYTHONPATH</tt>, you can add the above to your .bashrc file (or the appropriate initialization file for your shell). 


## Usage
At runtime all systems need a few Python modules: <tt>numpy</tt> and <tt>six</tt>. Run this command:<br>
<tt>pip3 list</tt><br>
You should see something like this:<br>
<tt>Package    Version<br>
---------- -------<br>
numpy      1.22.3<br>
pip        22.0.4<br>
setuptools 58.1.0<br>
six        1.16.0</tt><br>
You can install the missing modules with:<br>
<tt>pip3 install numpy six</tt><br>

For more details on how to use the resulting modules, look here:

- C++ functions (as Python parser)
    - Look at the [API section](group__python__module.html) of this module for documentation about C++ functions.
    - Look at the C++ source of [demos](@ref tutorial_root) to learn how to use the C++ functions of this module.

- Python functions (as [PyChrono](@ref pychrono_introduction) )
    - Look at the [reference](@ref pychrono_reference) of PyChrono, about using Chrono from the Python side.
    - Look at the Python source of [demos](@ref tutorial_table_of_content_pychrono).

## Notes

The build process of the Python modules, as generated by CMake, consists of these automatic steps: 

- SWIG preprocesses C++ source code files in order to generate a .cxx 
  wrapper file that contains the code for accessing C++ functions through the 
  API, and some .py files,

- the C++ compiler compiles the .cxx file and generates one or more library files, 

- the SWIG-generated files (*.py) and resulting library files (*.pyd on Windows and *.so on Linux/Mac) are collected in a single location in the *BUILD* tree, so that they can be used directly from there.  Similarly, after installation, all Chrono::Python modules are copied in a single location in the *INSTALL* tree.  See the comments above about these exact locations for your particular configuration and about setting the <tt>PYTHONPATH</tt> environment variable.

<div class="ce-info">
This module is tested with Python3 (up to and including Python 3.10). 
Support for Python 2.7 is discontinued.
</div>

<div class="ce-warning">
In some distributions of Python, the debug library 'python33_d.lib' (the debug version of the python33.lib library) is not included by default. 
If you need it because you recompile the python module in debug mode, either you recompile the entire Python source, or you modify pyconfig.h to force the use of python33.lib by following these steps:
1. Comment out the line:
   ~~~{.py}
   //#define Py_DEBUG
   ~~~
2. Modify
   ~~~{.py}
   #if defined(_DEBUG)
   #pragma comment(lib,"python33_d.lib")
   ~~~
   to
   ~~~{.py}
   #if defined(_DEBUG)
   #pragma comment(lib,"python33.lib")
   ~~~
3. Press 'Advanced' in CMake, set the PYTHON_DEBUG_LIBRARY to the same lib that you have in PYTHON_LIBRARY, and press 'Generate' so that your project will link 'python33.lib' instead than 'python33_d.lib'.
</div>
