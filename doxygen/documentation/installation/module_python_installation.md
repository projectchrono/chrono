Install the PYTHON module {#module_python_installation}
==========================

[TOC]

This is an optional module that adds Python support in Chrono.

## Features

The **PYTHON module** allows users to use [Python](http://www.python.org) to creating simulations. This module is used to build the [PyChrono](@ref pychrono_introduction) wrapper.

This module consists of *two* main sets of build targets:

- The Python modules for [PyChrono](@ref pychrono_introduction). 
  Currently, the PyChrono Python modules that are built are:
    - *pychrono*, which wraps most Chrono classes, equivalent to the chrono namespace
    - *pychrono.fea*, which wraps FEA classes, equivalent to the chrono::fea namespace.
    - *pychrono.postprocess*, which wraps the POSTPROCESS module.
    - *pychrono.irrlicht*, which wraps the IRRLICHT module.
    - *pychrono.pardisomkl*, which wraps the Pardiso MKL module (not available on the Mac).
    - *pychrono.cascade*, which wraps the CASCADE module.

- A *PYPARSER module*, which is a C++ module for parsing / executing / interpreting 
  Python instructions from C++ programs.

<div class="ce-info">
    Note: If you are only interested in using PyChrono, an alternative to building the Chrono Python module is to install the precompiled [PyChrono conda package](@ref pychrono_installation).
</div>

## Requirements

- To **run** applications based on this module:
    - you must have [Python](http://www.python.org) installed. On the Mac load the actual Python distribution from https://www.python.org and install it. Set the appropriate environment variable. Don't use the python installed by homebrew.
    
- To **build** this module:
    - you must have [Python](http://www.python.org) installed,
    - you must have the [SWIG](http://www.swig.org/) wrapper generator installed. On the Mac type <tt>brew install swig</tt>.

<div class="ce-warning">
Building PyChrono requires SWIG version **4.0.0* or newer.
On the Mac **SWIG v. 4.0.1** is known to work. It is installed by homebrew.
</div>

## Building instructions

1. Install SWIG on your system. Version 4.0.0 or newer is required. (on Windows, just unzip where you want).

2. Repeat the instructions for the [full installation](@ref tutorial_install_chrono), but when you see 
   the CMake window, you must add the following steps:

3. Set the `ENABLE_MODULE_PYTHON` as 'on', then press 'Configure' (to refresh the variable list) 

4. If prompted, set the `CH_PYTHONDIR` variable to the directory where you have your copy of Python. 
   For example, it could be <tt>C:/Python33</tt>

5. When you pressed 'Configure', CMake should have detected your SWIG tools. This should create two variables in CMake called <tt>SWIG_DIR</tt> and <tt>SWIG_EXECUTABLE</tt>. 
   If all is fine, they should automatically contain meaningful values, 
   for example <tt>C:/swigwin-4.0.0/Lib</tt> and <tt>C:/swigwin-4.0.0/swig.exe</tt> , 
   so you do not need to touch them. (On Windows, you may need to set them by hand).

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
Note that the SWIG tool requires a few minutes to process the source 
and generate the .cxx wrapper file. When you start the compilation of the entire 
Chrono project, the process might look 'frozen' 
for one or two minutes when SWIG does its job. 
</div>

<div class="ce-info">
This module is tested with Python 3.3 and 3.2. 
Support of the previous version Python 2.7 is discontinued.
</div>

<div class="ce-warning">
If you installed Python for 32 bit, you must compile 
Chrono in 32 bit mode. If you installed 
Python for 64bit, you must compile Chrono in 64 bit mode. 
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
