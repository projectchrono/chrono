Install Chrono Solidworks {#chrono_solidworks_installation}
==========================

Chrono::SolidWorks is an add-in for [SolidWorks](http://www.solidworks.com) that allows to simulate and export SolidWorks models by leveraging the Chrono library.

Please refer to [Chrono::SolidWorks User Manual](@ref manual_chrono_solidworks) for the complete documentation.

Installation
------------

- a copy of [SolidWorks](http://www.solidworks.com) must be installed on your computer. Supported version 2011 or later, 64bit.
- download the [Chrono::SolidWorks Installer](http://projectchrono.org/download/#chronosolidworks) and install it;
- this allows to *simulate models directly from the SolidWorks interface*;
- exporting to Python, C++ or JSON format is not needed but *always possible*.
  In this case it is required to have PyChrono or Chrono (C++) to import them.  
  In particular, model exported in
  - Python can be loaded by:
    - Chrono (C++), if the [PARSERS module](@ref manual_parsers) and at least `ENABLE_MODULE_PYTHON` is enabled or PyChrono is available in the system;
    - PyChrono, if available in the system;
  - C++ requires Chrono (C++) to be available, this option requires to compile C++ code;
  - JSON requires Chrono (C++); currently the import from JSON is not supported in other languages;

How to use it
-------------

Please refer to [Chrono::SolidWorks User Manual](@ref manual_chrono_solidworks) for the complete documentation.  
Learn by example with the [Chrono::SolidWorks Tutorials](@ref tutorial_table_of_content_chrono_solidworks).