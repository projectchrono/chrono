
Introduction to modules        {#modularity}
==============================

Chrono::Engine is organized as a set of [modules](@ref modules).

**Modules** are additional libraries that can be _optionally_ used 
to expand the features of Chrono::Engine. 

In this sense, the Chrono::Engine framework is a modular concept 
that can be expanded or simplified, depending on user's needs.

There are various motivations for modularity:

* the compilation of modules, usually, depends on additional 
  software components that require some installation and configuration 
  from the user (for instance, the MATLAB module requires the Matlab API to be installed, etc.) 
  We do not want to force the user to have all these prerequisites - one can compile 
  and use only the modules that fit into his system;

* splitting the project in smaller components avoid dealing with a monolithic, huge dll;

* further modules could be developed in future without changing the core library of the project.

For these and other reasons, we decided to make the compilation of the modules *conditional*: 
the user can enable their compilation -if interested-, otherwise disable them to make 
the build process easier and with minimum requirements.

In the following picture one can get an idea of how modules can depend on external libraries, 
whereas the core system of Chrono::Engine just depends on the plain operating system.

![](Units.png)

A list of the available modules, along with informations on how to compile them, 
can be found in the [modules page](@ref modules) of the [manual](@ref manual_root).

