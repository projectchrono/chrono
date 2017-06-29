
Introduction to modules        {#modularity}
==============================

Chrono is organized as a set of [modules](modules.html).

**Modules** are additional libraries that can be _optionally_ used 
to expand the features of Chrono. In this sense, the Chrono framework is a modular concept 
that can be expanded or simplified depending on a user's needs.

Benefits of modularity:

* The compilation of modules usually depends on additional 
  software components that might not be free and/or require extra installation and configuration 
  effort. For instance, the MATLAB module requires the Matlab API to be installed.
  In this context, Chrono does not force the user to have all these prerequisites - one can compile 
  and use only the modules that are strictly needed by the user.

* Splitting the project into smaller components avoids producing a monolithic, huge dll;

* Further modules could be developed in the future without changing the core library of the project.

The Chrono module compilation is thus *conditional*: 
The user can enable a module compilation.
Otherwise, the module is disabled, which keeps the build process clean and simple.

The picture below illustrates how modules can depend on external libraries, 
whereas the core system of Chrono depends on the underlying operating system only.

![](http://www.projectchrono.org/assets/manual/Units.png)

<br/>

A list of the available modules, along with information on how to compile them, 
can be found in the [modules page](modules.html) of the [manual](@ref manual_root).

