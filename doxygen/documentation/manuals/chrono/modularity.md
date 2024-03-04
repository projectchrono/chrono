
Introduction to Modules {#modularity}
==============================

Chrono is organized in modules.

Each **module** consists in additional libraries that can be _optionally_ used 
to expand the features of Chrono, depending on users needs.

This allows to:

* reduce the number of dependencies to the minimum, requiring only those strictly needed by the specific user;
* have smaller size libraries, instead of a big ones;
* have a better isolation during the development

Each module can be enabled/disabled through the corresponding `ENABLE_MODULE_XXX` in the CMake configuration.

The picture below illustrates how modules can depend on external libraries, 
whereas the core system of Chrono depends on the underlying operating system only.

![](http://www.projectchrono.org/assets/manual/Units.png)


