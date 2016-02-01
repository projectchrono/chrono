---
layout: default
title: About Units
permalink: /documentation/chrono_engine/units/
---
**Units** are additional libraries that can be *optionally* used to
expand the features of Chrono::Engine. In this sense, the Chrono::Engine
framework is a modular concept that can be expanded or simplified,
depending on user's needs.

There are various motivations for modularity:

-   the compilation of units, usually, depends on additional software
    components that require some installation and configuration from the
    user (for instance, the unit for GPU computation requires that the
    user has the CUDA SDK installed in his system, the Matlab unit
    requires the Matlab API to be installed, etc.) We do not want to
    force the user to have all these prerequisites - one can compile and
    use only the units that fit into his system;

-   splitting the project in smaller components avoid dealing with a
    monolithic, huge dll;

-   further units could be developed in future without changing the core
    library of the project.

For these and other reasons, we decided to make the compilation of the
units *conditional*: the user can enable their compilation -if
interested-, otherwise disable them to make the build process easier and
with minimum requirements.

In the following picture one can get an idea of how units can depend on
external libraries, whereas the core system of Chrono::Engine just
depends on the plain operating system.

![](/images/Units.png "Units.png")

A list of the available units, along with informations on how to how to
compile them, can be found [ here](Main_Page "wikilink").

A list of tutorials for the available units can be found [
here](/documentation/tutorials/).
