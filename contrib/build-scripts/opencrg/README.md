# Enabling OpenCRG support

Chrono::Vehicle has the ability of using road geometry specified through a `crg` file. The data format is very compact. Roads of arbitrary length can be handled. If you want to use it you have to build the OpenCRG support library before building Chrono. 

Get the distribution here: http://www.opencrg.org. The most current version is 1.1.2. 

Mac and Linux users can build the OpenCRG library using the provided makefiles. 

For Windows users, we provide the script `buildOpenCRG.bat`. To use it, you need the Visual Studio `cl` compiler. First, edit the script to specify the location of the OpenCRG distribution and the installation directory. Enable the linking to static MSVC runtime libraries if you are going to do so for Chrono. Note that you must run this script from a VS developer command prompt or PowerShell. 

# Why should anybody use OpenCRG files?

Chrono::Vehicle has to calculate the wheel/road contact points very often. Since the OpenCRG file act like an equidistant ordered mesh the calculation time is deterministic independent of the road length, width and resolution. A vehicle course line can easily be generated from the crg. Also the 3d surface definition is contained, so no additional obj file is needed.

The standard of chrono is to use unordered meshs that make the calculation time grow with mesh size and complexity. Some tasks need to use adaptive mesh refinement, in this cases OpenCRG is not the right tool to use. Actually crg files cannot be used for tiled terrain surfaces.

If you want to generate crg files, please use Matlab. It is the safest way to do because it allows pretty good error and consistency checking and generates binary format.

