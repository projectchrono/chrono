# Enabling OpenCRG support

Chrono::Vehicle has the ability of using road geometry specified through a `crg` file. The data format is very compact. Roads of arbitrary length can be handled. If you want to use it you have to build the OpenCRG support library before building Chrono. 

Get the distribution here: http://www.opencrg.org. The most current version is 1.1.2. For Mac and Linux you can use the provided makefiles. 

Alternatively, you can use the `CMakeLists.txt` file in this directory, which should also work for Windows.

1. Copy OpenCRG.1.1.2.zip to a location of your coice
2. Unpack the zip archive: 
   - [Linux, Mac] `unzip -a OpenCRG.1.1.2.zip`
   - [Windows] use explorer or a 3rd party tool such as 7-zip 
3. Copy the `CMakeLists.txt` provided here into `/your/path/OpenCRG.1.1.2/c-api`
4. Configure: 
   `cmake /your/path/OpenCRG.1.1.2/c-api` 
5. Build it:
   - [Linux, Mac] `make`
   - [Windows] use the native tool specified during configuration (Nmake, Visual Studio, etc.)
6. The generated files are:
   - header: `CrgBaseLib.h`
   - library: `libOpenCRG.a` [Linux, Mac], `OpenCRG.lib` [Windows]
7. Install the files
8. When you configure Chrono let the configure script point to the needed OpenCRG files

On all platforms, the provided CmakeLists.txt build OpenCRG as a static library.

# Why should anybody use OpenCRG files?

Chrono::Vehicle has to calculate the wheel/road contact points very often. Since the OpenCRG file act like an equidistant ordered mesh the calculation time is deterministic independent of the road length, width and resolution. A vehicle course line can easily be generated from the crg. Also the 3d surface definition is contained, so no additional obj file is needed.

The standard of chrono is to use unordered meshs that make the calculation time grow with mesh size and complexity. Some tasks need to use adaptive mesh refinement, in this cases OpenCRG is not the right tool to use. Actually crg files cannot be used for tiled terrain surfaces.

If you want to generate crg files, please use Matlab. It is the safest way to do because it allows pretty good error and consistency checking and generates binary format.

