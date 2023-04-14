# Enabling OpenCRG support

Chrono::Vehicle has the ability of using road geometry specified through a `crg` file. The data format is very compact. Roads of arbitrary length can be handled. If you want to use it you have to build the OpenCRG support library before building Chrono. 

Get the distribution here: http://www.opencrg.org. The most current version is 1.1.2. 

Mac and Linux users can build the OpenCRG library using the provided makefiles. 

For Windows users, we provide the script `buildOpenCRG.bat`. To use it, you need the Visual Studio `cl` compiler. First, edit the script to specify the location of the OpenCRG distribution and the installation directory. Note that you must run this script from a VS developer command prompt or PowerShell. 

Alternatively, you can use the `CMakeLists.txt` file in this directory, which should also work for Windows. The `CMakeLists.txt` has been modified to build the Release and the Debug variants with different names, so that they can be installed to the same directories. You have to use different runs of cmake for Release and for Debug. Instead of 'make' 'ninja' should be used, it works well on Linux, Mac and Windows.

1. Copy OpenCRG.1.1.2.zip to a location of your coice
2. Unpack the zip archive: 
   - [Linux, Mac] `unzip -a OpenCRG.1.1.2.zip`
   - [Windows] use explorer or a 3rd party tool such as 7-zip 
3. Copy the `CMakeLists.txt` provided here into `/your/path/OpenCRG.1.1.2/c-api`
4. Configure: 
   `cmake  -G Ninja -B Release -DCMAKE_BUILD_TYPE=Release /your/path/OpenCRG.1.1.2/c-api` 
   `cmake  -G Ninja -B Debug -DCMAKE_BUILD_TYPE=Debug /your/path/OpenCRG.1.1.2/c-api` 
5. Build it:
   - [Linux, Mac, Windows] `cmake --build Release`
   - [Linux, Mac, Windows] `cmake --build Debug`
6. The generated files are:
   - header: `CrgBaseLib.h`
   - Release library: `libOpenCRG.a` [Linux, Mac], `OpenCRG.lib` [Windows]
   - Debug library: `libOpenCRGd.a` [Linux, Mac], `OpenCRGd.lib` [Windows]
7. Install the files
   - [Linux, Mac, Windows] `ninja --install Release --prefix=/where/you/want/it_to_be`
   - [Linux, Mac, Windows] `ninja --install Debug --prefix=/where/you/want/it_to_be`
6. When you configure Chrono let the configure script point to the needed OpenCRG files

On all platforms, the provided CmakeLists.txt builds OpenCRG as a static library.

# Why should anybody use OpenCRG files?

Chrono::Vehicle has to calculate the wheel/road contact points very often. Since the OpenCRG file act like an equidistant ordered mesh the calculation time is deterministic independent of the road length, width and resolution. A vehicle course line can easily be generated from the crg. Also the 3d surface definition is contained, so no additional obj file is needed.

The standard of chrono is to use unordered meshs that make the calculation time grow with mesh size and complexity. Some tasks need to use adaptive mesh refinement, in this cases OpenCRG is not the right tool to use. Actually crg files cannot be used for tiled terrain surfaces.

If you want to generate crg files, please use Matlab. It is the safest way to do because it allows pretty good error and consistency checking and generates binary format.

