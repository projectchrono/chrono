# Building Irrlicht with Xcode 10

Xcode 10 brought a new issue related to building the Irrlicht library. This patch provides a work-around:

1. unpack irrlicht-1.8.4.zip to a directory of your choice
2. copy the patch file irrlicht-xcode10.diff into the irrlicht-1.8.4 directory
3. from the terminal, execute the following commands:
    ```
    cd /your/Path/irrlicht-1.8.4
    patch -p1 < irrlicht-xcode10.diff
    cd source/Irrlicht/MacOSX
    xcodebuild -target libIrrlicht.a -configuration Release build
    cp build/Release/libIrrlicht.a ../../../lib/MacOSX
    ```

This patch was tested on MacOS 10.13 (High Sierra).

On MacOS 10.14 (Mojave) you can also build Irrlicht with these changes, but Apple messed up the OpenGL library and you will get only an empty window without 3D graphics.
The same problem exists for GLFW 3.2.1.
