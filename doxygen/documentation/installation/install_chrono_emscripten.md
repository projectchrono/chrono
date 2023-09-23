Build Chrono for WASM {#tutorial_install_chrono_emscripten}
==========================

The following instructions detail the process of building Chrono for WebAssembly using [Emscripten](https://emscripten.org/) on a Linux host. 

These instructions may also work for a Windows or macOS host with minimal modification. 

See [Install Chrono](@ref tutorial_install_chrono) for native build instructions. 



#### 1) Install Emscripten 

Download and install Emscripten using [one of the documented methods](https://emscripten.org/docs/getting_started/downloads.html). 


#### 2) Download and install the Eigen library

Chrono uses [Eigen3](http://eigen.tuxfamily.org/) for all of its internal dense linear algebra needs. Chrono requires Eigen version 3.3.0 or newer, but we strongly encourage using the **latest stable release family, Eigen 3.4.0 (or newer)**. Eigen is a header-only library, so a version installed by your system package manager or downloaded from the upstream source will work with emscripten. 

#### 3) Download and install GLM (OPTIONAL)

If you intend to make use of Chrono's built-in WebGL visualization capability, the [OpenGL Mathematics](https://github.com/g-truc/glm) library will be needed during build. As it is a header-only library, it may be available through your system package manager, or the source code can be [downloaded from GitHub](https://github.com/g-truc/glm/releases) and used as-is. 

#### 4) Download and Install CMake

[CMake](https://cmake.org/) is required to configure the build before compiling Chrono. It is also widely used among the Linux community to build many other software units. If it isn't already installed, use your system's package manager to install it. On most systems, the package is simply called `cmake`.

<div class="ce-warning">
Debian-based distributions may need to install the package `cmake-curses-gui` along with `cmake` to use terminal based GUI mentioned in the steps below. 

> Note: The Qt GUI for CMake (`cmake-gui` on some systems) will not work nicely with emscripten. Use the command line or the curses terminal GUI instead.  
</div>

#### 5) Download and install Ninja

[Ninja](https://ninja-build.org/) is recommended to facilitate a faster and more portable build process. It is typically available from your system package manager as `ninja-build` or just `ninja`. 

#### 6) Download the Project Chrono source code using Git 

Download the Chrono SDK by performing a **clone** of the Git repository on your machine. `git clone -b master git@github.com:projectchrono/chrono.git` will create a copy of the Github repository in the current directory.

<div class="ce-info">
Checking out the [latest release tag](https://github.com/projectchrono/chrono/tags) is **highly recommended**. Releases denote the most stable and well-tested versions of Chrono. To check out release 8.0.0, use: `git switch --detach 8.0.0`.

If you are interested in using the latest features as they are developed and before the next official release, you can switch to the `develop` branch with `git switch develop`.
</div>

#### 6) Run CMake using the Emscripten Wrapper 

Create a new _empty_ directory inside of the Chrono SDK source directory. This will be used to build Chrono: `mkdir build`. Next, change your current directory to it: `cd build`.

> We recommend using the terminal based GUI for CMake to configure the Chrono build, however the following process may be replicated through the command line.

-  Run `emcmake ccmake -G Ninja ..` from the build directory. The wrapper `emcmake` will set up the emscripten toolchain for CMake, and then a text based GUI will appear in the terminal.

-  Enter `c` to **Configure** and continue. The interface will reload to a new screen with more options.
  
-  Specify the location of the Eigen installation.
   If this is not detected automatically, you may need to manually set the CMake variable `EIGEN3_INCLUDE_DIR`.<br>

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_1.png" class="img-responsive" width="400">

<div class="ce-warning">
If the Eigen library was installed through a package manager, its location may be detected automatically. If so, verify the path to the library is correct before continuing.
</div>

  - The following instructions are optional but highly **recommended** as they provide support for WebGL visualization when running in browser.

    - Press `c` to **Configure** and continue to the next screen.

    - Enable the recommended optional modules: at least tick `ENABLE_MODULE_OPENGL`.<br>
     Other modules might require additional settings and dependencies.

    -  Enter `c` to **Configure** and continue to the next screen.

	- Emscripten contains embedded headers for GLEW and GLFW, two dependencies of Chrono, in its sysroot folder. If you installed an emscripten environment using emsdk, a folder should be available in your home directory at `~/.emscripten_cache/sysroot` which contains the required headers.
	
<div class="ce-warning">
Due to a bug in CMake's handling of the GLEW and GLFW with emscripten, the `GLEW_LIBRARY` and `GLFW_LIBRARY` variables may need to be set to some nonsense value in order for CMake to build the OpenGL module correctly. If you enabled the module, but no demos are generated for it, try setting the variables to a valid path such as `/dev/null`.
</div>

	- Specify the location of the GLM installation. If this is not detected automatically, you may need to manually set the CMake variable `GLM_INCLUDE_DIR`.  

<div class="ce-warning">
Similar to the Eigen directory field set earlier, the GLM include directory may be detected automatically. If it was detected automatically, verify the location of the library is correct before continuing.
</div>
 
- Enter `c` to **Configure** and continue until you reach the final screen. The `g` option to **Generate** will appear on this screen. Press `g` and allow the configuration to complete; ccmake will close on completion.

<img src="http://www.projectchrono.org/assets/Images/install_ccmake_3.png" class="img-responsive" width="400">

#### 7) Compile the project

Run command `ninja` while in the same build directory as the newly created Makefile. Be prepared to wait 15 - 25 minutes for the build to complete. On systems with an excess of CPU cores, up to _N_ build steps can be run simultaneously by using `ninja -j N` to speed up the process. This is the final step of the Chrono build process. Congratulations!

<div class="ce-info">
`ninja install` would normally copy the Chrono libraries, data files, and demo executables to the install directory specified during CMake configuration, however this is not practical when targeting WebAssembly as the generated files will usually be embedded into a webpage instead. 
</div>

#### 8) Test the demos

Navigate to the directory that you used to build Chrono earlier. Change the current directory to the subdirectory, `bin`. Demo example files are stored here, they are great resource to **demo**nstrate the capacities of Project Chrono.

For each demo, there are two files generated, a binary file ending in `.wasm` and a JavaScript wrapper ending in `.js`.

- For command-line demos, A JavaScript engine such as [Node.js](https://nodejs.org/en/) can be used to execute the demo right in your terminal. 

- For more complicated demos, including those which employ visualization, the JavaScript file must be loaded into a web page. This process is somewhat more complicated, but it is detailed in [Emscripten's documentation](https://emscripten.org/docs/compiling/Deploying-Pages.html).

