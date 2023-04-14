Install DEM-Engine {#deme_installation}
=================================

### Installation from source

On a Linux machine, install CUDA if you do not already have it. Useful installation instructions may be found [here](https://developer.nvidia.com/cuda-downloads). 

Some additional troubleshooting tips for getting CUDA ready:

- On WSL this code may be buildable (and [this](https://docs.nvidia.com/cuda/wsl-user-guide/index.html) is the guide for installing CUDA on WSL), but may not run. This is due to the [many limitations on unified memory and pinned memory support](https://docs.nvidia.com/cuda/wsl-user-guide/index.html#known-limitations-for-linux-cuda-applications) on WSL. A native Linux machine or cluster is recommended.

Once CUDA is ready, clone this project and then:

```
git submodule init
git submodule update
```

This will pull the submodule NVIDIA/jitify so that we can do runtime compilation. 

Then, one typical choice is to make a build directory in it. Then in the build directory, use `cmake` to configure the compilation. An example:

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
```

You may want to use [this information](https://askubuntu.com/questions/1203635/installing-latest-cmake-on-ubuntu-18-04-3-lts-run-via-wsl-openssl-error) if you need to update cmake to the newest. 

We suggest that you install a `cmake` GUI such as `ccmake`, and `ninja_build` generator, to better help you configure the project. In this case, the example above can be done like this alternatively:

```
mkdir build
cd build
ccmake -G Ninja ..
```

You generally do not have to change the build options in the GUI, but preferably you can change `CMAKE_BUILD_TYPE` to `Release`, and if you need to install this package as a library you can specify a `CMAKE_INSTALL_PREFIX`. 

Some additional troubleshooting tips for generating the project:

- If some dependencies such as CUB are not found, then you probably need to manually set `$PATH` and `$LD_LIBRARY_PATH`. An example is given below for a specific version of CUDA, note it may be different on your machine or cluster. You should also inspect if `nvidia-smi` and `nvcc --version` give correct returns.
```
export CPATH=/usr/local/cuda-12.0/targets/x86_64-linux/include${CPATH:+:${CPATH}}
export PATH=/usr/local/cuda-12.0/bin${PATH:+:${PATH}}
export PATH=/usr/local/cuda-12.0/lib64/cmake${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export CUDA_HOME=/usr/local/cuda-12.0
```

Finally, build the project.

```
ninja
```

Some additional troubleshooting tips for building the project:

- If you see some grammatical errors during compilation, such as `filesystem` not being a member of `std` or arguments not expanded with `...`, then manually setting the flag `TargetCXXStandard` to `STD_CXX17` might help.

### Install as C++ library

Set the `CMAKE_INSTALL_PREFIX` flag in `cmake` GUI to your desired installation path and then 

```
ninja install
```

We provide examples of linking against both [Chrono](https://github.com/projectchrono/chrono) and _DEME_ for co-simulations in [chrono-projects](https://github.com/projectchrono/chrono-projects/tree/feature/DEME).

Assuming you know how to build `chrono-projects` linking against a Chrono installation, then the extra things that you should do to link against _DEME_ are

- Set `ENABLE_DEME_TESTS` to `ON`;
- Set `ChPF_DIR` when prompted. It should be in `<your_install_dir>/lib64/cmake/ChPF`;
- Set `DEME_DIR` when prompted. It should be in `<your_install_dir>/lib64/cmake/DEME`.

Then build the project and you should be able to run the demo scripts that demonstrate the co-simulation between _DEME_ and Chrono.

