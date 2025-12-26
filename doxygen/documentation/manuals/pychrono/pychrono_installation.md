Install PyChrono {#pychrono_installation}
==========================

There are two options for installing PyChrono on your computer. 
The first one uses a prebuilt conda packages and is the recommended way. 
The second one is for users who need to build the full library from the C++ source.


## A) Install precompiled Python modules

We provide precompiled PyChrono modules that can be installed in Python in a single step. PyChrono modules are available both for the latest released code, as well as for the latest version of the code in the git *main* branch.  Conda packages are made available for Linux, Windows and MacOS, and for different versions of Python3.

![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/latest_release_date.svg)
![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/platforms.svg)
![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/license.svg)
![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/downloads.svg)

To install a PyChrono conda module, do the following:

1. Install the [Anaconda](https://www.anaconda.com/download/) Python distribution. <br>

2. We strongly recommend to use a dedicated conda environment and install the desired Python distribution, necessary dependencies, and PyChrono package under that environment.

   To create a `chrono` environment with Python 3.12, use:

   ```
   conda create -n chrono python=3.12
   ```   
   Then activate that environment:
   ```
   conda activate chrono
   ```
   so that all subsequent conda commands occur within that environment.

3. Decide which version of the Chrono code (latest release or latest code) you want and for which Python version.  Consult the list of available modules on the [PyChrono Anaconda Repository](https://anaconda.org/projectchrono/pychrono/files). 

   PyChrono packages built from a Chrono release version have label 'release'; PyChrono packages built from the latest Chrono development code have label 'main'.


4. Install the PyChrono conda package. If you want to install the latest developmental branch, run

   ```
   conda install projectchrono::pychrono -c conda-forge
   ```

   Otherwise, run with the specific version code you want to install, for example:

   ```
   conda install projectchrono::pychrono=9.0.1=py312hf1de3a3_6463 -c conda-forge
   ```

<div class="ce-warning">
vsg3d module and ROS module are not yet available in the conda version of PyChrono. To use them you might need to "Build Python modules from the C++ API"
</div>




## B) Build Python modules from the C++ API

Advanced users that use the entire Chrono C++ API can build PyChrono from scratch. 
This is the preferred way to have the most updated PyChrono, but it is more complicated.
Do this:

1. install [the Chrono API](@ref tutorial_install_chrono) with C++ source code and build it,
2. install [Python](http://www.python.org) (only Python version 3.2 or greater is supported). 
   Or, if you have hard disk space, better install a full stack like [Anaconda](https://www.anaconda.com/download/)
3. build the PyChrono module, following [these instructions](@ref module_python_installation)

<div class="ce-warning">
When building PyChrono from the C++ source, the PYTHONPATH environment variable must be edited to include the path to the bin/ directory in the Chrono build tree.
For example:
```
export PYTHONPATH=$HOME/chrono_build/bin
```
</div>

## Tips

<div class="ce-info">
We suggest you to use a specialized IDE editor that nicely handles the Python language (syntax highlighting, intellisense, etc.). 
The default IDE installed with most Python distribution is IDLE which is sufficient only for simpler tasks. 
Our best pick is **Spyder**, the IDE that comes together with the [Anaconda](https://www.anaconda.com/download/) Python distribution. 
Other free options are [Visual Studio Code](https://code.visualstudio.com/) or the [Python add-on for Visual Studio](https://marketplace.visualstudio.com/items?itemName=ms-python.python).
</div>


