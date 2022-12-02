Install PyChrono {#pychrono_installation}
==========================

There are two options for installing PyChrono on your computer, A) or B). 
The first is the reccommended way. The second is for users that need to build the full library starting from the C++ source.


## A) Install precompiled Python modules

We provide precompiled PyChrono modules that can be installed in Python in a single step. PyChrono modules are available both for the latest released code, as well as for the latest version of the code in the git *main* branch.  Conda packages are made available for Linux, Windows and MacOS, and for different versions of Python3.

To install a PyChrono conda module, do the following:

1. Be sure that you have installed the [Anaconda](https://www.anaconda.com/download/) Python distribution. <br>
   (If you already have [Python](http://www.python.org) installed, you can still install Anaconda or MiniConda).

2. Add the `conda-forge` and `intel` channels to the top of the list of channels:
```
   conda config --add channels https://conda.anaconda.org/conda-forge
   conda config --add channels https://conda.anaconda.org/intel
```   

3. Decide which version of the Chrono code (latest release or latest code) you want and for which Python version.  See the full list of available modules on the [PyChrono Anaconda Repository](https://anaconda.org/projectchrono/pychrono/files)

4. We recommend you use a dedicated conda environment (e.g., `chrono`) and install the desired Python distribution and PyChrono module there.

   For example, to create a `chrono` environment with Python 3.9, use:
```
   conda create -n chrono python=3.9
```   

5. Install the necessary dependencies (**Attention**: Install the dependencies *before* installing the PyChrono conda package!)

   - Numpy package:
   ```
   conda install -c conda-forge numpy
   ```
   - for run-time visualization with Irrlicht:
   ```
   conda install -c conda-forge irrlicht
   ```
   - for Cascade support (**Attention**: Chrono requires version 7.4 of OpenCascade):
   ``` 
   conda install -c conda-forge pythonocc-core=7.4.1
   ```
   - for sensor support:
     - requires NVIDIA graphics driver 515.xx +
     - install cuda 11.7 from conda
     ```
     conda install -c nvidia/label/cuda-11.7.0 cuda-toolkit
     ```
     - install glfw 
     ```
     conda install -c conda-forge glfw
     ```


6. If you want the PyChrono package for the latest Chrono *code*, simply do
```
   conda install -c projectchrono pychrono
```
   To install a specific PyChrono module (release or development code, for a given opperating system, and built with a given Python version), download the corresponding tarball from the [PyChrono Anaconda Repository](https://anaconda.org/projectchrono/pychrono/files) and then install it from the local file; for example:
```
   conda install <pychrono_package>.tar.bz2
```    

<div class="ce-warning">
If you are interested in using the Chrono::Cascade module through PyChrono, note that the conda [pythonocc-core](https://anaconda.org/conda-forge/pythonocc-core/files) package is **not** available for Python 3.10.  Use verion 3.9 or earlier.
</div>

<div class="ce-warning">
Note that by default  Conda installs the 'main' packages from the Anaconda repository, which correspond to the latest Chrono code (in the *main* branch of the Chrono GitHub repository). 
If you want to install a particular release of PyChrono, go to the [Anaconda package repository](https://anaconda.org/projectchrono/pychrono/install)
and pick the one that you need. <br>
   PyChrono packages corresponding to code in the *main* git branch have label 'main'.<br>
   PyChrono packages corresponding to a Chrono release have label 'release'.
</div>

![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/latest_release_date.svg)
![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/platforms.svg)
![Anaconda-Server Badge](https://anaconda.org/projectchrono/pychrono/badges/installer/conda.svg)

   

## B) Build Python modules from the C++ API

Advanced users that use the entire Chrono::Engine C++ API can build PyChrono from scratch. 
This is the preferred way to have the most updated PyChrono, but it is more complicated.
Do this:

1. install [the Chrono API](@ref tutorial_install_chrono) with C++ source code and build it,
2. install [Python](http://www.python.org) (only Python version 3.2 or greater is supported). 
   Or, if you have hard disk space, better install a full stack like [Anaconda](https://www.anaconda.com/download/)
3. build the PyChrono module, following [these instructions](@ref module_python_installation)


## Tips

<div class="ce-info">
We strongly encourage using [Anaconda](https://www.anaconda.com/download/) rather 
than the plain Python interpreter because Anaconda ships with many scientific packages like Numpy installed by default.
Another popular Python distribution is [Enthough](http://enthought.com/products/epd.php) that already includes the two packages.
</div>

<div class="ce-info">
We suggest you to use a specialized IDE editor that nicely handles 
the Python language (syntax highlighting, intellisense, etc.). 
The default IDE installed with most Python distribution is IDLE: 
it is suficient only for simple stuff, so we suggest to use a more powerful editor. 
Our best pick is **Spyder**, the IDE that comes together with 
the [Anaconda](https://www.anaconda.com/download/) Python distribution. Other free options could be
or [Visual Studio Code](https://code.visualstudio.com/) or the Python add-on for VisualStudio.
</div>

<div class="ce-warning">
If you build PyChrono from the C++ source, i.e. method B), you need to set the PYTHONPATH environment variable so that it points
to the directory where you built the binaries. 
This is not needed if you installed PyChrono with method A), because the Conda installer takes care of the PYTHONPATH.
</div>
