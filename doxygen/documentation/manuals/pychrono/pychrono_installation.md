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


2. Optionally add the `conda-forge` and `intel` channels to the list of channels:
```
   conda config --add channels https://conda.anaconda.org/conda-forge
   conda config --add channels https://conda.anaconda.org/intel
```   

3. Decide which version of the Chrono code (latest release or latest code) you want and for which Python version.  Consult the list of available modules on the [PyChrono Anaconda Repository](https://anaconda.org/projectchrono/pychrono/files) and download the appropriate archive (tar.bz2). 

   PyChrono packages built from a Chrono release version have label 'release'; PyChrono packages built from the latest Chrono development code have label 'main'.

4. We striongly recommend to use a dedicated conda environment and install the desired Python distribution, necessary dependencies, and PyChrono package under that environment.

   For example, to create a `chrono` environment with Python 3.9, use:
```
   conda create -n chrono python=3.9
```   
   Then activate that environment:
```
   conda activate chrono
```
   so that all subsequent conda commands occur within that environment.

5. Install the necessary dependencies.<br>
   **Attention**: Install the following packages using the versions specified below, in the order given, and *before* installing the PyChrono conda package itself!<br><br>

   - Intel MKL package (required for PyChrono demos using the Pardiso direct sparse linear solver, for Numpy, and for PythonOCC):
   ```
   conda install -c conda-forge mkl=2020
   ```
   - Numpy package (required for the Chrono::Sensor module):
   ```
   conda install -c conda-forge numpy=1.24.0
   ```
   - Irrlicht, for run-time visualization:
   ```
   conda install -c conda-forge irrlicht=1.8.5
   ```
   - Pythonocc-core, for Chrono::Cascade support:
   ``` 
   conda install -c conda-forge pythonocc-core=7.4.1
   ```
   - For Chrono::Sensor support:
     - Requires NVIDIA graphics driver 515.xx +
     - Install CUDA 11.7:
   ```
     conda install -c nvidia/label/cuda-11.7.0 cuda-toolkit
   ```
     - install GLFW: 
   ```
     conda install -c conda-forge glfw
   ```

6. Install the PyChrono conda package downloaded in step 3 above (for release or development Chrono code, a given operating system, and built for a given Python version):
```
   conda install <pychrono_package>.tar.bz2
```    

   Note that installing the default conda package (latest package with label 'main' for your current operating system and Python version):
```
   conda install -c projectchrono pychrono
```
   may not always work as expected. We strongly recommend downloading the appropriate PyChrono installation archive and install it from a local file as described above.

<div class="ce-warning">
In general, no changes to PYTHONPATH are required when installing the PyChrono conda package.  The exception is MacOS Apple silicon for which PYTHONPATH must be changed (or created if not present) to include the path to the PyChrono *.so libraries. For example:
```
export PYTHONPATH=$HOME/opt/anaconda3/envs/chrono/share/chrono/python
```
</div>

<div class="ce-warning">
If you are interested in using the Chrono::Cascade module through PyChrono, note that the conda [pythonocc-core](https://anaconda.org/conda-forge/pythonocc-core/files) package version 7.4.1 is **not** available for Python 3.10.  Use Python version 3.9 or 3.8.
</div>

<div class="ce-info">
The reason for the specific dependency versions and order of installation of dependency conda packages is related to the fact that Chrono::Cascade uses the API from OpenCascade v. 7.4 and that the pythonocc-core version 7.4.1 hardcodes a requirement for MKL version 2020. As such, the PyChrono conda packages are linked against this specific version of MKL.  If installed first, the Numpy package would install MKL as its own dependency and, by default, Numpy version 1.24.0 installs MKL version 2022.  Until we modify the PyChrono conda package to automatically install all required dependencies of the necessary version and in the proper order, it is the user's responsibility to ensure version compatibility by following the steps above as listed.
</div>   

   


## B) Build Python modules from the C++ API

Advanced users that use the entire Chrono::Engine C++ API can build PyChrono from scratch. 
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
The default IDE installed with most Python distribution is IDLE which is suficient only for simpler tasks. 
Our best pick is **Spyder**, the IDE that comes together with the [Anaconda](https://www.anaconda.com/download/) Python distribution. 
Other free options are [Visual Studio Code](https://code.visualstudio.com/) or the [Python add-on for Visual Studio](https://marketplace.visualstudio.com/items?itemName=ms-python.python).
</div>


