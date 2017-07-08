Install Chrono::PyEngine {#chrono_pyengine_installation}
==========================

There are two options for installing Chrono::PyEngine on your computer, A) or B). 
The first is for users that are not interested in the C++ API.


## A) Install precompiled Python modules

For users that do not want to install the entire Chrono::Engine API 
there is a precompiled installer. 
Do this:

1. download and install [Python](http://www.python.org) (only Python version 3.2 or greater is supported)

2. download and install the [Chrono::PyEngine module for Python](http://www.projectchrono.org/download), 
   using the installer in our download section.

<div class="ce-info">
You do not need to install the entire C++ API/SDK of Chrono, in this case. 
</div>

<div class="ce-danger">
Note that there are separate installers for the 32 bit or 64 bit distributions of Python. 
Do not mix 32bit with 64bit. We suggest you to use the 64 bit Python and, consequently, the 64 bit Chrono::PyEngine
</div>

<div class="ce-warning">
Note that the releases of the installers in our download page might be lagging behind the most recent 
Chrono API: if you want to exploit the most recent features, you should use the following second method.
</div>



## B) Build Python modules from the C++ API

Advanced users that use the entire Chrono::Engine C++ API can build Chrono::PyEngine from scratch. 
This is the preferred way to have the most updated Chrono::PyEngine, but it is more complicated.
Do this:

1. install [the Chrono API](@ref tutorial_install_chrono) with C++ source code and build it,
2. install [Python](http://www.python.org) (only Python version 3.2 or greater is supported)
3. build the Chrono::PyEngine module, following [these instructions](@ref module_python_installation)


## Tips

<div class="ce-info">
We suggest you to use a specialized IDE editor that nicely handles 
the Python language (syntax highlighting, auto completion of text, etc.). 
The default IDE installed with most Python distribution is IDLE: 
it is nice but we suggest a suggest to install a more powerful editor: 
[PyScripter](https://sourceforge.net/projects/pyscripter/), 
that is free.
</div>

<div class="ce-info">
We suggest to install also the following third party packages for expanding 
the capabilities of Python in mathematical ad plotting areas:
<ul>
  <li>[Numpy](http://numpy.scipy.org/)</li>
  <li>[Matplotlib](http://matplotlib.sourceforge.net/)</li>
</ul>
NOTE. Precompiled binaries of Numpy and Matplotlib for Python 3.2 can be 
downloaded  from the [unofficial repository](http://www.lfd.uci.edu/~gohlke/pythonlibs/). 
A faster option is to install the [entire SciPy stack](http://www.lfd.uci.edu/~gohlke/pythonlibs/#scipy-stack)
that includes both.
Otherwise there is a custom Python distribution called [Enthough](http://enthought.com/products/epd.php) that already includes the two packages.
</div>
