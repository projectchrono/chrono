Install Chrono::PyEngine {#chrono_pyengine_installation}
==========================

There are two options for installing Chrono::PyEngine on your computer, A) or B). 
The first is for users that are not interested in the C++ API.


## A) Install precompiled Python modules

For users that do not want to install the entire Chrono::Engine API 
there is a precompiled installer. 
Do this:

1. download and install [Python](http://www.python.org) (only Python version 3.2 or greater is supported). 
   Or, if you have hard disk space, better install a full stack like [Anaconda](https://www.anaconda.com/download/)

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
2. install [Python](http://www.python.org) (only Python version 3.2 or greater is supported). 
   Or, if you have hard disk space, better install a full stack like [Anaconda](https://www.anaconda.com/download/)
3. build the Chrono::PyEngine module, following [these instructions](@ref module_python_installation)


## Tips

<div class="ce-info">
Hint: instead than installing the basic Python interpreter, you can install 
a pre-configured Python distribution that contains Python plus many useful packages, editors
and tools. Our suggestion is to downloadand and install [Anaconda](https://www.anaconda.com/download/),
a very powerful stack that aims at scientific computing.
Another popular Python distribution is [Enthough](http://enthought.com/products/epd.php) that already includes the two packages.
</div>

<div class="ce-info">
We suggest you to use a specialized IDE editor that nicely handles 
the Python language (syntax highlighting, auto completion of text, etc.). 
The default IDE installed with most Python distribution is IDLE: 
it is suficient only for simple stuff, so we suggest to use a more powerful editor. 
Our best pick is ***Spyder***, the IDE that comes together with 
the [Anaconda](https://www.anaconda.com/download/) Python distribution. Other free options could be
[PyScripter](https://sourceforge.net/projects/pyscripter/), 
or [Visual Studio Code](https://code.visualstudio.com/).
</div>

<div class="ce-warning">
Either you install Chrono::PyEngine via method A) or B), do not forget to set the PYTHONPATH environment variable.
</div>

