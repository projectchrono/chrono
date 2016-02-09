---
layout: default
title: Download
permalink: /download/
---

Chrono::Engine precompiled SDK
------------------------------

This is the latest SDK, precompiled for Windows. It comes as an
executable installer that you can install in order to develop C++
programs which use CHRONO::ENGINE. If you need the full access to the
source code, you should better use the [GIT
repository](GIT_repository "wikilink").

Included in the precompiled SDK there is almost everything that you can
find in the [GIT repository](GIT_repository "wikilink") such as
documentation and headers, but except the .cpp source files. Also you
get the precompiled libraries of the engine and some precompiled demo
executables. Moreover, the installer also installs a 'Chrono::Engine
Wizard' in your Microsoft Visual C++ IDE.

<div class="well">
<h4> <span class="glyphicon glyphicon-download-alt"></span> Download precompiled Chrono::Engine</h4>
<a href="http://www.chronoengine.info/download/ChronoEngine_v1.7.0.exe">Download Chrono::Engine v 1.7.0 - precompiled libraries for VC++ 32/64 bit (Windows)</a>

<h5> older versions:</h5>
<a href="http://www.chronoengine.info/download/ChronoEngine_v1.6.0.exe">Download Chrono::Engine v 1.6.0 - precompiled libraries for VC++ 32/64 bit (Windows)</a><br />
<a href="http://www.chronoengine.info/download/ChronoEngine_v1.3.0.exe">Download Chrono::Engine v 1.3.0 - precompiled libraries for VC++ 32 bit (Windows)</a>

</div>

If you want to install *only* the 'Chrono::Engine Wizard', in your
Microsoft Visual C++ IDE, without installing the precompiled SDK above
(for instance, because you already installed the source code using the
[SVN](SVN_repository "wikilink")), you can download it separately here:
<div class="well">
<h4> Download Visual Studio Wizard</h4>
<a href="http://www.chronoengine.info/download/ChronoEngine_wizard_v1.7.0.exe">Download and install Chrono::Engine wizard in Microsoft Visual Studio 9.0/10.0</a>
</div>

Chrono::PyEngine
----------------

This is the ready-to-use Chrono::PyEngine for Python. It is a module
that can be installed in your Python environment so that you can access
the functions of Chrono::Engine via the [Python
language](http://www.python.org). This can be an easier way to use
Chrono::Engine because Python is more friendly than C++ (but remember
that not all C++ functions are mapped to Python, moreover Python is
slower than C++).

The installer automatically detects your Python installation and adds
the Chrono::Engine module.

Note! currently only Python versions 3.2, 3.3 are supported. Also, only
the 32 bit version of Python is currently supported with this installer.


<div class="well">
<h4> <span class="glyphicon glyphicon-download-alt"></span> Download PyChrono::Engine for Python</h4>

<a href="http://www.chronoengine.info/download/PyChronoEngine_v1.8.0.exe"> Download Chrono::PyEngine v.1.8.0 module for Python v.3.3 64bit</a><br />
<a href="http://www.chronoengine.info/download/PyChronoEngine_v1.7.1.exe"> Download Chrono::PyEngine v.1.7.1 module for Python v.3.3 32bit</a><br />
<a href="http://www.chronoengine.info/download/PyChronoEngine_v1.7.0.exe"> Download Chrono::PyEngine v.1.7.0 module for Python v.3.2 32bit</a><br />

<h5> older versions:</h5>

<a href="http://www.chronoengine.info/download/ChronoEngine_for_Python_v1.61.exe"> Download Chrono::PyEngine v.1.6.1 module for Python v.3.2 32bit</a><br />
<a href="http://www.chronoengine.info/download/ChronoEngine_for_Python_v1.60.exe"> Download Chrono::PyEngine v.1.6.0 module for Python v.3.2 32bit</a><br />
<a href="http://www.chronoengine.info/download/ChronoEngine_for_Python_v1.50.exe"> Download Chrono::PyEngine v.1.5.0 module for Python v.3.2 32bit</a>

</div>

<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span> If you plan to use ''only'' Python, this is the only installer that you need to download.

<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span> If you need to plot graphs in Python, we suggest you to consider also the installation of both <a href="http://matplotlib.org/downloads.html"> Matplotlib</a>  and <a href="http://www.numpy.org/"> Numpy </a> Python packages.

Chrono::SolidWorks
------------------

This optional tool can be installed as an add-in for the
[SolidWorks](http://www.SolidWorks.com) 3D CAD, that is widely used by
many engineering companies. After you install this add-in, you will find
a new exporter tool in the right panel of SolidWorks: this can be used
to generate .py files with PyChrono::Engine scene descriptions
containing masses, constraints, etc.

The installer automatically detects your SolidWorks installation (v.2011
and v.2012 64bit supported and tested, for the moment) and adds the
Chrono::Engine add-in.

<div class="well">
<h4> <span class="glyphicon glyphicon-download-alt"></span> Download Chrono::SolidWorks add-in for SolidWorks</h4>
<a href="http://www.chronoengine.info/download/ChronoEngine_SolidWorks_v2.04.exe"> Download Chrono::SolidWorks v.2.0.4 add-in for SolidWorks 2013 and higher (beta)</a><br />

<h5> older versions:</h5>

<a href="http://www.chronoengine.info/download/ChronoEngine_SolidWorks_v2.01.exe"> Download Chrono::SolidWorks v.2.0.1 add-in for SolidWorks 2012</a><br />
<a href="http://www.chronoengine.info/download/ChronoEngine_SolidWorks_v1.70.exe"> Download Chrono::SolidWorks v.1.7.0 add-in for SolidWorks 2011</a><br />
<a href="http://www.chronoengine.info/download/ChronoEngine_SolidWorks_v1.62.exe"> Download Chrono::SolidWorks v.1.6.2 add-in for SolidWorks 2011</a>

</div>

Required tools
--------------

### Irrlicht 3D

The [Irrlicht](http://irrlicht.sourceforge.net/downloads.html) library
is **required** in order to compile the Chrono::Engine demos that use a
3D visualization. It is a cross-platform 3D visualization library, based
on OpenGL and DirectX.

<div class="well">
<h4> <span class="glyphicon glyphicon-download-alt"></span> Download Irrlicht</h4>
<a href="http://downloads.sourceforge.net/irrlicht/irrlicht-1.7.3.zip">Download Irrlicht v.1.7.3 </a>
</div>

<span class="label label-danger"><span class="glyphicon glyphicon-exclamation-sign"></span></span> The new release v.1.8 of Irrlicht is not supported because the support of soft shadows via Xeffects is broken. Until we wait for a fix to this issue, we urge you to download and install only the previous Irrlicht release v.1.7.3

### CMake

The CMake tool is **required** in order to perform the building of the
Chrono::Engine libraries and demos. It is cross-platform and available
for many operating systems (choose the one that you need, ex. Windows,
Linux, etc.):

-   Download [CMake](http://www.cmake.org/cmake/resources/software.html)
    from the www.cmake.org web site.

If you want to use the Chrono::Engine pre-built libraries and build your
projects using the easy VisualStudio wizard, CMake is *not needed*.

Optional tools
--------------

### Matlab

The *optional* **unit\_MATLAB** in Chrono::Engine provides the support
for interoperation with Matlab(TM). If you want to build that unit and
to test its demos, you must have the Matlab API installed in your
computer: that is distributed in the main Matlab software, so if you
already have Matlab installed you are fine. If not, just do not use the
unit\_MATLAB in the Chrono::Engine project. If you want to buy the
Matlab software, go to
[<http://www.mathworks.com>](http://www.mathworks.com).

### MPI

The *optional* **unit\_MPI** in Chrono::Engine provides functionality
for performing distributed parallel computing on large clusters of
computers, using the Message Passing Interface (MPI) standard. If you
want to build that unit and to test its demos, you must have the MPICH 2
installed in your computer. It can be downloaded from [this
site](http://www.mcs.anl.gov/research/projects/mpich2/downloads/index.php?s=downloads).

### CUDA

The *optional* **unit\_GPU** in Chrono::Engine provides functionality
for performing parallel computation on NVIDIA GPU boards. If you want to
build that unit and to test its demos, you must have the CUDA SDK and
toolkit installed in your computer. They can be downloaded from [this
site](http://developer.nvidia.com/object/cuda_downloads.html).

### POVray

The *optional* **unit\_POSTPROCESSING** in Chrono::Engine provides
functionality for exporting simulation data in formats that can be
rendered by external software. No libraries are needed to build or use
unit\_POSTPROCESSING; however you may need raytracing software for
processing the saved data. Currently, the POVray 3D raytracer is
supported and tested. Download it from [this
site](http://www.povray.org).

<div class="well">
<p>In case you do not have a C++ compiler already installed on your computer, you can download one of these free tools:</p>

<h4> Windows </h4>
<a href="http://www.microsoft.com/express/Windows"> Microsoft Visual C++ Express (suggested)</a><br />
<a href="http://www.mingw.org/wiki/InstallationHOWTOforMinGW"> MingW GNU C++</a>
<p></p>
<h4> Linux </h4>
On most distributions, the GNU gcc compiler should be already installed by default. If not, install with:  <tt>sudo apt-get install gcc build-essential</tt> (assuming you have Ubuntu) or download from <a href="http://gcc.gnu.org"> http://gcc.gnu.org</a>.
</div>
