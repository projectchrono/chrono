---
layout: default
title: Chrono PyEngine
permalink: /documentation/chrono_pyengine/
---
<div class="text-center">
<img src="/images/Carousel_chronopyengine.jpg" alt="Chrono PyEngine">
</div>

* This will become a table of contents (this text will be scraped).
{:toc}

##Introduction 

This is an introduction on how to use Chrono::PyEngine, that is Chrono::Engine for Python.

The Chrono::PyEngine Python module is an alternative way of creating applications based on Chrono::Engine that does not require any C++ programming. In fact, once you installed the Chrono::Engine Python module in your Python environment, you will be able to use the easy Python scripting language to call a big part of the Chrono::Engine API functions, classes, methods, etc.

Advantages of Python programming vs. C++ programming:

* Python is simple,
* Python can be interpreted on the fly,
* there are lot of third party modules for Python, for instance Matplotlib for plotting, Numpy for algebra, etc.,
* a minimal installation is required. 

Disadvantages of Python programming vs. C++ programming:

* Python is slower than C++,
* the Chrono::Engine Python module does not cover all the features of the C++ API. 

## Installation

There are two options:

* For users that do not want to install the entire Chrono::Engine API there is a precompiled installer. Do this:
	* install [Python](http://www.python.org/) (only Python version 3.2 or greater, for 32 bit, is supported)
	* install the [Chrono::PyEngine](/download/#chronopyengine) module for Python, using the installer. 

* For advanced users that use the entire Chrono::Engine C++ API, they should
	* install Chrono::Engine API with C++ source code and build it,
	* install Python (only Python version 3.2 or greater is supported)
	* build the Chrono::PyEngine module, following [these instructions](/documentation/chrono_pyengine/compile_guide)

<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span> We suggest you to use a specialized IDE editor that nicely handles the Python language (syntax highlighting, auto completion of text, etc.). The default IDE installed with most Python distribution is IDLE: it is nice but we suggest a suggest to install a more powerful editor: [PyScripter](https://github.com/pyscripter/pyscripter), that is free. 

<span class="label label-info"><span class="glyphicon glyphicon-info-sign"></span></span> We suggest to install also the following third party packages for expanding the capabilities of Python in mathematical ad plotting areas: 

* [Numpy](http://numpy.scipy.org/)
* [Matplotlib](http://matplotlib.sourceforge.net/)

NOTE. Precompiled binaries of Numpy and Matplotlib for Python 3.2 can be downloaded from the unofficial [repository](http://www.lfd.uci.edu/~gohlke/pythonlibs/). A faster option is to install the entire [SciPy stack](http://www.lfd.uci.edu/~gohlke/pythonlibs/#scipy-stack) that includes both. Otherwise there is a custom Python distribution called [Enthought](http://enthought.com/products/epd.php) that already includes the two packages. 

## How to write programs
You can read about Chrono::PyEngine usage, its syntax, its differences respect to the C++ API, etc., at the [Documentation](/documentation/chrono_pyengine/doc/) page. 

## Demos and Examples

You can find examples of use in these [tutorials](/tutorials/#chronopyengine). 