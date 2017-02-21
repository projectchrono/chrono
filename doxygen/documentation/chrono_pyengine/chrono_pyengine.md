Chrono::PyEngine {#introduction_chrono_pyengine}
==========================

![](http://projectchrono.org/assets/manual/carousel_chronopyengine.jpg)

* @subpage chrono_pyengine_installation
* @subpage chrono_pyengine_reference
* [Tutorials](@ref tutorial_table_of_content_chrono_pyengine)

This is an introduction on how to use Chrono::PyEngine, that is Chrono::Engine for Python.

The Chrono::PyEngine Python module is an alternative way of creating 
applications based on Chrono::Engine that does not require any C++ programming. 
In fact, once you installed the Chrono::Engine Python module in your Python environment, 
you will be able to use the easy Python scripting language to call a big part of the 
Chrono::Engine API functions, classes, methods, etc.


**Advantages** of Python programming vs. C++ programming:

* Python is simple,
* Python can be interpreted on the fly,
* there are lot of third party modules for Python, for instance Matplotlib for plotting, Numpy for algebra, etc.,
* a minimal installation is required. 

**Disadvantages** of Python programming vs. C++ programming:

* Python is slower than C++,
* the Chrono::Engine Python module does not cover all the features of the C++ API. 
	
	
The idea is that, once installed, you can open your Python IDE, import the ChronoEngine
Python module(s) and start creating Chrono objects as in the following:

~~~~~~~~~~~~~~~{.py}
import ChronoEngine_python_core as chrono

my_systemA = chrono.ChSystem()
my_vect1   = chrono.ChVectorD()
...
~~~~~~~~~~~~~~~

You can find more information on how to use Chrono::PyEngine in its [reference](@ref chrono_pyengine_reference) page.

Go to the [tutorials](@ref tutorial_table_of_content_chrono_pyengine) for examples.



