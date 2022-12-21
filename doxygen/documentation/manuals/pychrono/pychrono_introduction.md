PyChrono {#pychrono_introduction}
==========================

![](http://projectchrono.org/assets/manual/logo_pychrono_h90.png)

<br>
<br>

* @subpage pychrono_installation
* @subpage pychrono_reference
* [Tutorials](@ref tutorial_table_of_content_pychrono)
* [gym-chrono](https://github.com/projectchrono/gym-chrono): OpenAI Gym robotic environments based on PyChrono

This is an introduction on how to use PyChrono, that is Chrono::Engine for Python.

PyChrono is a Python module, an alternative way of creating 
applications based on Chrono::Engine that does not require any C++ programming. 
In fact, once you installed PyChrono in your Python environment, 
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
import pychrono as chrono

my_systemA = chrono.ChSystem()
my_vect1   = chrono.ChVectorD()
...
~~~~~~~~~~~~~~~


First steps with Python
-----------------------

After the [installation](@ref pychrono_installation), you are ready to use PyChrono from
Python. To begin:

-   start your editor, for example Spyder.
-   create a new blank Python script file, for example 'test.py'

Now you can type Python programs in this new Python file, execute it,
save it on disk, etc.

Let's see a first program.

-   First of all, you should use the **import** keyword to specify which
    Python modules must be load and used in your program. Most of the
    core functionalities of Chrono::Engine are in a module called
    **pychrono**, hence write:

~~~~~~~~~~~~~~~{.py}
import pychrono as chrono
~~~~~~~~~~~~~~~

Note that the *as chrono* is optional: but if you avoid it you must call
all Chrono::Engine functions using the syntax
pychrono.ChClassFoo..., whereas if you use *as chrono*
you simply rename the namespace just like the C++ equivalent:
chrono.ChClassFoo...

-   Let's create a 3D vector object:

~~~~~~~~~~~~~~~{.py}
my_vect1 = chrono.ChVectorD()
~~~~~~~~~~~~~~~

(Note that in this way all PyChrono classes are prefixed by the *chrono*
word).

-   Modify the properties of that vector object; this is done using the
    **.** dot operator:

~~~~~~~~~~~~~~~{.py}
my_vect1.x=5
my_vect1.y=2
my_vect1.z=3
~~~~~~~~~~~~~~~

-   Some classes have build parameters, for example anothe vector can be
    built by passing the 3 coordinates for quick initialization:

~~~~~~~~~~~~~{.py}
my_vect2 = chrono.ChVectorD(3,4,5)
~~~~~~~~~~~~~

-   Most operator-overloading features that are available in C++ for the
    Chrono::Engine vectors and matrices are also available in Python,
    for example:

~~~~~~~~~~~~~~~{.py}
my_vect4 = my_vect1*10 + my_vect2
~~~~~~~~~~~~~~~

-   Member functions of an object can be called simply using the **.**
    dot operator, just like in C++:

~~~~~~~~~~~~~~~{.py}
my_len = my_vect4.Length()
print ('vector length =', my_len)
~~~~~~~~~~~~~~~

-   You can use most of the classes that you would use in C++, for
    example let's play with quaternions and vectors:

~~~~~~~~~~~~~{.py}
my_quat = chrono.ChQuaternionD(1,2,3,4)
my_qconjugate = ~my_quat
print ('quat. conjugate  =', my_qconjugate)
print ('quat. dot product=', my_qconjugate ^ my_quat)
print ('quat. product=',     my_qconjugate % my_quat)
my_vec = chrono.ChVectorD(1,2,3)
my_vec_rot = my_quat.Rotate(my_vec)
~~~~~~~~~~~~~
PyChrono linear algebra (ChMatrixDynamicD and ChVectorDynamicD) are interfaced with Python lists, this allows to use any third-party package to perform linear algebra operation. In the following example we use NumPy:
~~~~~~~~~~~~~{.py}
mlist = [[1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16]]
ma = chrono.ChMatrixDynamicD() 
ma.SetMatr(mlist)   # Create a Matrix from a list. Size is adjusted automatically.
npmat = np.asarray(ma.GetMatr()) # Create a 2D npy array from the list extracted from ChMatrixDynamic
w, v = LA.eig(npmat)  # get eigenvalues and eigenvectors using numpy
mb = chrono.ChMatrixDynamicD(4,4)
prod = v * npmat  
mb.SetMatr(v.tolist())    
~~~~~~~~~~~~~

-   If you want to know the list of methods and/or properties that are
    available in a class, you can simply use the code completion feature
    of IDEs like Spyder or VisualStudio Code: for example once you type *chrono.* you will see a
    pop-up window with a list of available classes, constants, etc.


<div class="ce-info">
Most classes behave like their C++ counterparts, so you are 
invited to look at the [C++ API documentation](http://api.chrono.projectchrono.org) to understand their features. 
Currently there is no Sphinx automated generation of Python API docs, so you should look at the C++ API docs.
</div>


Further reading
---------------

- Go to the [tutorials](@ref tutorial_table_of_content_pychrono) for examples.

- You can find more information on how PyChrono differs from the C++ Chrono in the [reference](@ref pychrono_reference) page.
