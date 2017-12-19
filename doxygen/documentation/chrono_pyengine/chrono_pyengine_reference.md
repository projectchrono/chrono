Chrono::PyEngine reference {#chrono_pyengine_reference}
==========================

Chrono::PyEngine is a Python wrapper for Chrono::Engine. It is a set of
Python modules that correspond to the main units of Chrono::Engine, as
shown in this scheme:

![](http://www.projectchrono.org/assets/manual/Units_python.png)


First steps with Python
-----------------------

After the [installation](@ref chrono_pyengine_installation), you are ready to use Chrono::PyEngine from
Python. To begin:

-   start your editor, for example PyScripter.
-   create a new blank Python script file, for example 'test.py'

Now you can type Python programs in this new Python file, execute it,
save it on disk, etc.

Let's see a first program.

-   First of all, you should use the **import** keyword to specify which
    Python modules must be load and used in your program. Most of the
    core functionalities of Chrono::Engine are in a module called
    **ChronoEngine\_python\_core**, hence write:

~~~~~~~~~~~~~~~{.py}
import ChronoEngine_python_core as chrono
~~~~~~~~~~~~~~~

Note that the *as chrono* is optional: but if you avoid it you must call
all Chrono::Engine functions using the long syntax
ChronoEngine\_python\_core.ChClassFoo..., whereas if you use *as chrono*
you simply rename the namespace and you can type more shortly:
chrono.ChClassFoo...

-   Let's create a 3D vector object:

~~~~~~~~~~~~~~~{.py}
my_vect1 = chrono.ChVectorD()
~~~~~~~~~~~~~~~

(Note that all PyChrono::Engine classes are prefixed by the *chrono*
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
    example let's play with quaternions and matrices:

~~~~~~~~~~~~~{.py}
my_quat = chrono.ChQuaternionD(1,2,3,4)
my_qconjugate = ~my_quat
print ('quat. conjugate  =', my_qconjugate)
print ('quat. dot product=', my_qconjugate ^ my_quat)
print ('quat. product=',     my_qconjugate % my_quat)
ma = chrono.ChMatrixDynamicD(4,4)
ma.FillDiag(-2)
mb = chrono.ChMatrixDynamicD(4,4)
mb.FillElem(10)
mc = (ma-mb)*0.1;   # operator overloading of +,-,* is supported
print (mc);
mr = chrono.ChMatrix33D()
mr.FillDiag(20)
print  (mr*my_vect1);
~~~~~~~~~~~~~

-   If you want to know the list of methods and/or properties that are
    available in a class, you can simply use the code completion feature
    of PyScripter: for example once you type *chrono.* you will see a
    pop-up window with a list of available classes, constants, etc.

<div class="ce-info">
Learn additional lessons by reading the PyChrono::Engine tutorials. 
</div>

<div class="ce-info">
Most classes behave like their C++ counterparts, so you are 
invited to look at the [C++ API documentation](http://api.chrono.projectchrono.org) to understand their features. 
</div>

<div class="ce-warning">
There are also few but important differences between C++ and Python that are worth mentioning, so read also the following section! 
</div>


Differences between Python and C++
----------------------------------

Not all the functionalities of the C++ API can be mapped 1:1 to the
Python API, of course. Python is an interpreted language that differs
from C++ for many reasons, so there are some differences and
limitations, but also advantages. If you already used the C++ API of
Chrono::Engine, you will find that this documentation is useful for
easily jumping into the PyChrono::Engine development.

### Object creation

In C++ you can create objects in two ways: on the stack and on the heap
(the latter is used for *dynamic allocation*). For example,
respectively, this is **object creation in C++ language**

~~~~{.cpp}
chrono::ChSystem  my_system;  // on stack, or..
chrono::ChSystem* my_system_pointer = new chrono::ChSystem();  // on heap 
~~~~

In the second case, in C++ you must remember to deallocate the object
with *delete (my\_system\_pointer)* soon or later. In Python, object
creation is always done in a single way: *objectname =
namespace.classname()* , and it does not require that you remember to
delete an object, because the lifetime of an object is automatically
managed by Python. So **object creation in Python** is simply:

~~~~~~~~~~~~~~~{.py}
my_vect = chrono.ChSystem()
~~~~~~~~~~~~~~~

Note that the = operator in Python does not mean *copy* (except for
simple types such as integers, floats, etc.) but means *assign*, so for
example

~~~~~~~~~~~~~~~{.py}
my_systemA = chrono.ChSystem()
my_systemB = my_systemA      # assign another handle to the same system
my_systemA.SetTol(2)
print (my_systemB.GetTol())
~~~~~~~~~~~~~~~

will output 2, because you created a single ChSystem object, with two
handles pointing to it.

### Templated classes

Currently there is no support for templated clases in Python. On the
other side, there are some templated classes in the C++ API of
Chrono::Engine, expecially for vectors, matrices, etc., because in C++
you might want to create vectors of floats, or doubles, or integers,
etc. by using the <> syntax, as:

~~~~{.cpp}
ChVector<float> my_vect;    // vector of floats
ChVector<double> my_vect;   // vector of double precision floats
ChVector<int> my_vect;      // vector of integers
ChVector<> my_vect;         // default: as ChVector<double>
ChQuaternion<float> my_vect;    // quaternion of floats
ChQuaternion<double> my_vect;   // quaternion of double precision floats
...                             // etc.
~~~~

In PyChrono::Engine, we simply decided to support only the ```<double>```
templated versions, that are used most of the times. Templated classes
are renamed by appending a **D** at the end (to remember that they
represent the ```<double>``` templated versions). In detail, this is a list of
the Python classes that are equivalent to C++ templated classes:

~~~~~~~~~~~~~~~{.py}
chrono.ChVectorD          # as ChVector<double>  in c++
chrono.ChQuaternionD      # as ChQuaternion<double>  in c++
chrono.ChMatrix33D        # as ChMatrix33<double>  in c++
chrono.ChMatrixNMD        # as ChMatrixNM<double>  in c++
chrono.ChMatrixDynamicD   # as ChMatrixDynamic<double>  in c++
chrono.ChMatrixD          # as ChMatrix<double>  in c++
chrono.ChFrameD           # as ChFrame<double>  in c++
chrono.ChFrameMovingD     # as ChFrameMoving<double>  in c++
chrono.ChCoordsysD        # as ChCoordsys<double>  in c++
~~~~~~~~~~~~~~~

There is a (quite limited) support for templates of templates, especially
for the std::vector<> container. The concept is the same: the C++ template 
is translated in a special name in Python. For std::vector, we prepend
the vector_ prefix: 

~~~~~~~~~~~~~~~{.py}
chrono.vector_ChVectorD   # as std::vector<ChVector<double>>  in c++
~~~~~~~~~~~~~~~


### ChVector x y z members

In C++ you access the x y z components of a 3D vector using the 
functions ```x() y() z()```, that return a reference to the components. 
To avoid some issues, we drop the () parentheses and we 
mapped these functions to direct access to ```x y z``` class 
members in Python, so:

~~~~{.cpp}
chrono::ChVector<double>  my_vector;
my_vector.x() = 123;
double sum = my_vector.y() + my_vector.z();
~~~~

becomes

~~~~~~~~~~~~~~~{.py}
chrono.ChVectorD  my_vector
my_vector.x = 123
sum = my_vector.y + my_vector.z
~~~~~~~~~~~~~~~

A similar concept is applied for quaternions components e0 e1 e2 e3, 
eg. one uses myquaternion.e0 in Python, instead of myquaternion.e0() in C++.



### Shared pointers

Except for vectors, matrices, etc., most of the complex objects that you
create with the Chrono::Engine API are managed via C++ shared pointers.
This is the case of ChBody parts, ChLink constraints, etc. Shared
pointers are a C++ technology that allows the user to create objects and
do not worry about deletion, because deletion is managed automatically.
In the Chrono::Engine C++ API such shared pointers are based on
templates; as we said previously templates are not supported in Python,
but this is not an issue, because Chrono::PyEngine **automatically handle objects with shared pointers if necessary**
PyChrono::Engine.

This is an example of **shared pointers in C++** :

~~~~cpp
std::shared_ptr<ChLinkLockRevolute>  my_link_BC(new ChLinkLockRevolute);
~~~~

This is the equivalent syntax **in Python** :

~~~~~~~~~~~~~~~{.py}
my_link_BC = chrono.ChLinkLockRevolute()
~~~~~~~~~~~~~~~


<div class="ce-info">
When you create a shared pointer object in PyChrono::Engine, also 
the referenced object is created. For instance, in the last example, 
a revolute joint is created at the same line. 
If you need other shared pointers to the same object, simply type 
```my_link_other = my_link_BC``` etc. 
</div>



Downcasting and upcasting
-------------------------

#### Upcasting

Upcasting of derived classes to the base class works **automatically**
in Python and does not require intervention. We managed to do this also
for shared pointers. For example *ChSystem.Add()* requires a
(shared)pointer to a *ChPhysicsItem* object, that is a base class, but
you can pass it a *ChBody*, a *ChLinkGears*, etc. that are derived
classes. The same in Python.

#### Downcasting

A bigger problem is the downcasting of base classes to derived classes.
That is, if a function *returns* a pointer to a base class, how to
understand that a specific returned object belongs to a derived class?
At the time of writing, an automatic downcasting is performed only for
classes inherited from ChFunction and ChAsset. Otherwise, one has to
perform manual downcasting using the CastToXXXYYYZZZ() helper functions;
this is a bit similar to the ```dynamic_cast<derived>(base)``` method in C++.
Currently such casting functions are provided for almost all shared
pointers. Use this in Python as in the following example:

~~~~~~~~~~~~~~~{.py}
myvis = chrono.CastToChVisualizationShared(myasset)
print ('Could be cast to visualization object?', !myvis.IsNull())
~~~~~~~~~~~~~~~

### Not supported classes

We managed to map in Python most of the C++ classes of common interest,
but maybe that there are still some classes or functions that are not
yet mapped to Python. This can be understood in the following way.
Functions that are correctly mapped will return objects whose type is
echoed in the interpreter window as:

~~~~~~~~~~~~~~~{.py}
my_system.Get_G_acc()
<ChronoEngine_python_core.ChVectorD; 
proxy of <Swig Object of type 'chrono::ChVector< double > *' at 0x03EDCEA8> >
~~~~~~~~~~~~~~~

A function that is not yet mapped, for instance
ChSystem.GetSystemDescriptor(), will give a shorter echo, that
represents the type information for a pointer to an object that is
unusable in Python:

~~~~~~~~~~~~~~~{.py}
my_system.GetSystemDescriptor()
<Swig Object of type 'chrono::ChSystemDescriptor *' at 0x03EDD800>
~~~~~~~~~~~~~~~

<div class="ce-info">
As the development of PyChrono::Engine proceeds, the latter case will
happen less an less frequently. Tell us if you encounter this type of
problem in some function, so we can fix it.
</div>

Demos and examples
------------------

You can find examples of use in 
[these tutorials](@ref tutorial_table_of_content_chrono_pyengine)