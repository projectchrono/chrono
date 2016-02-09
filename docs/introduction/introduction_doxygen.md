About Doxygen {#introduction_doxygen}
==========================
Chrono is a C++ library of tools for physics simulation (multibody dynamics, kinematics, etc.). This documentation is an important part of it. If you have any questions or suggestions, just send a email to the author of the engine, Alessandro Tasora (tasora (at) deltaknowledge.com).

\section links Links

<A HREF="namespaces.html">Namespaces</A>: An interesting place to start reading
the documentation.<BR>
<A HREF="annotated.html">Class list</A>: List of all classes with descriptions.<BR>
<A HREF="functions.html">Class members</A>: Nice place to find forgotten features.<BR>

Everything in the engine is
placed into the namespace 'chrono'. All Chrono classes and functions should be
accessed with th e:: syntax, as: chrono::[class or functions here] . Of course, in
sake of a more compact syntax, you could avoid all the chrono::... typing by adding at the
beginning of your source code the following statement:

\code
using namespace chrono;
\endcode

There are also other namespaces.
You can find a list of all namespaces with descriptions at the
<A HREF="namespaces.html"> namespaces page</A>.
This is also a good place to start reading the documentation.
If you don't want to write the namespace names all the time, just use all namespaces,
like in this example:
\code
using namespace collision;
using namespace pneumatics;
using namespace geometry;
\endcode

There is a lot more the engine can do, but we hope this gave a short
overview over the basic features of the engine. For some examples, please take
a look into the 'demos' directory of the SDK, and read the tutorials.
