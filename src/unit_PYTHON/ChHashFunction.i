/* %module ChronoEngine_PYTHON_mod */

%include "std_string.i"

%{
/* Includes the header in the wrapper code */
#include "core/ChHashFunction.h"

%}
 
/* Shortcut: let Swig directly parse the header file to generate wrappers */
%include "../core/ChHashFunction.h"


%template(HashFunctionInteger) chrono::HashFunction_Generic<int>;
%template(HashFunctionFloat) chrono::HashFunction_Generic<float>;
