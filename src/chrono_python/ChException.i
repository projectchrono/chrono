/* %module ChronoEngine_python_mod */

%include "std_string.i"

%{
/* Includes the header in the wrapper code */
#include "core/ChException.h"

%}
 
/* Shortcut: let Swig directly parse the header file to generate wrappers */
%include "../chrono/core/ChException.h"
