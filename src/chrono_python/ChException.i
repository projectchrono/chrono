/* %module ChronoEngine_PYTHON_mod */

%include "std_string.i"

%{
/* Includes the header in the wrapper code */
#include "core/ChException.h"

%}
 
/* Shortcut: let Swig directly parse the header file to generate wrappers */
%include "../core/ChException.h"
