/*  */

%include "std_string.i"

%{
/* Includes the header in the wrapper code */
#include "chrono/core/ChException.h"

%}
 
/* Shortcut: let Swig directly parse the header file to generate wrappers */
%include "../../chrono/core/ChException.h"
