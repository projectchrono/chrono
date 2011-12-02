%{

/* Includes the header in the wrapper code */
#include "core/ChTrasform.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../core/ChTrasform.h"    


%template(ChTrasformD) chrono::ChTrasform<double>;


