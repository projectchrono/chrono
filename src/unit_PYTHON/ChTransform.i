%{

/* Includes the header in the wrapper code */
#include "core/ChTransform.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../core/ChTransform.h"


%template(ChTrasformD) chrono::ChTrasform<double>;


