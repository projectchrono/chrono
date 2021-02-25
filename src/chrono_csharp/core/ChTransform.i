%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChTransform.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChTransform.h"


%template(ChTransformD) chrono::ChTransform<double>;


