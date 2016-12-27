%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChCoordsys.h"

%}
 
/* Parse the header file to generate wrappers */
%include "../chrono/core/ChCoordsys.h" 


%template(ChCoordsysD) chrono::ChCoordsys<double>; 
//%template(ChCoordsysF) chrono::ChCoordsys<float>; 


// This because constants do not work well, so implement them in script-side

%pythoncode %{
    
    CSYSNULL = ChCoordsysD(VNULL,QNULL)
    CSYSNORM = ChCoordsysD(VNULL,QUNIT)
%}