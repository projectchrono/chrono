%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChCoordsys.h"

%}
 
 %import "ChQuaternion.i"

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChCoordsys.h" 


%template(ChCoordsysD) chrono::ChCoordsys<double>; 
//%template(ChCoordsysF) chrono::ChCoordsys<float>; 


// This because constants do not work well, so implement them in script-side
/*
%pythoncode %{
    
    CSYSNULL = ChCoordsysD(VNULL,QNULL)
    CSYSNORM = ChCoordsysD(VNULL,QUNIT)
%}*/

%extend chrono::ChVector<double>{
        // Workaround because mixed 'out of class' operators  
        // not supported in Swig
        public:
            chrono::ChVector<double> operator>> (chrono::ChCoordsys<double>& msys) 
            {
                return msys.TransformPointLocalToParent(*$self);
            }
        };