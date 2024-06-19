%{
#include "chrono/core/ChCoordsys.h"
%}
 
 %import "ChQuaternion.i"

// Parse the header file to generate wrappers */
%include "../../../chrono/core/ChCoordsys.h" 


%template(ChCoordsysd) chrono::ChCoordsys<double>; 
//%template(ChCoordsysf) chrono::ChCoordsys<float>; 

// This is needed because a std::vector<ChCoordsys<>> might be used somewhere,
// and we want to use it via python or C#
%template(vector_ChCoordsysd) std::vector< chrono::ChCoordsys<double> >;
//%template(vector_ChCoordsysf) std::vector< chrono::ChCoordsys<float> >;

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// This because constants do not work well, so implement them in script-side
/*
%pythoncode %{
    
    CSYSNULL = ChCoordsysd(VNULL,QNULL)
    CSYSNORM = ChCoordsysd(VNULL,QUNIT)
%}*/

%extend chrono::ChVector3<double>{
        // Workaround because mixed 'out of class' operators  
        // not supported in Swig
        public:
            chrono::ChVector3<double> operator>> (chrono::ChCoordsys<double>& msys) 
            {
                return msys.TransformPointLocalToParent(*$self);
            }
        };

#endif             // --------------------------------------------------------------------- PYTHON
