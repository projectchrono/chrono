%{
#include "chrono/core/ChFrame.h"
%}
 
%import "ChMatrix33.i" 

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"    

%shared_ptr(chrono::ChFrame<double>)

%template(ChFramed) chrono::ChFrame<double>; 
// %template(ChFramef) chrono::ChFrame<float>; 

//%rename(__rshift__) chrono::ChFrame<double>::operator>>;

%extend chrono::ChVector3<double>{
		public:
					// Workaround because mixed 'out of class' operators  
                    // not supported in Swig
			chrono::ChVector3<double> operator>> (chrono::ChFrame<double>& mframe) const
			{
					return mframe.TransformPointLocalToParent(*$self);
			}
			
		};
