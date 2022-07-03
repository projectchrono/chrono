%{
#include "chrono/core/ChFrame.h"
%}
 
%import "ChMatrix33.i" 

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChFrame.h"    

%template(ChFrameD) chrono::ChFrame<double>; 
// %template(ChFrameF) chrono::ChFrame<float>; 

//%rename(__rshift__) chrono::ChFrame<double>::operator>>;

%extend chrono::ChVector<double>{
		public:
					// Workaround because mixed 'out of class' operators  
                    // not supported in Swig
			chrono::ChVector<double> operator>> (chrono::ChFrame<double>& mframe) const
			{
					return mframe.TransformLocalToParent(*$self);
			}
			
		};
