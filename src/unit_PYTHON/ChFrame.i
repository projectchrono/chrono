%{

/* Includes the header in the wrapper code */
#include "core/ChFrame.h"

using namespace chrono;

%}
 
/* Parse the header file to generate wrappers */
 %include "../core/ChFrame.h"    



%template(ChFrameD) chrono::ChFrame<double>; 
// %template(ChFrameF) chrono::ChFrame<float>; 



%rename(__rshift__) chrono::ChFrame<double>::operator>>;

%extend chrono::ChVector<double>{
		public:
					// Workaround because the vector >> frame mixed operator (using 'friend')
					// in ChFrame.h is not supported by SWIG
			chrono::ChVector<double> operator>> (const chrono::ChFrame<double>& mframe) const
			{
					return mframe.TrasformLocalToParent(*$self);
			}
			
		};
