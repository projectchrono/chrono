%{

/* Includes the header in the wrapper code */
#include "core/ChFrame.h"

using namespace chrono;

%}
 
// Forward ref
//%import "ChMatrix.i"   //***THIS CONFUSES SWIG!
//class chrono::ChMatrix33<Real>; //***THIS DOES NOT WORK!
//namespace chrono {  //***THESE DO NOT WORK!
	//class ChMatrix33<double>;
	//%template(ChMatrix33D) chrono::ChMatrix33<double>; 
	//template <class Real> class ChMatrix33<Real>;
//}

/* Parse the header file to generate wrappers */
%include "../chrono/core/ChFrame.h"    



%template(ChFrameD) chrono::ChFrame<double>; 
// %template(ChFrameF) chrono::ChFrame<float>; 



%rename(__rshift__) chrono::ChFrame<double>::operator>>;

%extend chrono::ChVector<double>{
		public:
					// Workaround because the vector >> frame mixed operator (using 'friend')
					// in ChFrame.h is not supported by SWIG
			chrono::ChVector<double> operator>> (const chrono::ChFrame<double>& mframe) const
			{
					return mframe.TransformLocalToParent(*$self);
			}
			
		};
