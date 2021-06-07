%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChQuaternion.h"
#include <Eigen/Core>
%}
 
 %import "ChMatrix.i"

// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi  


// Hack to avoid problems with .e0() .e1() .e2() .e3 that work with references. 
// This is not straightforward in SWIG. So access them as .e0 .e1 .e2 .e3 attributes 
// using the following workaround (NOTE! must happen before calling %include)
%include <attribute.i>
%attributeref(chrono::ChQuaternion<double>, double, e0);
%attributeref(chrono::ChQuaternion<double>, double, e1);
%attributeref(chrono::ChQuaternion<double>, double, e2);
%attributeref(chrono::ChQuaternion<double>, double, e3);
%attributeref(chrono::ChQuaternion<float>, float, e0);
%attributeref(chrono::ChQuaternion<float>, float, e1);
%attributeref(chrono::ChQuaternion<float>, float, e2);
%attributeref(chrono::ChQuaternion<float>, float, e3);

%import "ChMatrix.i"

%ignore chrono::ChQuaternion::eigen;

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChQuaternion.h"  


%template(ChQuaternionD) chrono::ChQuaternion<double>; 
%template(ChQuaternionF) chrono::ChQuaternion<float>; 

// This is needed because a std::vector<ChQuaternion<double>
// might be used  somewhere, and we want to use it via python:
%template(vector_ChQuaternionD) std::vector< chrono::ChQuaternion<double> >;
%template(vector_ChQuaternionF) std::vector< chrono::ChQuaternion<float> >;


%extend chrono::ChQuaternion<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
					{
						static char temp[256];
						sprintf(temp,"[ %g, %g, %g, %g ]", $self->e0(),$self->e1(),$self->e2(),$self->e3());
						return &temp[0];
					}
					// operator  ~  as ! in c++ 
			ChQuaternion<double> __invert__() const  
					{
						return $self->operator!();
					}
					// operator  ^  as ^ in c++ 
			double __xor__(const ChQuaternion<double>& other) const 
					{ 
						return $self->operator^(other);
					}
		};


// This because constants do not work well, so implement them in script-side

%pythoncode %{

	QNULL  = ChQuaternionD(0,0,0,0)
	QUNIT  = ChQuaternionD(1,0,0,0)
%}