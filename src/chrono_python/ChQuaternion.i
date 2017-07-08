%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChQuaternion.h"

%}
 
// Undefine ChApi otherwise SWIG gives a syntax error
#define ChApi  


// HACK to deal with SWIG changing references to pointers
%extend chrono::ChQuaternion<double> {
	double e0() {return self->e0();}
	double e1() {return self->e1();}
	double e2() {return self->e2();}
	double e3() {return self->e3();}
};

%ignore chrono::ChQuaternion<double>::e0();
%ignore chrono::ChQuaternion<double>::e1();
%ignore chrono::ChQuaternion<double>::e2();
%ignore chrono::ChQuaternion<double>::e3();


/* Parse the header file to generate wrappers */
%include "../chrono/core/ChQuaternion.h"  


%template(ChQuaternionD) chrono::ChQuaternion<double>; 
//%template(ChQuaternionF) chrono::ChQuaternion<float>; 



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