%{
#include <cstddef>
/* Includes the header in the wrapper code */
#include "chrono/core/ChVector.h"
#include <Eigen/Core>
%}

%import "ChMatrix.i"

// Hack to avoid problems with .x() .y() .z() that work with references. 
// This is not straightforward in SWIG. So access them as .x .y .z attributes 
// using the following workaround (NOTE! must happen before calling %include)
%include <attribute.i>
%attributeref(chrono::ChVector<double>, double, x);
%attributeref(chrono::ChVector<double>, double, y);
%attributeref(chrono::ChVector<double>, double, z);
%attributeref(chrono::ChVector<float>, float, x);
%attributeref(chrono::ChVector<float>, float, y);
%attributeref(chrono::ChVector<float>, float, z);

%ignore chrono::ChVector::eigen;

/* Parse the header file to generate wrappers */
%include "../../chrono/core/ChVector.h"  


%template(ChVectorD) chrono::ChVector<double>; 
%template(ChVectorF) chrono::ChVector<float>; 
%template(ChVectorI) chrono::ChVector<int>;

// This is needed because a std::vector<ChVector<double>
// might be used  somewhere, and we want to use it via python:
%template(vector_ChVectorD) std::vector< chrono::ChVector<double> >;
%template(vector_ChVectorF) std::vector< chrono::ChVector<float> >;
%template(vector_ChVectorI) std::vector< chrono::ChVector<int> >;

// Constants seem not to work...
// %constant chrono::ChVector<double> VNULL = chrono::ChVector<double>(0,0,0);
// %constant chrono::ChVector<double> VECT_X= chrono::ChVector<double>(1,0,0);
// %constant chrono::ChVector<double> VECT_Y= chrono::ChVector<double>(0,1,0);
// %constant chrono::ChVector<double> VECT_Z= chrono::ChVector<double>(0,0,1);


%extend chrono::ChVector<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
					{
						static char temp[256];
						sprintf(temp,"[ %g, %g, %g ]", $self->x(),$self->y(),$self->z());
						return &temp[0];
					}
					// operator  ^  as ^ in c++ 
			double __xor__(const ChVector<double>& other) const 
					{ 
						return $self->operator^(other);
					}
		};



// This because constants do not work well, so implement them in script-side

%pythoncode %{

	VNULL  = ChVectorD(0,0,0)
	VECT_X = ChVectorD(1,0,0)
	VECT_Y = ChVectorD(0,1,0)
	VECT_Z = ChVectorD(0,0,1)

%}