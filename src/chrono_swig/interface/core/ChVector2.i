%{
#include <cstddef>
#include "chrono/core/ChVector2.h"
#include <Eigen/Core>
%}

// Hack to avoid problems with .x() .y() that work with references. 
// This is not straightforward in SWIG. So access them as .x .y attributes 
// using the following workaround (NOTE! must happen before calling %include)
%include <attribute.i>
%attributeref(chrono::ChVector2<double>, double, x);
%attributeref(chrono::ChVector2<double>, double, y);
%attributeref(chrono::ChVector2<float>, float, x);
%attributeref(chrono::ChVector2<float>, float, y);
%attributeref(chrono::ChVector2<int>, int, x);
%attributeref(chrono::ChVector2<int>, int, y);

%ignore chrono::ChVector2::eigen;

// Parse the header file to generate wrappers
%include "../../../chrono/core/ChVector2.h"  

%template(ChVector2d) chrono::ChVector2<double>; 
%template(ChVector2f) chrono::ChVector2<float>; 
%template(ChVector2i) chrono::ChVector2<int>;

// Wrap an std::vector<ChVector2d> for use via python or C#
%template(vector_ChVector2d) std::vector<chrono::ChVector2<double>>;
%template(vector_ChVector2f) std::vector<chrono::ChVector2<float>>;
%template(vector_ChVector2i) std::vector<chrono::ChVector2<int>>;

// Wrap an std::vector<std::vector<ChVector2d>> for use via python or C#
%template(vector_vector_ChVector2d) std::vector<std::vector<chrono::ChVector2<double>>>;

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

%extend chrono::ChVector2<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
					{
						static char temp[256];
						sprintf(temp,"[ %g, %g ]", $self->x(),$self->y());
						return &temp[0];
					}
					// operator  ^  as ^ in c++ 
			double __xor__(const ChVector2<double>& other) const 
					{ 
						return $self->operator^(other);
					}
		};

#endif             // --------------------------------------------------------------------- PYTHON
