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

// Indexing must raise IndexError, not the RuntimeError that the module-wide
// %exception would produce, or Python iteration over the components breaks.
%exception chrono::ChVector2<double>::__getitem__ {
	try {
		$action
	} catch (const std::out_of_range& e) {
		SWIG_exception(SWIG_IndexError, e.what());
	}
}

// Indexing must raise IndexError, not the RuntimeError that the module-wide
// %exception would produce, or Python iteration over the components breaks.
%exception chrono::ChVector2<double>::__setitem__ {
	try {
		$action
	} catch (const std::out_of_range& e) {
		SWIG_exception(SWIG_IndexError, e.what());
	}
}

%extend chrono::ChVector2<double>{
		public:
					// Add function to support python 'print(...)'
			char *__str__() 
					{
						static char temp[256];
						sprintf(temp,"[ %g, %g ]", $self->x(),$self->y());
						return &temp[0];
					}

					// Component access as v[i]. The C++ subscript operator itself is not
					// wrapped (see chrono_ignore_operators.i); these provide the Python form.
			double __getitem__(int i) const {
				if (i < 0) i += 2;
				if (i < 0 || i >= 2) throw std::out_of_range("index out of range");
				return (*$self)[(unsigned)i];
			}

			void __setitem__(int i, double value) {
				if (i < 0) i += 2;
				if (i < 0 || i >= 2) throw std::out_of_range("index out of range");
				(*$self)[(unsigned)i] = value;
			}

			int __len__() const { return 2; }
		};

#endif             // --------------------------------------------------------------------- PYTHON
