%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChRotation.h"
#include <Eigen/Core>

#ifdef CHRONO_PYTHON_NUMPY
#include <numpy/arrayobject.h>
#endif
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
%rename(conj) chrono::ChQuaternion::operator!; 

/* Parse the header file to generate wrappers */
%include "../../../chrono/core/ChQuaternion.h"  
%include "../../../chrono/core/ChRotation.h"  


%template(ChQuaterniond) chrono::ChQuaternion<double>; 
%template(ChQuaternionf) chrono::ChQuaternion<float>; 

%extend chrono::ChQuaternion<double>{
		public:
			ChQuaternion<double> __rmul__(const chrono::ChQuaternion<double>& q) {
				ChQuaternion<double> r = (*$self) * q;
				return r;
			}
		};

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// This is needed because a std::vector<ChQuaternion<double>
// might be used  somewhere, and we want to use it via python:
%template(vector_ChQuaterniond) std::vector< chrono::ChQuaternion<double> >;
%template(vector_ChQuaternionf) std::vector< chrono::ChQuaternion<float> >;


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

// NumPy integration: single-call conversion to numpy array
#ifdef CHRONO_PYTHON_NUMPY
		PyObject* to_numpy() {
			npy_intp dims[1] = {4};
			PyObject* array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
			if (!array) return NULL;
			double* dst = (double*)PyArray_DATA((PyArrayObject*)array);
			const double* src = $self->data();
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
			return array;
		}
#endif
};


// This because constants do not work well, so implement them in script-side

%pythoncode %{

	QNULL  = ChQuaterniond(0,0,0,0)
	QUNIT  = ChQuaterniond(1,0,0,0)

	def _chquat_array(self, dtype=None):
		import numpy as np
		if hasattr(self, 'to_numpy'):
			a = self.to_numpy()
			return np.asarray(a, dtype=dtype) if dtype is not None else a
		return np.array([self.e0(), self.e1(), self.e2(), self.e3()], dtype=dtype)

	ChQuaterniond.__array__ = _chquat_array

%}

#endif             // --------------------------------------------------------------------- PYTHON
