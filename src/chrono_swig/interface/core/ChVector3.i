%{
#include <cstddef>
#include "chrono/core/ChVector3.h"
#include <Eigen/Core>
#ifdef CHRONO_PYTHON_NUMPY
#include <numpy/arrayobject.h>
#endif
%}

%import "ChMatrix.i"

// Hack to avoid problems with .x() .y() .z() that work with references. 
// This is not straightforward in SWIG. So access them as .x .y .z attributes 
// using the following workaround (NOTE! must happen before calling %include)
%include <attribute.i>
%attributeref(chrono::ChVector3<double>, double, x);
%attributeref(chrono::ChVector3<double>, double, y);
%attributeref(chrono::ChVector3<double>, double, z);
%attributeref(chrono::ChVector3<float>, float, x);
%attributeref(chrono::ChVector3<float>, float, y);
%attributeref(chrono::ChVector3<float>, float, z);
%attributeref(chrono::ChVector3<int>, int, x);
%attributeref(chrono::ChVector3<int>, int, y);
%attributeref(chrono::ChVector3<int>, int, z);

%ignore chrono::ChVector3::eigen;

// Parse the header file to generate wrappers
%include "../../../chrono/core/ChVector3.h"  


%template(ChVector3d) chrono::ChVector3<double>; 
%template(ChVector3f) chrono::ChVector3<float>; 
%template(ChVector3i) chrono::ChVector3<int>;

%template(ChWrenchd) chrono::ChWrench<double>; 
%template(ChWrenchf) chrono::ChWrench<float>; 


// Wrap an std::vector<ChVector3d> for use via python or C#
%template(vector_ChVector3d) std::vector<chrono::ChVector3<double>>;
%template(vector_ChVector3f) std::vector<chrono::ChVector3<float>>;
%template(vector_ChVector3i) std::vector<chrono::ChVector3<int>>;

// Wrap an std::vector<std::vector<ChVector3d>> for use via python or C#
// (e.g., for the storage of mesh or hulls into a vector of vectors of ChVector3)
%template(vector_vector_ChVector3d) std::vector<std::vector<chrono::ChVector3<double>>>;

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// Constants seem not to work...
// %constant chrono::ChVector3<double> VNULL = chrono::ChVector3<double>(0,0,0);
// %constant chrono::ChVector3<double> VECT_X= chrono::ChVector3<double>(1,0,0);
// %constant chrono::ChVector3<double> VECT_Y= chrono::ChVector3<double>(0,1,0);
// %constant chrono::ChVector3<double> VECT_Z= chrono::ChVector3<double>(0,0,1);
%extend chrono::ChVector3<double>{
	public:
			// Add function to support python 'print(...)'
		char *__str__() 
			{
				static char temp[256];
				sprintf(temp,"[ %g, %g, %g ]", $self->x(),$self->y(),$self->z());
				return &temp[0];
			}
			// operator  ^  as ^ in c++ 
		double __xor__(const ChVector3<double>& other) const 
			{ 
				return $self->operator^(other);
			}

// NumPy integration: single-call conversion to numpy array
#ifdef CHRONO_PYTHON_NUMPY
		PyObject* to_numpy() {
			npy_intp dims[1] = {3};
			PyObject* array = PyArray_SimpleNew(1, dims, NPY_DOUBLE);
			if (!array) return NULL;
			double* dst = (double*)PyArray_DATA((PyArrayObject*)array);
			const double* src = $self->data();
			std::memcpy(dst, src, 3 * sizeof(double));
			return array;
		}
#endif
};

// This because constants do not work well, so implement them in script-side

#ifdef CHRONO_PYTHON_NUMPY
%pythoncode %{
	def _chvector3_array(self, dtype=None):
		import numpy as np
		a = self.to_numpy()
		return np.asarray(a, dtype=dtype) if dtype is not None else a

	ChVector3d.__array__ = _chvector3_array

	VNULL  = ChVector3d(0,0,0)
	VECT_X = ChVector3d(1,0,0)
	VECT_Y = ChVector3d(0,1,0)
	VECT_Z = ChVector3d(0,0,1)

%}
#endif

#endif             // --------------------------------------------------------------------- PYTHON

// Include global functions not directly accessible because they're defined outside the ChVector3 Class
// TODO: add more. Testing vdot and vcross for now

extern double Vdot(const chrono::ChVector3d& va, const chrono::ChVector3d& vb);
extern chrono::ChVector3d Vcross(const chrono::ChVector3d& va, const chrono::ChVector3d& vb);

%inline %{
    double Vdot(const chrono::ChVector3d& va, const chrono::ChVector3d& vb) {
        return chrono::Vdot(va, vb);
    }
    chrono::ChVector3d Vcross(const chrono::ChVector3d& va, const chrono::ChVector3d& vb) {
        return chrono::Vcross(va, vb);
    }
%}
