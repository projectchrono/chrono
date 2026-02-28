%{

/* Includes the header in the wrapper code */
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrix33.h"
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Macros.h"

using namespace chrono;

#ifdef CHRONO_PYTHON_NUMPY
#include <numpy/arrayobject.h>
#endif

%}

%import "ChQuaternion.i"


%include "../../../chrono/core/ChMatrix33.h"


%template(ChMatrix33d) chrono::ChMatrix33<double>; 

%extend chrono::ChMatrix33<double>{
		public:
			double getitem(int i, int j) {
				return (*$self)(i, j);
				}

			void setitem(int i, int j, double v) {
				(*$self)(i, j) = v;
				}
			const int GetRows() {
				const int r = $self->rows();
				return r;
				}
			const int GetColumns() {
				const int c = $self->cols();
				return c;
				}			
			ChMatrix33<double> __rmul__(double s) {
				ChMatrix33<double> r = (*$self) * s;
				return r;
			}

#ifdef CHRONO_PYTHON_NUMPY
			// Numpy integration: constructor from numpy array
			ChMatrix33(double* a, int rows, int cols) {
				auto* m = new chrono::ChMatrix33<double>();

				for (int i = 0; i < rows; i++) {
					for (int j = 0; j < cols; j++) {
						(*m)(i, j) = a[i*cols + j];
					}
				}
				return m;
  			}
  			// NumPy integration: single-call conversion to numpy array
			PyObject* to_numpy() {
				npy_intp dims[2] = {3, 3};
				PyObject* array = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
				if (!array) return NULL;
				double* dst = (double*)PyArray_DATA((PyArrayObject*)array);
				const double* src = $self->data();
				std::memcpy(dst, src, 9 * sizeof(double));
				return array;
			}
#endif

		/*
					// these functions are also argument-templated, so we need to specify the types
					// ***SWIG template mechanism does not work here for operator() ***
			//%template(operator+) operator+<double>;
			//%template(operator-) operator-<double>;
			//%template(operator*) operator*<double>;
			ChMatrix33<double> operator+(const ChMatrix<double>& matbis) 
						{ return $self->operator+(matbis);};
			ChMatrix33<double> operator-(const ChMatrix<double>& matbis) 
						{ return $self->operator-(matbis);};
			ChMatrix33<double> operator*(const ChMatrix<double>& matbis) 
						{ return $self->operator*(matbis);};

			ChMatrix33<double>(const ChQuaternion<double>& mq){ 
						ChMatrix33<double>* newX = new ChMatrix33<double>();
						newX->SetFromQuaternion(mq);
						return newX;};
			
			//%template(ChMatrix33) ChMatrix33<double>;
			%template(Matr_x_Vect) Matr_x_Vect<double>;
			%template(MatrT_x_Vect) MatrT_x_Vect<double>;
			%template(FastInvert) FastInvert<double>;
			%template(SetFromQuaternion) SetFromQuaternion<double>;
			%template(Set_X_matrix) Set_X_matrix<double>;
			%template(SetFromDirectionAxes) SetFromDirectionAxes<double>;
			%template(SetFromEulerAnglesZXZ) SetFromEulerAnglesZXZ<double>;
			%template(SetFromCardanAnglesZXY) SetFromCardanAnglesZXY<double>;
			%template(SetFromCardanAnglesZYX) SetFromCardanAnglesZYX<double>;
			%template(SetFromCardanAnglesXYZ) SetFromCardanAnglesXYZ<double>;
			%template(SetFromRodriguezParameters) SetFromRodriguezParameters<double>;
			*/
		};

#ifdef SWIGPYTHON  // --------------------------------------------------------------------- PYTHON

// ADD PYTHON CODE

%pythoncode %{

def __matr33_setitem(self,index,vals):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Setting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    self.setitem(index[0],index[1],vals)

def __matr33_getitem(self,index):
    row = index[0];
    col = index[1];
    if row>=self.GetRows() or row <0:
        raise NameError('Bad row. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    if col>=self.GetColumns() or col <0:
        raise NameError('Bad column. Getting value at [{0},{1}] in a {2}x{3} matrix'.format(row,col,self.GetRows(),self.GetColumns()))
    return self.getitem(index[0],index[1])

setattr(ChMatrix33d, "__getitem__", __matr33_getitem)
setattr(ChMatrix33d, "__setitem__", __matr33_setitem)

def SetMatr(self, l_in):
    if len(l_in)>3 or len(l_in[0])>3:
	    raise NameError('Wrong Input List. Must be 3x3')
    for i, li in enumerate(l_in):
        for j, lij in enumerate(li):
          self[i,j] = lij 

def GetMatr(self, ):
    l_out = []
    for i in range(3):
        irow = []
        for j in range(3):
          irow.append(self[i,j])
        l_out.append(irow)
    return l_out

setattr(ChMatrix33d, "SetMatr", SetMatr)
setattr(ChMatrix33d, "GetMatr", GetMatr)

#ifdef CHRONO_PYTHON_NUMPY
def _matr33_array(self, dtype=None):
	import numpy as np
	a = self.to_numpy()
	return np.asarray(a, dtype=dtype) if dtype is not None else a

setattr(ChMatrix33d, "__array__", _matr33_array)
#endif
%}

#endif             // --------------------------------------------------------------------- PYTHON
